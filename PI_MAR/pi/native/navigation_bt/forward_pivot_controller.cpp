// Follow collision-checked forward sections without looking past their pivots.
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "footprint_sweep.hpp"

namespace mar_navigation
{
namespace
{
constexpr double kSamePosition = 1e-5;

double distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

// RPP must reach an intermediate corner before rotating. The server's real goal
// checker still sees only the complete destination; its tolerance is unchanged.
class CornerGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b, const geometry_msgs::msg::Twist &) override
  {
    return distance(a, b) < 0.005;
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose,
    geometry_msgs::msg::Twist & velocity) override
  {
    pose = geometry_msgs::msg::Pose();
    velocity = geometry_msgs::msg::Twist();
    pose.position.x = pose.position.y = 0.005;
    pose.orientation.w = 1.0;
    return true;
  }
};
}  // namespace

class ForwardPivotController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
  using RPP = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
  struct Section
  {
    std::size_t first;
    std::size_t last;
    bool pivot;
    double turn;
  };

public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    RPP::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    if (!node) {throw nav2_core::PlannerException("ForwardPivot: node unavailable");}
    const auto parameter = [&](const std::string & key, double value) {
        nav2_util::declare_parameter_if_not_declared(
          node, name + "." + key, rclcpp::ParameterValue(value));
        return node->get_parameter(name + "." + key).as_double();
      };
    pivot_xy_tolerance_ = parameter("pivot_xy_tolerance", 0.015);
    pivot_heading_tolerance_ = parameter("pivot_heading_tolerance", 0.04);
    path_entry_heading_tolerance_ = parameter("path_entry_heading_tolerance", 0.12);
    pivot_heading_gain_ = parameter("pivot_heading_gain", 1.8);
    pivot_heading_damping_ = parameter("pivot_heading_damping", 0.3);
    pivot_settle_heading_tolerance_ = parameter("pivot_settle_heading_tolerance", 0.1);
    pivot_reanchor_tolerance_ = parameter("pivot_reanchor_tolerance", 0.06);
    linear_deceleration_ = parameter("pivot_linear_deceleration", 0.6);
    if (!(pivot_xy_tolerance_ > 0.0 && pivot_xy_tolerance_ <= 0.05) ||
      !(pivot_heading_tolerance_ > 0.0 && pivot_heading_tolerance_ <= 0.2) ||
      !(path_entry_heading_tolerance_ >= pivot_heading_tolerance_ &&
      path_entry_heading_tolerance_ <= 0.3) ||
      !(pivot_heading_gain_ > 0.0 && pivot_heading_gain_ <= 5.0) ||
      !(pivot_heading_damping_ >= 0.0 && pivot_heading_damping_ <= 2.0) ||
      !(pivot_settle_heading_tolerance_ >= pivot_heading_tolerance_ &&
      pivot_settle_heading_tolerance_ <= 0.3) ||
      !(pivot_reanchor_tolerance_ >= pivot_xy_tolerance_ &&
      pivot_reanchor_tolerance_ <= 0.1) ||
      !(linear_deceleration_ > 0.0) || !std::isfinite(linear_deceleration_))
    {
      throw nav2_core::PlannerException("ForwardPivot: invalid corner parameters");
    }
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    std::lock_guard<std::mutex> lock(section_mutex_);
    if (path.poses.empty()) {
      throw nav2_core::PlannerException("ForwardPivot: empty path");
    }
    nav_msgs::msg::Path cleaned;
    cleaned.header = path.header;
    if (cleaned.header.frame_id.empty()) {
      throw nav2_core::PlannerException("ForwardPivot: path frame missing");
    }
    for (auto pose : path.poses) {
      if (pose.header.frame_id.empty()) {pose.header.frame_id = cleaned.header.frame_id;}
      if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
        !std::isfinite(tf2::getYaw(pose.pose.orientation)) ||
        pose.header.frame_id != cleaned.header.frame_id)
      {
        throw nav2_core::PlannerException("ForwardPivot: invalid path pose or frame");
      }
      if (!cleaned.poses.empty() &&
        distance(cleaned.poses.back().pose, pose.pose) <= kSamePosition &&
        std::abs(angles::shortest_angular_distance(
          tf2::getYaw(cleaned.poses.back().pose.orientation),
          tf2::getYaw(pose.pose.orientation))) <= 1e-6) {continue;}
      cleaned.poses.push_back(pose);
    }
    // Re-publication of the committed path must not restart an unfinished turn.
    if (equivalent(cleaned, full_plan_) && section_ < sections_.size()) {return;}
    full_plan_ = std::move(cleaned);
    sections_.clear();
    std::size_t i = 0;
    while (i + 1 < full_plan_.poses.size()) {
      const bool pivot = pivotEdge(i);
      std::size_t last = i + 1;
      double turn = yawDelta(i, last);
      while (last + 1 < full_plan_.poses.size()) {
        if (pivotEdge(last) != pivot) {break;}
        const double next_turn = yawDelta(last, last + 1);
        if (pivot && turn * next_turn < 0.0) {break;}
        if (pivot) {turn += next_turn;}
        ++last;
      }
      sections_.push_back({i, last, pivot, pivot ? turn : 0.0});
      i = last;
    }
    if (sections_.empty()) {sections_.push_back({0, 0, false, 0.0});}
    section_ = 0;
    section_loaded_ = false;
    pivot_started_ = false;
    translation_aligned_ = false;
    pivot_settling_ = false;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override
  {
    std::lock_guard<std::mutex> lock(section_mutex_);
    if (sections_.empty()) {throw nav2_core::PlannerException("ForwardPivot: no plan");}
    if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
      !std::isfinite(velocity.angular.z))
    {
      throw nav2_core::PlannerException("ForwardPivot: invalid odometry velocity");
    }
    geometry_msgs::msg::PoseStamped in_plan;
    if (!transformPose(full_plan_.header.frame_id, pose, in_plan)) {
      throw nav2_core::PlannerException("ForwardPivot: plan TF unavailable");
    }
    // At most skip already-completed sections. A returned command never mixes
    // translation with a planned pivot, and never introduces lateral movement.
    for (std::size_t skipped = 0; skipped <= sections_.size(); ++skipped) {
      if (section_ >= sections_.size()) {return stopped(pose);}
      const auto & current = sections_[section_];
      const auto & end = full_plan_.poses[current.last].pose;
      if (current.pivot) {
        // Translation still reaches its corner within pivot_xy_tolerance_.
        // Normal centimetre-scale map/odom corrections during a turn need not
        // discard the whole plan: validate the full remaining turn at the
        // actual observed pose instead. Larger shifts require a new plan.
        if (distance(in_plan.pose, end) > pivot_reanchor_tolerance_) {
          throw nav2_core::PlannerException("ForwardPivot: pivot position changed; replan required");
        }
        // A small entry correction on a new plan is tracked by RPP, rather
        // than creating another stop/turn. Interior corners remain pivots.
        if (section_ == 0 && sections_.size() > 1 &&
          std::abs(current.turn) <= path_entry_heading_tolerance_ &&
          std::abs(angles::shortest_angular_distance(tf2::getYaw(in_plan.pose.orientation),
            tf2::getYaw(end.orientation))) <= path_entry_heading_tolerance_)
        {
          const double correction = angles::shortest_angular_distance(
            tf2::getYaw(in_plan.pose.orientation), tf2::getYaw(end.orientation));
          ensureTurnClear(pose, correction, velocity, 0.0);
          advance();
          continue;
        }
        if (std::hypot(velocity.linear.x, velocity.linear.y) > 0.015) {
          return stopped(pose);
        }
        const double yaw = tf2::getYaw(in_plan.pose.orientation);
        if (!pivot_started_) {
          remaining_turn_ = current.turn - angles::shortest_angular_distance(
            tf2::getYaw(full_plan_.poses[current.first].pose.orientation), yaw);
          previous_yaw_ = yaw;
          pivot_started_ = true;
        } else {
          remaining_turn_ -= angles::shortest_angular_distance(previous_yaw_, yaw);
          previous_yaw_ = yaw;
        }
        if (std::abs(remaining_turn_) <= pivot_heading_tolerance_) {pivot_settling_ = true;}
        if (pivot_settling_) {
          // Do not immediately reverse a moving robot after crossing the
          // target. Let the downstream smoother stop it inside a wider settle
          // band, then make a proportional correction only if still needed.
          if (std::abs(remaining_turn_) <= pivot_settle_heading_tolerance_ &&
            std::abs(velocity.angular.z) > 0.05)
          {
            ensureTurnClear(pose, remaining_turn_, velocity, 0.0);
            return stopped(pose);
          }
          if (std::abs(remaining_turn_) <= pivot_heading_tolerance_ &&
            std::abs(velocity.angular.z) <= 0.05)
          {
            ensureTurnClear(pose, remaining_turn_, velocity, 0.0);
            advance();
            continue;
          }
          pivot_settling_ = false;
        }
        return pivotCommand(pose, velocity, remaining_turn_);
      }

      const bool intermediate = section_ + 1 < sections_.size();
      const double to_end = distance(in_plan.pose, end);
      if (intermediate && to_end <= pivot_xy_tolerance_) {
        advance();
        continue;
      }
      if (!section_loaded_) {
        nav_msgs::msg::Path section_path;
        section_path.header = full_plan_.header;
        section_path.poses.assign(full_plan_.poses.begin() + current.first,
          full_plan_.poses.begin() + current.last + 1);
        RPP::setPlan(section_path);
        section_loaded_ = true;
      }
      if (!translation_aligned_ && current.last > current.first) {
        const double yaw = tf2::getYaw(in_plan.pose.orientation);
        double heading = yaw;
        double best_distance = std::numeric_limits<double>::infinity();
        double best_alignment = std::numeric_limits<double>::infinity();
        for (std::size_t edge = current.first; edge < current.last; ++edge) {
          const auto & a = full_plan_.poses[edge].pose.position;
          const auto & b = full_plan_.poses[edge + 1].pose.position;
          const double dx = b.x - a.x, dy = b.y - a.y;
          const double length_squared = dx * dx + dy * dy;
          if (length_squared <= kSamePosition * kSamePosition) {continue;}
          const double t = std::clamp(((in_plan.pose.position.x - a.x) * dx +
              (in_plan.pose.position.y - a.y) * dy) / length_squared, 0.0, 1.0);
          const double error_x = in_plan.pose.position.x - (a.x + t * dx);
          const double error_y = in_plan.pose.position.y - (a.y + t * dy);
          const double distance_squared = error_x * error_x + error_y * error_y;
          const double candidate = std::atan2(dy, dx);
          const double alignment = std::abs(angles::shortest_angular_distance(yaw, candidate));
          if (distance_squared < best_distance - 1e-10 ||
            (std::abs(distance_squared - best_distance) <= 1e-10 && alignment < best_alignment))
          {
            best_distance = distance_squared;
            best_alignment = alignment;
            heading = candidate;
          }
        }
        // Re-entering a retained path halfway through an arc must use the
        // nearest local tangent, rather than rotating toward its old start or
        // its distant endpoint. RPP owns carrot/curvature corrections afterward.
        const double error = angles::shortest_angular_distance(
          yaw, heading);
        // A changed replan can add a centimetre-scale connector with a few
        // degrees of heading correction. RPP can track that safely; forcing a
        // full stop down to the stricter pivot tolerance would interrupt every
        // new path. Explicit planned pivots retain their exact turn tolerance.
        if (std::abs(error) > path_entry_heading_tolerance_)
        {
          if (std::hypot(velocity.linear.x, velocity.linear.y) > 0.015) {
            return stopped(pose);
          }
          return pivotCommand(pose, velocity, error);
        }
        translation_aligned_ = true;
      }
      auto command = RPP::computeVelocityCommands(
        pose, velocity, intermediate ? &corner_goal_checker_ : goal_checker);
      command.twist.linear.y = 0.0;
      command.twist.linear.x = std::max(0.0, command.twist.linear.x);
      if (intermediate && command.twist.linear.x > 0.0) {
        // RPP retains its normal collision/cost regulation. This additional
        // braking bound prevents a short section overshooting its pivot.
        const double braking_distance = std::max(0.0, to_end - 0.005);
        const double old_speed = command.twist.linear.x;
        command.twist.linear.x = std::min(old_speed,
            std::sqrt(2.0 * linear_deceleration_ * braking_distance));
        // Preserve the collision-checked RPP curvature while braking. Clamping
        // linear velocity alone would turn more sharply than the checked arc.
        command.twist.angular.z *= command.twist.linear.x / old_speed;
      }
      return command;
    }
    throw nav2_core::PlannerException("ForwardPivot: invalid section progress");
  }

private:
  double yawDelta(std::size_t first, std::size_t last) const
  {
    return angles::shortest_angular_distance(
      tf2::getYaw(full_plan_.poses[first].pose.orientation),
      tf2::getYaw(full_plan_.poses[last].pose.orientation));
  }
  bool pivotEdge(std::size_t first) const
  {
    return distance(full_plan_.poses[first].pose, full_plan_.poses[first + 1].pose) <=
           kSamePosition && std::abs(yawDelta(first, first + 1)) > 1e-6;
  }
  static bool equivalent(const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b)
  {
    if (a.header.frame_id != b.header.frame_id || a.poses.size() != b.poses.size()) {return false;}
    for (std::size_t i = 0; i < a.poses.size(); ++i) {
      if (distance(a.poses[i].pose, b.poses[i].pose) > 1e-6 ||
        std::abs(angles::shortest_angular_distance(tf2::getYaw(a.poses[i].pose.orientation),
          tf2::getYaw(b.poses[i].pose.orientation))) > 1e-6) {return false;}
    }
    return true;
  }
  void advance()
  {
    ++section_;
    section_loaded_ = false;
    pivot_started_ = false;
    translation_aligned_ = false;
    pivot_settling_ = false;
  }
  geometry_msgs::msg::TwistStamped stopped(const geometry_msgs::msg::PoseStamped & pose)
  {
    geometry_msgs::msg::TwistStamped result;
    result.header = pose.header;
    return result;
  }

  void ensureTurnClear(const geometry_msgs::msg::PoseStamped & pose,
    double turn, const geometry_msgs::msg::Twist & velocity, double next_command)
  {
    geometry_msgs::msg::PoseStamped in_costmap;
    if (!transformPose(costmap_ros_->getGlobalFrameID(), pose, in_costmap)) {
      throw nav2_core::PlannerException("ForwardPivot: local costmap TF unavailable");
    }
    std::unique_ptr<FootprintSweep> snapshot;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
      snapshot = std::make_unique<FootprintSweep>(*costmap_, costmap_ros_->getRobotFootprint());
    }
    const SE2Pose start{in_costmap.pose.position.x, in_costmap.pose.position.y,
      tf2::getYaw(in_costmap.pose.orientation)};
    if (!snapshot->sweepClear(start, start, turn)) {
      throw nav2_core::PlannerException("ForwardPivot: remaining turn footprint blocked");
    }
    // Check both measured momentum and the outgoing command. In particular,
    // a reversal must not assume the current rotation stops immediately.
    for (const double speed : {velocity.angular.z, next_command}) {
      const double braking_turn = speed * std::abs(speed) /
        (2.0 * std::max(0.01, max_angular_accel_));
      if ((braking_turn * turn < 0.0 || std::abs(braking_turn) > std::abs(turn)) &&
        !snapshot->sweepClear(start, start, braking_turn))
      {
        throw nav2_core::PlannerException("ForwardPivot: braking turn footprint blocked");
      }
    }
  }
  geometry_msgs::msg::TwistStamped pivotCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, double remaining)
  {
    const double acceleration = std::max(0.01, max_angular_accel_);
    const double limit = std::abs(rotate_to_heading_angular_vel_);
    const double braking_limit = std::sqrt(2.0 * acceleration * std::abs(remaining));
    // A sqrt-only turn law requests 0.6 rad/s for only 0.1 rad of error and
    // reverses aggressively after overshoot. Reduce demand with heading error
    // and measured momentum instead. Only the downstream OPEN_LOOP Nav2
    // velocity smoother owns acceleration, avoiding two delayed ramp stages.
    const double direction = remaining >= 0.0 ? 1.0 : -1.0;
    const double demand = pivot_heading_gain_ * remaining -
      pivot_heading_damping_ * velocity.angular.z;
    const double next = direction * std::max(0.0,
        std::min({limit, braking_limit, direction * demand}));
    ensureTurnClear(pose, remaining, velocity, next);
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.angular.z = next;
    return command;
  }

  std::mutex section_mutex_;
  nav_msgs::msg::Path full_plan_;
  std::vector<Section> sections_;
  std::size_t section_{0};
  bool section_loaded_{false}, pivot_started_{false}, translation_aligned_{false};
  double previous_yaw_{0.0}, remaining_turn_{0.0};
  double pivot_xy_tolerance_{0.015}, pivot_heading_tolerance_{0.04}, linear_deceleration_{0.6};
  double path_entry_heading_tolerance_{0.12};
  bool pivot_settling_{false};
  double pivot_heading_gain_{1.8}, pivot_heading_damping_{0.3};
  double pivot_settle_heading_tolerance_{0.1};
  double pivot_reanchor_tolerance_{0.06};
  CornerGoalChecker corner_goal_checker_;
};
}  // namespace mar_navigation

PLUGINLIB_EXPORT_CLASS(mar_navigation::ForwardPivotController, nav2_core::Controller)
