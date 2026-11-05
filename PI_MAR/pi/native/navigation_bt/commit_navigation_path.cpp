// Keep planner output transactional: a failed replan cannot erase the live path.
#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <cmath>

class CommitNavigationPath : public BT::SyncActionNode {
 public:
  CommitNavigationPath(const std::string& name, const BT::NodeConfiguration& cfg)
      : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("candidate"),
            BT::OutputPort<nav_msgs::msg::Path>("path"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("committed_goal")};
  }
  BT::NodeStatus tick() override {
    nav_msgs::msg::Path next;
    if (!getInput("candidate", next) || next.poses.empty() || next.header.frame_id.empty())
      return BT::NodeStatus::FAILURE;
    for (const auto& p : next.poses) {
      const auto& x = p.pose.position;
      const auto& q = p.pose.orientation;
      if (!std::isfinite(x.x) || !std::isfinite(x.y) || !std::isfinite(x.z) ||
          !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
        return BT::NodeStatus::FAILURE;
    }
    setOutput("path", next);
    geometry_msgs::msg::PoseStamped goal;
    if (getInput("goal", goal)) setOutput("committed_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }
};
// The BT blackboard may outlive a NavigateToPose goal. Compare the actual
// target instead of assuming a retained, collision-free path belongs to it.
class NavigationPathMatchesGoal : public BT::ConditionNode {
 public:
  NavigationPathMatchesGoal(const std::string& name, const BT::NodeConfiguration& cfg)
      : BT::ConditionNode(name, cfg) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("committed_goal")};
  }
  BT::NodeStatus tick() override {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped goal, committed_goal;
    if (!getInput("path", path) || !getInput("goal", goal) ||
        !getInput("committed_goal", committed_goal) ||
        committed_goal.header.frame_id != goal.header.frame_id || committed_goal.pose != goal.pose || path.poses.empty() ||
        path.header.frame_id.empty() || path.header.frame_id != goal.header.frame_id)
      return BT::NodeStatus::FAILURE;
    const auto& last = path.poses.back().pose.position;
    const auto& target = goal.pose.position;
    if (!std::isfinite(last.x) || !std::isfinite(last.y) ||
        !std::isfinite(target.x) || !std::isfinite(target.y))
      return BT::NodeStatus::FAILURE;
    return std::hypot(last.x-target.x, last.y-target.y) <= 0.075 ?
        BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<CommitNavigationPath>("CommitNavigationPath");
  factory.registerNodeType<NavigationPathMatchesGoal>("NavigationPathMatchesGoal");
}
