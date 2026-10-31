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
            BT::OutputPort<nav_msgs::msg::Path>("path")};
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
    return BT::NodeStatus::SUCCESS;
  }
};
BT_REGISTER_NODES(factory) { factory.registerNodeType<CommitNavigationPath>("CommitNavigationPath"); }
