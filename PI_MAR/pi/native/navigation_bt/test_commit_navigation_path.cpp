#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <limits>
#include <iostream>
#include <stdexcept>

void require(bool value) { if (!value) throw std::runtime_error("path transaction regression"); }
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  BT::BehaviorTreeFactory factory; factory.registerFromPlugin(argv[1]);
  auto board = BT::Blackboard::create();
  nav_msgs::msg::Path live; live.header.frame_id="map"; live.poses.resize(2);
  live.poses[1].pose.position.x=1.; board->set("path",live);
  nav_msgs::msg::Path candidate; board->set("candidate",candidate);
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<CommitNavigationPath candidate='{candidate}' path='{path}'/>"
    "</BehaviorTree></root>",board);
  require(tree.tickRoot()==BT::NodeStatus::FAILURE);
  require(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x==1.);
  candidate=live; candidate.poses[1].pose.position.x=2.; board->set("candidate",candidate);
  require(tree.tickRoot()==BT::NodeStatus::SUCCESS);
  require(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x==2.);
  candidate.poses[1].pose.position.x=std::numeric_limits<double>::quiet_NaN();
  board->set("candidate",candidate); require(tree.tickRoot()==BT::NodeStatus::FAILURE);
  require(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x==2.);
  // A planner may return one valid pose when already at the destination.
  candidate=live;candidate.poses.resize(1);candidate.poses[0].pose.position.x=3.;
  board->set("candidate",candidate);require(tree.tickRoot()==BT::NodeStatus::SUCCESS);
  require(board->get<nav_msgs::msg::Path>("path").poses.size()==1);
  std::cout << "PASS: failed/invalid candidates preserve the last valid typed path\n";
}
