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
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.position.x=3.;
  board->set("goal",goal);board->set("committed_goal",goal);
  auto matches=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<NavigationPathMatchesGoal path='{path}' goal='{goal}' committed_goal='{committed_goal}'/>"
    "</BehaviorTree></root>",board);
  require(matches.tickRoot()==BT::NodeStatus::SUCCESS);
  goal.pose.position.x=4.;board->set("goal",goal);
  require(matches.tickRoot()==BT::NodeStatus::FAILURE);
  goal.pose.position.x=3.01;board->set("goal",goal);
  require(matches.tickRoot()==BT::NodeStatus::FAILURE); // nearby new endpoint is still a new goal
  goal.pose.position.x=3.;goal.header.frame_id="odom";board->set("goal",goal);
  require(matches.tickRoot()==BT::NodeStatus::FAILURE);
  goal.header.frame_id="map";goal.pose.position.x=std::numeric_limits<double>::quiet_NaN();board->set("goal",goal);
  require(matches.tickRoot()==BT::NodeStatus::FAILURE);
  int computations=0;bool clear=true;
  factory.registerSimpleCondition("TestPathClear",[&](BT::TreeNode&) {
    return clear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  factory.registerSimpleAction("TestComputePath",[&](BT::TreeNode&) {
    ++computations;
    auto next=live;next.poses.back()=board->get<geometry_msgs::msg::PoseStamped>("goal");
    board->set("candidate",next);return BT::NodeStatus::SUCCESS;
  });
  goal.pose.position.x=4.;board->set("goal",goal);
  auto policy=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><Fallback>"
    "<Sequence><NavigationPathMatchesGoal path='{path}' goal='{goal}' committed_goal='{committed_goal}'/>"
    "<TestPathClear/></Sequence>"
    "<Sequence><TestComputePath/><CommitNavigationPath candidate='{candidate}' path='{path}'"
    " goal='{goal}' committed_goal='{committed_goal}'/></Sequence>"
    "</Fallback></BehaviorTree></root>",board);
  require(policy.tickRoot()==BT::NodeStatus::SUCCESS);require(computations==1);
  for(int i=0;i<28;++i)require(policy.tickRoot()==BT::NodeStatus::SUCCESS);
  require(computations==1); // 28 clear-path checks do not replace the controller path
  clear=false;require(policy.tickRoot()==BT::NodeStatus::SUCCESS);require(computations==2);
  clear=true;goal.pose.position.x=4.01;board->set("goal",goal);
  require(policy.tickRoot()==BT::NodeStatus::SUCCESS);require(computations==3);
  std::cout << "PASS: failed/invalid candidates preserve the last valid typed path\n";
}
