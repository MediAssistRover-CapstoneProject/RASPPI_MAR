#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <vector>

void require(bool value) { if (!value) throw std::runtime_error("path transaction regression"); }

// Exercise the production XML with real Nav2 control/decorator nodes and the
// native transaction plugin. Only action-server calls are replaced; no ROS
// node, executor or motor publisher is created by this test.
struct NavigationScenario {
  int computations=0, follows=0, waits=0;
  bool planning=false, fail_plan=false, invalid_plan=false;
  double robot_x=0.;
  std::vector<BT::NodeStatus> follow_results;
  std::vector<double> followed_starts;
};

class TestPlanner : public BT::StatefulActionNode {
 public:
  TestPlanner(const std::string& name, const BT::NodeConfiguration& config, NavigationScenario& scenario)
      : BT::StatefulActionNode(name, config), scenario_(scenario) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::OutputPort<nav_msgs::msg::Path>("path"), BT::InputPort<std::string>("planner_id")};
  }
  BT::NodeStatus onStart() override {
    require(!scenario_.planning); scenario_.planning=true; ++scenario_.computations;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    scenario_.planning=false;
    nav_msgs::msg::Path next;
    if (scenario_.fail_plan) {
      setOutput("path", next);  // A failed candidate must not erase the committed path.
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped goal;
    require(static_cast<bool>(getInput("goal", goal)));
    next.header=goal.header; next.poses.resize(2);
    next.poses.front().header=goal.header;
    next.poses.front().pose.position.x=scenario_.robot_x;
    next.poses.back()=goal;
    if (scenario_.invalid_plan) next.poses.front().pose.position.x=std::numeric_limits<double>::quiet_NaN();
    setOutput("path", next); return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override { scenario_.planning=false; }
 private:
  NavigationScenario& scenario_;
};

class TestFollower : public BT::StatefulActionNode {
 public:
  TestFollower(const std::string& name, const BT::NodeConfiguration& config, NavigationScenario& scenario)
      : BT::StatefulActionNode(name, config), scenario_(scenario) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"), BT::InputPort<std::string>("controller_id"),
            BT::InputPort<std::string>("goal_checker_id")};
  }
  BT::NodeStatus onStart() override {
    require(!scenario_.planning);
    nav_msgs::msg::Path path;
    require(static_cast<bool>(getInput("path", path)) && !path.poses.empty());
    scenario_.followed_starts.push_back(path.poses.front().pose.position.x);
    ++scenario_.follows; return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    require(!scenario_.follow_results.empty());
    auto status=scenario_.follow_results.front();
    scenario_.follow_results.erase(scenario_.follow_results.begin());
    if (status==BT::NodeStatus::FAILURE) scenario_.robot_x+=.5;
    return status;
  }
  void onHalted() override {}
 private:
  NavigationScenario& scenario_;
};

class TestWait : public BT::StatefulActionNode {
 public:
  TestWait(const std::string& name, const BT::NodeConfiguration& config, NavigationScenario& scenario)
      : BT::StatefulActionNode(name, config), scenario_(scenario) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<double>("wait_duration")}; }
  BT::NodeStatus onStart() override {
    double duration=0.; require(static_cast<bool>(getInput("wait_duration", duration)) && duration==1.);
    require(!scenario_.planning); ++scenario_.waits; return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
  void onHalted() override {}
 private:
  NavigationScenario& scenario_;
};

void navigationRecoveryRegression(const char* plugin, const std::filesystem::path& xml) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(plugin);
  factory.registerFromPlugin("libnav2_recovery_node_bt_node.so");
  factory.registerFromPlugin("libnav2_pipeline_sequence_bt_node.so");
  factory.registerFromPlugin("libnav2_rate_controller_bt_node.so");
  NavigationScenario* active=nullptr;
  factory.registerBuilder<TestPlanner>("ComputePathToPose", [&](const std::string& n, const BT::NodeConfiguration& c) {
    return std::make_unique<TestPlanner>(n,c,*active);
  });
  factory.registerBuilder<TestFollower>("FollowPath", [&](const std::string& n, const BT::NodeConfiguration& c) {
    return std::make_unique<TestFollower>(n,c,*active);
  });
  factory.registerBuilder<TestWait>("Wait", [&](const std::string& n, const BT::NodeConfiguration& c) {
    return std::make_unique<TestWait>(n,c,*active);
  });
  factory.registerSimpleCondition("IsPathValid", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
      {BT::InputPort<nav_msgs::msg::Path>("path"), BT::InputPort<int>("server_timeout")});
  geometry_msgs::msg::PoseStamped goal; goal.header.frame_id="map"; goal.pose.position.x=4.;
  nav_msgs::msg::Path saved; saved.header=goal.header; saved.poses.resize(2); saved.poses.back()=goal;
  const auto exercise=[&](NavigationScenario& scenario, bool retained) {
    active=&scenario;
    auto board=BT::Blackboard::create(); board->set("goal",goal);
    if (retained) { board->set("path",saved); board->set("committed_goal",goal); }
    auto tree=factory.createTreeFromFile(xml.string(),board);
    auto status=BT::NodeStatus::RUNNING;
    for (int ticks=0; ticks<200 && status==BT::NodeStatus::RUNNING; ++ticks) status=tree.tickRoot();
    require(status!=BT::NodeStatus::RUNNING);
    return std::make_pair(status,board->get<nav_msgs::msg::Path>("path"));
  };
  NavigationScenario healthy;
  healthy.follow_results.assign(28,BT::NodeStatus::RUNNING);
  healthy.follow_results.push_back(BT::NodeStatus::SUCCESS);
  require(exercise(healthy,false).first==BT::NodeStatus::SUCCESS);
  require(healthy.computations==1 && healthy.follows==1 && healthy.waits==0);
  NavigationScenario retained; retained.follow_results={BT::NodeStatus::SUCCESS};
  require(exercise(retained,true).first==BT::NodeStatus::SUCCESS);
  require(retained.computations==0 && retained.waits==0);
  NavigationScenario recovered; recovered.follow_results={BT::NodeStatus::FAILURE,BT::NodeStatus::SUCCESS};
  require(exercise(recovered,false).first==BT::NodeStatus::SUCCESS);
  require(recovered.computations==2 && recovered.follows==2 && recovered.waits==1);
  require(recovered.followed_starts==std::vector<double>({0.,.5}));
  NavigationScenario blocked; blocked.fail_plan=true; blocked.follow_results={BT::NodeStatus::FAILURE};
  auto failed=exercise(blocked,true);
  require(failed.first==BT::NodeStatus::FAILURE && failed.second==saved);
  require(blocked.computations==1 && blocked.follows==1 && blocked.waits==1);
  NavigationScenario invalid; invalid.invalid_plan=true; invalid.follow_results={BT::NodeStatus::FAILURE};
  auto rejected=exercise(invalid,true);
  require(rejected.first==BT::NodeStatus::FAILURE && rejected.second==saved);
  require(invalid.computations==1 && invalid.follows==1 && invalid.waits==1);
  NavigationScenario exhausted;
  exhausted.follow_results={BT::NodeStatus::FAILURE,BT::NodeStatus::FAILURE,BT::NodeStatus::FAILURE};
  require(exercise(exhausted,false).first==BT::NodeStatus::FAILURE);
  require(exhausted.computations==3 && exhausted.follows==3 && exhausted.waits==2);
}

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) return 2;
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
  auto xml=argc==3 ? std::filesystem::path(argv[2])
      : std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/"config/navigate_committed_path.xml";
  navigationRecoveryRegression(argv[1],xml);
  std::cout << "PASS: path transaction, stable tracking and bounded controller recovery replan\n";
}
