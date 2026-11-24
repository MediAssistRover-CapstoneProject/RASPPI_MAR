#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "footprint_sweep.hpp"

namespace mar_navigation {
class ForwardPivotPlanner : public nav2_core::GlobalPlanner {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
                 std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    node_=parent;name_=name;map_=std::move(map);auto n=parent.lock();
    auto param=[&](const std::string & key,double value) {
      nav2_util::declare_parameter_if_not_declared(n,name+"."+key,rclcpp::ParameterValue(value));
      return n->get_parameter(name+"."+key).as_double();
    };
    max_time_=param("max_planning_time",2.);rotation_cost_=param("rotation_cost",.10);
    cost_penalty_=param("cost_penalty",.8);pivot_penalty_=param("pivot_penalty",.08);
    goal_tolerance_=param("goal_tolerance",.025);
    direct_cost_threshold_=param("direct_cost_threshold",200.);
    nav2_util::declare_parameter_if_not_declared(n,name+".max_iterations",rclcpp::ParameterValue(200000));
    max_iterations_=n->get_parameter(name+".max_iterations").as_int();
    nav2_util::declare_parameter_if_not_declared(n,name+".respect_goal_heading",rclcpp::ParameterValue(false));
    respect_goal_heading_=n->get_parameter(name+".respect_goal_heading").as_bool();
    if (!std::isfinite(max_time_) || max_time_<=0. || max_time_>10. || max_iterations_<1 ||
        !std::isfinite(rotation_cost_) || rotation_cost_<=0. || !std::isfinite(cost_penalty_) || cost_penalty_<0. ||
        !std::isfinite(pivot_penalty_) || pivot_penalty_<0. || pivot_penalty_>1. ||
        !std::isfinite(goal_tolerance_) || goal_tolerance_<0. || goal_tolerance_>.05 ||
        !std::isfinite(direct_cost_threshold_) || direct_cost_threshold_<0. || direct_cost_threshold_>253.)
      throw nav2_core::PlannerException("Invalid forward/pivot planner limits");
  }
  void cleanup() override { map_.reset(); }
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                               const geometry_msgs::msg::PoseStamped & goal) override {
    const auto begun=std::chrono::steady_clock::now();auto node=node_.lock();
    if (!node || !map_ || start.header.frame_id!=map_->getGlobalFrameID() || goal.header.frame_id!=start.header.frame_id)
      throw nav2_core::PlannerException("FORWARD_PIVOT_FRAME_MISMATCH");
    std::unique_ptr<FootprintSweep> view;
    { auto * live=map_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
      // This already contains the configured 1 cm padding. Pivots use exactly
      // the same body as translation; check their actual sweep, not a larger box.
      view=std::make_unique<FootprintSweep>(*live,map_->getRobotFootprint()); }
    // A consistent RAM snapshot keeps search from blocking incoming lidar/costmap updates.
    const auto & grid=*view;const double r=grid.resolution;
    const SE2Pose origin{start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)};
    const SE2Pose target{goal.pose.position.x,goal.pose.position.y,tf2::getYaw(goal.pose.orientation)};
    if (!grid.poseClear(origin)) throw nav2_core::PlannerException("FORWARD_PIVOT_START_FOOTPRINT_BLOCKED");
    if (!grid.inBounds(target.x,target.y)) throw nav2_core::PlannerException("FORWARD_PIVOT_GOAL_OUTSIDE_MAP");
    if (!respect_goal_heading_ && std::hypot(target.x-origin.x,target.y-origin.y)<=goal_tolerance_)
      return makePath(start,{origin,origin},r,begun,0,0);
    std::vector<SE2Pose> direct;
    if (lowCostLine(grid,origin,target) && connect(grid,origin,target,direct,respect_goal_heading_)) {
      direct.insert(direct.begin(),origin);
      return makePath(start,direct,r,begun,0,0);
    }
    const auto width=grid.width,height=grid.height;
    const int sx=static_cast<int>((origin.x-grid.origin_x)/r),sy=static_cast<int>((origin.y-grid.origin_y)/r);
    const size_t count=static_cast<size_t>(width)*height*16;
    if (count>4000000) throw nav2_core::PlannerException("FORWARD_PIVOT_MAP_STATE_LIMIT");
    struct Record { float score=std::numeric_limits<float>::infinity();int parent=-1;int8_t clear=0,action=0;bool closed=false; };
    std::vector<Record> records(count);
    auto index=[&](int x,int y,int a) { return (y*static_cast<int>(width)+x)*16+a; };
    auto pose=[&](int id) { const int a=id%16,cell=id/16,x=cell%width,y=cell/width;
      return SE2Pose{origin.x+(x-sx)*r,origin.y+(y-sy)*r,heading(a)}; };
    auto valid=[&](int id) { auto & c=records[id].clear;if (!c)c=grid.poseClear(pose(id))?1:-1;return c>0; };
    struct QueueItem { double rank;float score;int id;bool operator<(const QueueItem & other) const {return rank>other.rank;} };
    std::priority_queue<QueueItem> open;
    auto heuristic=[&](const SE2Pose & p) {return std::hypot(p.x-target.x,p.y-target.y);};
    // Anchor translations at the exact start instead of silently strafing to a cell centre.
    int seeded=0;
    for (int a=0;a<16;++a) {
      const int id=index(sx,sy,a);const auto p=pose(id);const double turn=wrap_angle(p.yaw-origin.yaw);
      if (!valid(id) || !grid.sweepClear(origin,p,turn)) continue;
      records[id].score=rotation_cost_*std::abs(turn)+(std::abs(turn)>1e-5?pivot_penalty_:0.);
      records[id].action=turn>1e-5?1:turn< -1e-5?-1:0;++seeded;
      open.push({records[id].score+heuristic(p),records[id].score,id});
    }
    int found=-1,expanded=0,goal_probes=0;std::vector<SE2Pose> finish;
    while (!open.empty()) {
      auto item=open.top();open.pop();auto & current=records[item.id];
      if (current.closed || item.score!=current.score) continue;
      if ((expanded++%128)==0 && (!rclcpp::ok() || std::chrono::duration<double>(std::chrono::steady_clock::now()-begun).count()>max_time_))
        throw nav2_core::PlannerException("FORWARD_PIVOT_SEARCH_TIMEOUT");
      if (expanded>max_iterations_) throw nav2_core::PlannerException("FORWARD_PIVOT_ITERATION_LIMIT");
      current.closed=true;const auto p=pose(item.id);
      // A visible exact destination does not require a final one-cell shuffle.
      const double remaining=heuristic(p);
      if (!respect_goal_heading_ && remaining<=goal_tolerance_) { found=item.id;break; }
      const double target_bearing=std::atan2(target.y-p.y,target.x-p.x);
      if (remaining<=1.5*r || (remaining<=1. && goal_probes<64 &&
          std::abs(wrap_angle(target_bearing-p.yaw))<.15 && lowCostLine(grid,p,target))) {
        ++goal_probes;
        if (connect(grid,p,target,finish,respect_goal_heading_)) { found=item.id;break; }
      }
      const int a=item.id%16,cell=item.id/16,x=cell%width,y=cell/width;
      for (int action=-1;action<=1;++action) {
        const int na=action?((a+action+16)%16):a;
        const int nx=x+(action?0:directions_[a][0]),ny=y+(action?0:directions_[a][1]);
        if (nx<0 || ny<0 || nx>=static_cast<int>(width) || ny>=static_cast<int>(height)) continue;
        const int next=index(nx,ny,na);if (records[next].closed || !valid(next)) continue;
        const auto q=pose(next);const double turn=action?wrap_angle(q.yaw-p.yaw):0.;
        if (!grid.sweepClear(p,q,turn)) continue;
        const double distance=std::hypot(q.x-p.x,q.y-p.y);
        const double penalty=1.+cost_penalty_*grid.cost(q.x,q.y)/252.;
        const double new_pivot=action && current.action!=action?pivot_penalty_:0.;
        const float score=current.score+(distance+rotation_cost_*std::abs(turn))*penalty+new_pivot;
        if (score>=records[next].score) continue;
        records[next].score=score;records[next].parent=item.id;records[next].action=action;
        open.push({score+heuristic(q),score,next});
      }
    }
    if (found<0) {
      RCLCPP_WARN(node->get_logger(),"FORWARD_PIVOT_NO_ROUTE seeded=%d expanded=%d start=(%.3f,%.3f,%.3f) goal=(%.3f,%.3f)",
        seeded,expanded,origin.x,origin.y,origin.yaw,target.x,target.y);
      throw nav2_core::PlannerException("FORWARD_PIVOT_NO_COLLISION_FREE_ROUTE");
    }
    std::vector<SE2Pose> route;for (int id=found;id>=0;id=records[id].parent) route.push_back(pose(id));
    std::reverse(route.begin(),route.end());route.insert(route.begin(),origin);
    route.insert(route.end(),finish.begin(),finish.end());
    const auto old_pivots=pivots(route);
    const auto deadline=std::min(begun+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(max_time_)),std::chrono::steady_clock::now()+std::chrono::milliseconds(120));
    route=shortcuts(grid,route,std::min(deadline,std::chrono::steady_clock::now()+std::chrono::milliseconds(80)));
    route=roundCorners(grid,route,deadline);
    return makePath(start,route,r,begun,expanded,old_pivots);
  }
 private:
  static double heading(int bin) {return std::atan2(directions_[bin][1],directions_[bin][0]);}
  bool connect(const FootprintSweep & grid,const SE2Pose & from,const SE2Pose & goal,
               std::vector<SE2Pose> & output,bool match_heading) const {
    const double distance=std::hypot(goal.x-from.x,goal.y-from.y);
    const double bearing=distance>1e-6?std::atan2(goal.y-from.y,goal.x-from.x):from.yaw;
    const SE2Pose aligned{from.x,from.y,bearing},arrived{goal.x,goal.y,bearing};
    if (!grid.sweepClear(from,aligned,wrap_angle(bearing-from.yaw)) || !grid.sweepClear(aligned,arrived,0.)) return false;
    if (match_heading && !grid.sweepClear(arrived,goal,wrap_angle(goal.yaw-bearing))) return false;
    output={aligned,arrived};if (match_heading)output.push_back(goal);return true;
  }
  static int pivots(const std::vector<SE2Pose> & route) {
    int count=0;bool turning=false;
    for (size_t i=1;i<route.size();++i) {
      const bool turn=std::hypot(route[i].x-route[i-1].x,route[i].y-route[i-1].y)<1e-6 &&
        std::abs(wrap_angle(route[i].yaw-route[i-1].yaw))>1e-5;
      if (turn && !turning)++count;turning=turn;
    }
    return count;
  }
  bool lowCostLine(const FootprintSweep & grid,const SE2Pose & from,const SE2Pose & to) const {
    const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(to.x-from.x,to.y-from.y)/(.5*grid.resolution))));
    for (int i=0;i<=steps;++i) {
      const double t=static_cast<double>(i)/steps;
      if (grid.cost(from.x+t*(to.x-from.x),from.y+t*(to.y-from.y))>direct_cost_threshold_)return false;
    }
    return true;
  }
  double effort(const FootprintSweep & grid,const std::vector<SE2Pose> & route,size_t first,size_t last) const {
    double cost=0.;int previous=0;
    for (size_t i=first+1;i<=last;++i) {
      const auto & p=route[i-1];const auto & q=route[i];const double yaw=wrap_angle(q.yaw-p.yaw);
      const double length=std::hypot(q.x-p.x,q.y-p.y);
      const int turn=length<1e-6?(yaw>1e-5?1:yaw< -1e-5?-1:0):0;
      const int steps=std::max(1,static_cast<int>(std::ceil(length/(.5*grid.resolution))));
      double weighted=0.;
      for (int j=0;j<=steps;++j) {const double t=static_cast<double>(j)/steps;
        weighted+=(j==0 || j==steps?.5:1.)*(1.+cost_penalty_*grid.cost(p.x+t*(q.x-p.x),p.y+t*(q.y-p.y))/252.);}
      cost+=(length+rotation_cost_*std::abs(yaw))*weighted/steps;
      if (turn && turn!=previous)cost+=pivot_penalty_;previous=turn;
    }
    return cost;
  }
  static std::vector<SE2Pose> compact(const std::vector<SE2Pose> & input) {
    std::vector<SE2Pose> vertices;
    for (const auto & p : input) {
      if (!vertices.empty() && std::hypot(p.x-vertices.back().x,p.y-vertices.back().y)<1e-8 &&
          std::abs(wrap_angle(p.yaw-vertices.back().yaw))<1e-8)continue;
      if (vertices.size()>1) {
        const auto & a=vertices[vertices.size()-2];const auto & b=vertices.back();
        if (std::abs(wrap_angle(a.yaw-b.yaw))<1e-8 && std::abs(wrap_angle(b.yaw-p.yaw))<1e-8 &&
            std::abs((b.x-a.x)*(p.y-b.y)-(b.y-a.y)*(p.x-b.x))<1e-10 &&
            (b.x-a.x)*(p.x-b.x)+(b.y-a.y)*(p.y-b.y)>0.)vertices.pop_back();
      }
      vertices.push_back(p);
    }
    return vertices;
  }
  std::vector<SE2Pose> shortcuts(const FootprintSweep & grid,const std::vector<SE2Pose> & input,
                               const std::chrono::steady_clock::time_point & deadline) const {
    // Every shortcut retains both oriented body sweeps and the soft clearance
    // cost. Shortening a route must not erase its preference for corridor space.
    const auto vertices=compact(input);
    if (vertices.size()<3)return vertices;
    std::vector<SE2Pose> result{vertices.front()};size_t i=0;int checks=0;
    while (i+1<vertices.size()) {
      size_t selected=i+1;std::vector<SE2Pose> chosen;
      for (size_t j=vertices.size()-1;j>i+1;--j) {
        if (++checks>96 || std::chrono::steady_clock::now()>=deadline)break;
        std::vector<SE2Pose> candidate;
        if (!connect(grid,vertices[i],vertices[j],candidate,j+1<vertices.size() || respect_goal_heading_))continue;
        candidate.insert(candidate.begin(),vertices[i]);
        if (effort(grid,candidate,0,candidate.size()-1)+1e-6>=effort(grid,vertices,i,j))continue;
        selected=j;chosen=std::move(candidate);break;
      }
      if (chosen.empty())result.push_back(vertices[selected]);
      else result.insert(result.end(),chosen.begin()+1,chosen.end());
      i=selected;
    }
    return result;
  }
  std::vector<SE2Pose> roundCorners(const FootprintSweep & grid,const std::vector<SE2Pose> & input,
                                  const std::chrono::steady_clock::time_point & deadline) const {
    const auto route=compact(input);std::vector<SE2Pose> result;result.reserve(route.size());
    for (size_t i=0;i<route.size();) {
      size_t end=i;double planned_turn=0.;
      while (end+1<route.size() && std::hypot(route[end+1].x-route[i].x,route[end+1].y-route[i].y)<1e-6) {
        planned_turn+=wrap_angle(route[end+1].yaw-route[end].yaw);++end;
      }
      bool rounded=false;
      if (i>0 && end>i && end+1<route.size() && std::chrono::steady_clock::now()<deadline) {
        const auto & previous=route[i-1];const auto & corner=route[i];const auto & next=route[end+1];
        const double in_length=std::hypot(corner.x-previous.x,corner.y-previous.y);
        const double out_length=std::hypot(next.x-corner.x,next.y-corner.y);
        if (in_length>1e-6 && out_length>1e-6) {
          const double ux=(corner.x-previous.x)/in_length,uy=(corner.y-previous.y)/in_length;
          const double vx=(next.x-corner.x)/out_length,vy=(next.y-corner.y)/out_length;
          const double entry=std::atan2(uy,ux),exit=std::atan2(vy,vx),turn=wrap_angle(exit-entry);
          // Initial entry pivots, turnarounds and long-way rotations remain
          // explicit. A fillet replaces only a genuine internal forward corner.
          if (std::abs(turn)>.08 && std::abs(turn)<2.4 && std::abs(planned_turn-turn)<1e-5 &&
              std::abs(wrap_angle(corner.yaw-entry))<1e-5 && std::abs(wrap_angle(route[end].yaw-exit))<1e-5) {
            for (const double radius : {.45,.30,.18}) {
              if (std::chrono::steady_clock::now()>=deadline)break;
              const double trim=radius*std::tan(std::abs(turn)/2.);
              if (trim>.4*std::min(in_length,out_length))continue;
              const SE2Pose before{corner.x-ux*trim,corner.y-uy*trim,entry};
              const SE2Pose after{corner.x+vx*trim,corner.y+vy*trim,exit};
              const double sign=std::copysign(1.,turn),cx=before.x-sign*uy*radius,cy=before.y+sign*ux*radius;
              const double phase=std::atan2(before.y-cy,before.x-cx);
              const int steps=std::max(1,static_cast<int>(std::ceil(std::max(
                radius*std::abs(turn)/(.5*grid.resolution),std::abs(turn)/(2.*M_PI/180.)))));
              std::vector<SE2Pose> arc{before};bool clear=true;
              for (int j=1;j<=steps;++j) {
                if (j%8==0 && std::chrono::steady_clock::now()>=deadline) {clear=false;break;}
                const double t=static_cast<double>(j)/steps;
                const SE2Pose p{cx+radius*std::cos(phase+turn*t),cy+radius*std::sin(phase+turn*t),entry+turn*t};
                if (!grid.sweepClear(arc.back(),p,turn/steps)) {clear=false;break;}
                arc.push_back(p);
              }
              if (!clear || !grid.sweepClear(result.empty()?previous:result.back(),before,0.) ||
                  !grid.sweepClear(after,next,0.))continue;
              const std::vector<SE2Pose> old{{before.x,before.y,entry},{corner.x,corner.y,entry},
                {corner.x,corner.y,exit},{after.x,after.y,exit}};
              if (effort(grid,arc,0,arc.size()-1)>effort(grid,old,0,old.size()-1)+1e-6)continue;
              result.insert(result.end(),arc.begin(),arc.end());i=end+1;rounded=true;break;
            }
          }
        }
      }
      if (!rounded) {result.push_back(route[i]);++i;}
    }
    return result;
  }
  nav_msgs::msg::Path makePath(const geometry_msgs::msg::PoseStamped & start,const std::vector<SE2Pose> & route,
                             double resolution,const std::chrono::steady_clock::time_point & begun,
                             int expanded,int before) const {
    auto node=node_.lock();nav_msgs::msg::Path result;result.header=start.header;result.header.stamp=node->now();
    append(result,route.front());
    for (size_t i=1;i<route.size();++i) {
      const auto & from=route[i-1];const auto & to=route[i];const auto delta=wrap_angle(to.yaw-from.yaw);
      const int steps=std::max(1,static_cast<int>(std::ceil(std::max(std::hypot(to.x-from.x,to.y-from.y)/(.5*resolution),std::abs(delta)/(2.*M_PI/180.)))));
      for (int j=1;j<=steps;++j) {const double t=static_cast<double>(j)/steps;
        append(result,{from.x+t*(to.x-from.x),from.y+t*(to.y-from.y),from.yaw+t*delta});}
    }
    size_t curve_samples=0;
    for (size_t i=1;i<route.size();++i)if (std::hypot(route[i].x-route[i-1].x,route[i].y-route[i-1].y)>1e-6 &&
      std::abs(wrap_angle(route[i].yaw-route[i-1].yaw))>1e-5)++curve_samples;
    RCLCPP_INFO(node->get_logger(),"FORWARD_PIVOT_PLAN expanded=%d poses=%zu pivots_before=%d pivots_after=%d curve_samples=%zu compute_ms=%.1f footprint_padding=costmap",
      expanded,result.poses.size(),before,pivots(route),curve_samples,1000.*std::chrono::duration<double>(std::chrono::steady_clock::now()-begun).count());
    return result;
  }
  static void append(nav_msgs::msg::Path & path,const SE2Pose & p) {
    geometry_msgs::msg::PoseStamped v;v.header=path.header;v.pose.position.x=p.x;v.pose.position.y=p.y;
    v.pose.orientation.z=std::sin(p.yaw/2.);v.pose.orientation.w=std::cos(p.yaw/2.);path.poses.push_back(v);
  }
  inline static constexpr std::array<std::array<int,2>,16> directions_{{
    {{1,0}},{{2,1}},{{1,1}},{{1,2}},{{0,1}},{{-1,2}},{{-1,1}},{{-2,1}},
    {{-1,0}},{{-2,-1}},{{-1,-1}},{{-1,-2}},{{0,-1}},{{1,-2}},{{1,-1}},{{2,-1}} }};
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::string name_;double max_time_=2.,rotation_cost_=.10,cost_penalty_=.8,pivot_penalty_=.08,goal_tolerance_=.025,direct_cost_threshold_=200.;
  int max_iterations_=200000;bool respect_goal_heading_=false;
};
}  // namespace mar_navigation
PLUGINLIB_EXPORT_CLASS(mar_navigation::ForwardPivotPlanner,nav2_core::GlobalPlanner)
