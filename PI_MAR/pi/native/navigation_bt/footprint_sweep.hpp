#pragma once
// Immutable costmap observation; exact polygon/cell tests include the body interior.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace mar_navigation {
struct SE2Pose { double x, y, yaw; };
inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

class FootprintSweep {
 public:
  // Caller holds the live map mutex only while constructing this snapshot.
  FootprintSweep(const nav2_costmap_2d::Costmap2D & map,
                 const std::vector<geometry_msgs::msg::Point> & footprint)
      : width(map.getSizeInCellsX()), height(map.getSizeInCellsY()),
        resolution(map.getResolution()), origin_x(map.getOriginX()), origin_y(map.getOriginY()),
        footprint_(footprint), costs_(map.getCharMap(), map.getCharMap()+width*height),
        blocked_((width+1)*(height+1), 0) {
    for (const auto & p : footprint_) radius_=std::max(radius_,std::hypot(p.x,p.y));
    for (unsigned y=0;y<height;++y) {
      unsigned row=0;
      for (unsigned x=0;x<width;++x) {
        row+=costs_[y*width+x]>=254;
        blocked_[(y+1)*(width+1)+x+1]=blocked_[y*(width+1)+x+1]+row;
      }
    }
  }
  bool inBounds(double x,double y) const {
    return x>=origin_x && y>=origin_y && x<origin_x+width*resolution && y<origin_y+height*resolution;
  }
  unsigned char cost(double x,double y) const {
    if (!inBounds(x,y)) return 255;
    return costs_[static_cast<unsigned>((y-origin_y)/resolution)*width+
                  static_cast<unsigned>((x-origin_x)/resolution)];
  }
  bool poseClear(const SE2Pose & pose) const {
    if (footprint_.size()<3 || !std::isfinite(pose.x+pose.y+pose.yaw)) return false;
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    std::vector<Point> polygon;polygon.reserve(footprint_.size());
    double lo_x=1e30,lo_y=1e30,hi_x=-1e30,hi_y=-1e30;
    for (const auto & p : footprint_) {
      Point q{(pose.x+c*p.x-s*p.y-origin_x)/resolution,
              (pose.y+s*p.x+c*p.y-origin_y)/resolution};
      polygon.push_back(q);lo_x=std::min(lo_x,q.x);hi_x=std::max(hi_x,q.x);
      lo_y=std::min(lo_y,q.y);hi_y=std::max(hi_y,q.y);
    }
    const int x0=static_cast<int>(std::floor(lo_x)),x1=static_cast<int>(std::floor(hi_x));
    const int y0=static_cast<int>(std::floor(lo_y)),y1=static_cast<int>(std::floor(hi_y));
    if (x0<0 || y0<0 || x1>=static_cast<int>(width) || y1>=static_cast<int>(height)) return false;
    const auto stride=width+1;
    if (blocked_[(y1+1)*stride+x1+1]-blocked_[y0*stride+x1+1]
        -blocked_[(y1+1)*stride+x0]+blocked_[y0*stride+x0]==0) return true;
    for (int y=y0;y<=y1;++y) for (int x=x0;x<=x1;++x) {
      if (costs_[y*width+x]<254) continue;
      if (overlaps(polygon,x,y)) return false;
    }
    return true;
  }
  bool sweepClear(const SE2Pose & from,const SE2Pose & to,double turn) const {
    const auto steps=std::max(1,static_cast<int>(std::ceil(std::max(
      std::hypot(to.x-from.x,to.y-from.y)/(.5*resolution),std::abs(turn)/(2.*M_PI/180.)))));
    for (int i=0;i<=steps;++i) {
      const double t=static_cast<double>(i)/steps;
      if (!poseClear({from.x+t*(to.x-from.x),from.y+t*(to.y-from.y),from.yaw+t*turn})) return false;
    }
    return true;
  }
  unsigned width,height;
  double resolution,origin_x,origin_y;
 private:
  struct Point { double x,y; };
  static bool overlaps(const std::vector<Point> & polygon,int x,int y) {
    // The x/y bounds already overlap; the remaining separating axes are polygon edges.
    for (size_t i=0;i<polygon.size();++i) {
      const auto & p=polygon[i];const auto & q=polygon[(i+1)%polygon.size()];
      const double ax=-(q.y-p.y),ay=q.x-p.x;
      double pmin=1e30,pmax=-1e30;
      for (const auto & v : polygon) { const auto d=v.x*ax+v.y*ay;pmin=std::min(pmin,d);pmax=std::max(pmax,d); }
      const double middle=(x+.5)*ax+(y+.5)*ay,half=.5*(std::abs(ax)+std::abs(ay));
      if (pmax<middle-half-1e-10 || middle+half<pmin-1e-10) return false;
    }
    return true;
  }
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::vector<unsigned char> costs_;
  std::vector<unsigned> blocked_;
  double radius_=0.;
};
}  // namespace mar_navigation
