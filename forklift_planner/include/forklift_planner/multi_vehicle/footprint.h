#pragma once

#include <array>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/path_generator.h"

namespace forklift_planner {
namespace multi_vehicle {

struct OBB {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double half_l = 0.0;
    double half_w = 0.0;
};

struct FootprintPoint {
    double x = 0.0;
    double y = 0.0;
};

// 路径位姿 `ref` 以「后轴中心」为参考点。本助手沿车体纵轴(+ref.theta)前移
// rear_axle_to_center，返回车身几何中心位姿——所有碰撞/footprint 计算的实际锚点。
RoughWp bodyCenterPose(const RoughWp& ref, const MapParam& mp);

OBB makeBody(const RoughWp& pose, const MapParam& mp, double margin);
bool overlaps(const OBB& a, const OBB& b);
std::array<FootprintPoint, 4> footprintCorners(const RoughWp& pose,
                                               const MapParam& mp,
                                               double margin = 0.0);
bool footprintInsideField(const RoughWp& pose, const MapParam& mp,
                          double margin = 0.0);
bool footprintIntersectsShelf(const RoughWp& pose, const ShelfBlock& shelf,
                              const MapParam& mp, double margin = 0.0);

}  // namespace multi_vehicle
}  // namespace forklift_planner
