#pragma once
#include <cmath>
#include <string>
#include <vector>
#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/planner_param.h"

// ── Path data types ────────────────────────────────────────────────────────────

enum class WpType { FORWARD, REVERSE };

// Dense sampled path waypoint.路径点结构体
struct RoughWp {
    double  x, y, theta;
    WpType  type;
};

// Full path = ordered waypoints; only the terminal parking segment may be reverse.
// 因此一条完整的参考路径，由容器中一系列路径点结构体组成(注意！这里只是纯几何路径，不带有时间戳的轨迹！)
using RoughPath = std::vector<RoughWp>;

struct DebugPathPoint {
    double x = 0.0;
    double y = 0.0;
};

enum class DebugPathLayerType {
    SKELETON,
    CLOTHOID,
    LANE_SHIFT,
    ARC_FALLBACK,
};

struct DebugPathLayer {
    DebugPathLayerType type = DebugPathLayerType::SKELETON;
    std::string label;
    std::vector<DebugPathPoint> points;
};

//用于记录路径生成附加的额外信息（使用曲率连续转弯，或使用的是arc圆弧路径？）
struct PathGenerationInfo {
    bool used_arc_fallback = false;
    std::vector<DebugPathLayer> debug_layers;
};

enum class PathGeneratorRouteMode {
    AUTO,
    A1_TO_B,
    B_TO_A1,
    A2_TO_B,
    B_TO_A2,
};

// ── PathGenerator ─────────────────────────────────────────────────────────────

class PathGenerator {
public:
    //构造函数需要（地图信息+规划参数）
    PathGenerator(const MapParam& mp, const PlannerParam& pp);
    PathGenerator(const MapParam& mp, const PlannerParam& pp,
                  PathGeneratorRouteMode route_mode);

    // Build complete path: dock(src) → pre_dock(src) → corridors → pre_dock(tgt) → dock(tgt)
    //（最重要！！！）这里输入为，起点、终点的slot（slot不仅为货位编号，同样带有坐标、预停靠信息等）
    RoughPath generate(const Slot& src, const Slot& tgt) const;     //普通调用
    RoughPath generate(const Slot& src, const Slot& tgt,PathGenerationInfo* info) const;  //带调试信息调用

    // 从任意当前位姿 (x,y,θ) 规划到目标库位(死锁恢复/实车以真实位姿规划用)。
    // 用「合成源 slot」复用 generate 的骨架路由+终端泊车;合成源 id<0 → 跳过出库前缀,
    // 路径从当前位姿直接接入骨架。返回空=不可行。
    RoughPath generateFromPose(double x, double y, double theta,
                               const Slot& tgt, PathGenerationInfo* info) const;

private:
    RoughPath generateRouteCommon(const Slot& src, const Slot& tgt,
                                  PathGenerationInfo* info) const;
    RoughPath generateRouteA1ToB(const Slot& src, const Slot& tgt,
                                 PathGenerationInfo* info) const;
    RoughPath generateRouteBToA1(const Slot& src, const Slot& tgt,
                                 PathGenerationInfo* info) const;
    RoughPath generateRouteA2ToB(const Slot& src, const Slot& tgt,
                                 PathGenerationInfo* info) const;
    RoughPath generateRouteBToA2(const Slot& src, const Slot& tgt,
                                 PathGenerationInfo* info) const;

    //  根据货架行号判断属于第几条走廊
    int corridor_id(int row_id) const;
    
    // 当前 y 落在哪个走廊(1..4);供合成源 slot 反推 row_id，判断当前在哪条走廊上？
    int corridor_of_y(double y) const;

    // Normalize angle to (-π, π]角度归一化
    static double norm_angle(double a);

    MapParam   mp_;
    PlannerParam pp_;
    PathGeneratorRouteMode route_mode_ = PathGeneratorRouteMode::AUTO;

    double spine_x_;
    double left_bypass_x_;
    double right_bypass_x_;
};
