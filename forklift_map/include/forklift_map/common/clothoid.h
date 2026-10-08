#pragma once
// 曲率连续拐弯曲线（回旋曲线 / clothoid）——单一算法真相。
// 零外部依赖，forklift_map 与 forklift_planner 共用，保证"地图上画的拐弯"
// 与"规划器实际行驶的拐弯"几何完全一致（同一份代码、同一套参数）。
#include <vector>

namespace forklift_geom {

struct CurvePt {
    double x = 0.0;
    double y = 0.0;
};

// 局部坐标系下的拐弯曲线：起点在原点、初始朝向 +x，转过 signed_angle。
// t_in / t_out 为入/出切线长（角点到曲线起/止点沿两条切线的距离）。
struct ClothoidTurn {
    std::vector<CurvePt> pts;
    std::vector<double> headings;
    double t_in = 0.0;
    double t_out = 0.0;
};

// 沿弧长 s 的曲率剖面：线性斜升 → 恒定 → 线性斜降（保证曲率连续）。
double curvature_at(double s, double total_len, double ramp_len,
                    double hold_len, double peak_k);

// 生成一个转角为 signed_angle 的曲率连续拐弯。
//   max_curvature : 峰值曲率 (1/R_min)
//   rate_ramp_len : 曲率斜升段长度（转向速率限制决定）
//   ds            : 采样步长
ClothoidTurn build_clothoid_turn(double signed_angle, double max_curvature,
                                 double rate_ramp_len, double ds);

// 自适应版：在切线长不超过 max_tangent 的约束下，二分搜索最大可行 ramp_len。
ClothoidTurn fit_clothoid_turn(double signed_angle, double max_curvature,
                               double desired_ramp_len, double max_tangent,
                               double ds);

}  // namespace forklift_geom
