#include "forklift_planner/path_generator.h"

#include <algorithm>
#include <cmath>

#include "forklift_planner/path_generator_internal.h"

//注意！！该文件不是完整路径生成算法，而是 PathGenerator 的入口辅助层。
/*主要做如下工作：
1. 初始化几个关键通道 x 坐标
2. 角度归一化
3. 根据货架行号判断走廊编号
4. 把任意真实位姿包装成一个临时 Slot，再复用 generate()
*/

//构造函数传入地图参数与规划参数
PathGenerator::PathGenerator(const MapParam& mp, const PlannerParam& pp)
    : PathGenerator(mp, pp, PathGeneratorRouteMode::AUTO) {}

PathGenerator::PathGenerator(const MapParam& mp, const PlannerParam& pp,
                             PathGeneratorRouteMode route_mode)
    : mp_(mp), pp_(pp), route_mode_(route_mode) {
    spine_x_        = mp_.field_width * 0.5;        //计算场地中央通道的x坐标，地图中线
    left_bypass_x_  = mp_.row1_left_aisle * 0.5;    //左侧绕行通道
    const double row1_right = mp_.row1_left_aisle + mp_.row1_shelf_width;
    const double mini_left  = mp_.field_width - mp_.row1_mini_shelf;
    right_bypass_x_ = (row1_right + mini_left) * 0.5;   //右侧绕行通道
}

//进行角度归一化，把角度限制到 (-pi, pi]
double PathGenerator::norm_angle(double a) {
    while (a >  forklift_planner::path_internal::kPi) {
        a -= 2.0 * forklift_planner::path_internal::kPi;
    }
    while (a <= -forklift_planner::path_internal::kPi) {
        a += 2.0 * forklift_planner::path_internal::kPi;
    }
    return a;
}

//根据货位行号来判断走廊，共有0-7排货架，分割为4个走廊（用于起点、终点规则数据类型slot，来判断走廊）
int PathGenerator::corridor_id(int row_id) const {
    if (row_id <= 1) return 1;
    if (row_id <= 3) return 2;
    if (row_id <= 5) return 3;
    return 4;
}

//根据当前实际y坐标来判断走廊，（用于车当前实时重规划，由于车不一定在标准货位中，只能在路上某个x,y位置上）
int PathGenerator::corridor_of_y(double y) const {
    using namespace forklift_planner::path_internal;
    int best = 3;
    double best_d = 1e18;
    for (int c = 1; c <= 4; ++c) {
        const double lo = corridor_min_y(mp_, c), hi = corridor_max_y(mp_, c);
        if (y >= lo && y <= hi) return c;  // 落在走廊内
        const double d = std::min(std::abs(y - lo), std::abs(y - hi));
        if (d < best_d) { best_d = d; best = c; }
    }
    return best;  // 不在任何走廊内 → 取最近的
}

//重要！！！（从真实位姿来生成路径）----->用于实车当前位置重规划，死锁恢复
RoughPath PathGenerator::generateFromPose(double x, double y, double theta,
                                          const Slot& tgt,
                                          PathGenerationInfo* info) const {
    // 把当前位姿包成「合成源 slot」:id<0 → generate 跳过出库前缀,路径从 (x,y) 直接接骨架;
    // row_id 取当前 y 所在走廊的代表行(决定 src 走廊);dock_theta 记当前朝向(供朝向衔接)。
    const int corr = corridor_of_y(y);
    const int rep_row = (corr == 1) ? 1 : (corr == 2) ? 3 : (corr == 3) ? 5 : 6;
    Slot src;
    src.id = -2;            // <0:非真实库位 → 跳过出库前缀
    src.row_id = rep_row;
    src.col = -1;
    src.cx = x;
    src.cy = y;
    src.dock_theta = theta;
    src.pre_dock_x = x;     // 前缀被跳过,pre_dock 仅占位
    src.pre_dock_y = y;
    src.occupied = false;
    return generate(src, tgt, info);
    //用代码将真实位姿临时包装为一个slot（src），然后复用普通的路径生成函数generate(src, tgt)
    /*唯一与真实起点不同的是，id=-2<0，
    后续生成时候不需要生成“从货位倒出来/开出来”的出库段直接从当前 x,y,theta 接入通道路”*/
}

//---------------------------（核心路径生成器！！！！！）-------------------
//重载，若不关心是否圆弧拼凑，则直接调用generate(src, tgt)
RoughPath PathGenerator::generate(const Slot& src, const Slot& tgt) const {
    return generate(src, tgt, nullptr);
}
