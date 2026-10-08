#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <ros/ros.h>

struct MapParam {
    // Field dimensions
    double field_width  = 2.500;
    double field_height = 4.500;

    // Zone heights (stacked bottom-to-top)
    double bottom_shelf_depth = 0.2500;
    double corridor_height    = 0.6385;
    double shelf_row_depth    = 0.4820;

    // Row-2 (行2 瓶颈) horizontal
    double row2_left_width = 1.125;
    double row2_gap        = 0.250;

    // Row-1 (行1 中央岛) horizontal
    double row1_left_aisle  = 0.580;
    double row1_shelf_width = 1.140;
    double row1_mini_shelf  = 0.200;

    // Row-3 (行3 双车道) horizontal
    double row3_left_shelf   = 1.000;
    double row3_center_aisle = 0.500;

    // Top/bottom shelf
    double tb_shelf_width = 1.125;   // each side; center gap = field_width - 2*tb_shelf_width

    // Vehicle / slot dimensions
    double vehicle_length = 0.211;   // slot depth  (Y direction)
    double vehicle_width  = 0.191;   // slot width  (X direction)

    // 参考点：路径/轨迹以「后轴中心」为参考点（而非车身几何中心）。规划器对后轴
    // 生成曲率连续轨迹；碰撞 OBB / footprint 由后轴向前偏移 d 还原车身中心。
    // 实车几何：rear_hang + wheel_base + front_hang = vehicle_length。
    double rear_hang  = 0.032;   // 后悬：后轴中心→车尾
    double front_hang = 0.036;   // 前悬：前轴中心→车头
    // d = 后轴中心在车身几何中心「后方」的纵向距离 = length/2 - rear_hang；
    // fromROSParam 按 rear_hang 重算，此处为占位默认。
    double rear_axle_to_center = 0.0735;

    // Vehicle steering kinematics —— 拐弯（曲率连续 clothoid）的单一参数源。
    // 规划器(path_generator)与地图(map_node)的拐弯几何都从这里派生，保证一致。
    double wheel_base      = 0.143;  // m   轴距
    double max_steer_angle = 0.50;   // rad 最大前轮转角
    double max_steer_rate  = 0.25;   // rad/s 最大转向角速度
    double turn_speed      = 0.20;   // m/s 用于计算曲率斜升段长度的代表车速
    double path_resolution = 0.01;   // m   曲线采样步长

    // Visualization
    double pre_dock_clearance = 0.150;
    double road_line_width    = 0.020;
    double arrow_size         = 0.060;

    std::string frame_id = "map";

    // ── 拐弯几何（派生自上面的转向运动学）──────────────────────────────────
    double turn_max_curvature() const {
        return std::tan(std::max(0.05, max_steer_angle)) / wheel_base;  // 1/R_min
    }
    double turn_ramp_len() const {  // 曲率线性斜升段长度
        return std::max(path_resolution,
                        turn_speed * max_steer_angle / std::max(0.05, max_steer_rate));
    }
    double turn_ds() const { return std::max(0.01, path_resolution); }

    // ── 车身相对后轴参考点的纵向伸出量（沿 +heading 前进方向）────────────────
    double body_front_ext() const { return vehicle_length - rear_hang; }  // 后轴→车头 = wheel_base+front_hang
    double body_rear_ext()  const { return rear_hang; }                   // 后轴→车尾

    // Derived: zone Y boundaries (bottom=0, top=field_height)
    double y1()  const { return bottom_shelf_depth; }                  // 0.250
    double y2()  const { return y1() + corridor_height; }              // 0.889
    double y3()  const { return y2() + shelf_row_depth; }              // 1.371
    double y4()  const { return y3() + corridor_height; }              // 2.009
    double y5()  const { return y4() + shelf_row_depth; }              // 2.491
    double y6()  const { return y5() + corridor_height; }              // 3.130
    double y7()  const { return y6() + shelf_row_depth; }              // 3.611
    double y8()  const { return y7() + corridor_height; }              // 4.250

    // Corridor center Y values
    double corridor4_cy() const { return (y1() + y2()) / 2; }          // 0.570
    double corridor3_cy() const { return (y3() + y4()) / 2; }          // 1.690
    double corridor2_cy() const { return (y5() + y6()) / 2; }          // 2.811
    double corridor1_cy() const { return (y7() + y8()) / 2; }          // 3.930

    static MapParam fromROSParam(ros::NodeHandle& nh) {
        MapParam p;
        std::string ns = "forklift_map/";
        nh.param(ns + "field_width",          p.field_width,          p.field_width);
        nh.param(ns + "field_height",         p.field_height,         p.field_height);
        nh.param(ns + "bottom_shelf_depth",   p.bottom_shelf_depth,   p.bottom_shelf_depth);
        nh.param(ns + "corridor_height",      p.corridor_height,      p.corridor_height);
        nh.param(ns + "shelf_row_depth",      p.shelf_row_depth,      p.shelf_row_depth);
        nh.param(ns + "row2_left_width",      p.row2_left_width,      p.row2_left_width);
        nh.param(ns + "row2_gap",             p.row2_gap,             p.row2_gap);
        nh.param(ns + "row1_left_aisle",      p.row1_left_aisle,      p.row1_left_aisle);
        nh.param(ns + "row1_shelf_width",     p.row1_shelf_width,     p.row1_shelf_width);
        nh.param(ns + "row1_mini_shelf",      p.row1_mini_shelf,      p.row1_mini_shelf);
        nh.param(ns + "row3_left_shelf",      p.row3_left_shelf,      p.row3_left_shelf);
        nh.param(ns + "row3_center_aisle",    p.row3_center_aisle,    p.row3_center_aisle);
        nh.param(ns + "tb_shelf_width",       p.tb_shelf_width,       p.tb_shelf_width);
        nh.param(ns + "vehicle_length",       p.vehicle_length,       p.vehicle_length);
        nh.param(ns + "vehicle_width",        p.vehicle_width,        p.vehicle_width);
        nh.param(ns + "rear_hang",            p.rear_hang,            p.rear_hang);
        nh.param(ns + "front_hang",           p.front_hang,           p.front_hang);
        // d 由实车几何派生(单一真值源)：后轴在车身中心后方 length/2 - rear_hang。
        p.rear_axle_to_center = p.vehicle_length * 0.5 - p.rear_hang;
        nh.param(ns + "wheel_base",           p.wheel_base,           p.wheel_base);
        nh.param(ns + "max_steer_angle",      p.max_steer_angle,      p.max_steer_angle);
        nh.param(ns + "max_steer_rate",       p.max_steer_rate,       p.max_steer_rate);
        nh.param(ns + "turn_speed",           p.turn_speed,           p.turn_speed);
        nh.param(ns + "path_resolution",      p.path_resolution,      p.path_resolution);
        nh.param(ns + "pre_dock_clearance",   p.pre_dock_clearance,   p.pre_dock_clearance);
        nh.param(ns + "road_line_width",      p.road_line_width,      p.road_line_width);
        nh.param(ns + "arrow_size",           p.arrow_size,           p.arrow_size);
        nh.param(ns + "frame_id",             p.frame_id,             p.frame_id);
        return p;
    }
};
