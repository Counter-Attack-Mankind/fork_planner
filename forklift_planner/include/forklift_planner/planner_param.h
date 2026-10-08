#pragma once
#include <string>
#include <ros/ros.h>

struct PlannerParam {
    double vehicle_speed  = 0.20;   // m/s  (operational cruise speed)
    double stop_duration  = 2.0;    // s at dock
    int    random_seed    = 42;
    double update_rate    = 10.0;   // Hz（草履虫规格§1：固定 dt 10Hz 控制频率）
    // 转向运动学 / 拐弯几何已下沉到 MapParam（单一参数源）：
    //   wheel_base / max_steer_angle / max_steer_rate / path_resolution
    //   见 MapParam::turn_max_curvature()/turn_ramp_len()/turn_ds()
    double path_color_r   = 0.2f;
    double path_color_g   = 0.8f;
    double path_color_b   = 1.0f;
    std::string frame_id  = "map";
    std::string turn_model = "clothoid";  // "clothoid" or "arc"
    std::string terminal_docking_mode = "auto";  // "auto", "forward", or "reverse"

    static PlannerParam fromROSParam(ros::NodeHandle& nh) {
        PlannerParam p;
        const std::string ns = "forklift_planner/";
        nh.param(ns + "vehicle_speed",  p.vehicle_speed,  p.vehicle_speed);
        nh.param(ns + "stop_duration",  p.stop_duration,  p.stop_duration);
        nh.param(ns + "random_seed",    p.random_seed,    p.random_seed);
        nh.param(ns + "update_rate",    p.update_rate,    p.update_rate);
        nh.param(ns + "path_color_r",   p.path_color_r,   p.path_color_r);
        nh.param(ns + "path_color_g",   p.path_color_g,   p.path_color_g);
        nh.param(ns + "path_color_b",   p.path_color_b,   p.path_color_b);
        nh.param(ns + "frame_id",       p.frame_id,       p.frame_id);
        nh.param(ns + "turn_model",     p.turn_model,     p.turn_model);
        nh.param(ns + "terminal_docking_mode",
                 p.terminal_docking_mode, p.terminal_docking_mode);
        return p;
    }
};
