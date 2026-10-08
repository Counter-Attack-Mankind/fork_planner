#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

namespace forklift_planner {
namespace multi_vehicle {

struct MultiVehicleConfig {
    int vehicle_count = 8;
    int random_seed = 42;
    // true: repeatable pseudo-random task sequence from random_seed;
    // false: seed the task allocator from std::random_device at every launch.
    bool reproducible_task_random = true;

    double nominal_speed = 0.20;
    double max_speed = 0.26;
    double max_accel = 0.20;
    double max_decel = 0.30;
    double safety_margin = 0.0;
    double conflict_margin = 0.12;

    // Stateless bridge-aware TTC correction. The first threshold preserves
    // the existing local opposing definition. Traversal changes update actual
    // motion headings but do not themselves terminate backtracking.
    double bridge_opposing_threshold = -0.50;
    double bridge_backtrack_step = 0.01;

    double dwell_time = 20.0;       // sleep time
    double pickup_dwell_time = 5.0; // A1 pickup operation
    double unload_dwell_time = 5.0; // B-slot unload operation

    // A1 is an independent physical pickup station. These values describe
    // the parked vehicle BODY CENTER and its pre-dock reference; path points
    // remain rear-axle referenced and are converted by the path generator.
    double a1_pickup_center_x = 1.250;
    double a1_pickup_center_y = 4.375;
    double a1_pickup_theta = 1.5707963267948966;
    double a1_pre_dock_x = 1.250;
    double a1_pre_dock_y = 4.100;
    // Upstream control margin from the bare-body A1 conflict boundary. Unit: m.
    double a1_stop_margin = 0.10;
    // Future A1 owner reservation window. Independent of rolling/prediction
    // horizons and used only when refreshing the FutureA1Commitment. Unit: s.
    double a1_owner_horizon = 45.0;
    double rolling_horizon = 10.0;          // future planner time 
    double rolling_refresh_period = 2.0;        //  refresh period

    double prediction_horizon = 10.0;
    double prediction_step = 0.05;
    // NOMINAL-baseline timed-conflict bands. A first conflict at or beyond
    // far stays NOMINAL; [near, far) requests YIELD; below near requests
    // CREEP. Braking infeasibility independently tightens the request to STOP.
    double dynamic_speed_far_threshold = 10.0;
    double dynamic_speed_near_threshold = 5.0;
    // Ordinary NEAR STOP gate time reserve beyond one rolling period and the
    // planned action's braking time. Unit: seconds.
    double dynamic_stop_time_margin = 0.10;

    double creep_ratio = 0.25;
    double yield_ratio = 0.50;
    double boost_ratio = 1.20;
    bool enable_boost = true;

    double emergency_time = 0.80;
    double final_decision_time = 2.00;
    double warning_time = 5.00;

    double target_request_distance = 0.45;
    double target_stop_distance = 0.12;
    double following_normal_distance = 0.35;
    double following_creep_distance = 0.22;
    double following_min_distance = 0.13;
    double starvation_wait_time = 8.0;

    // Minimum time (s) an action must be held before it may relax toward a less
    // restrictive action. Stops per-tick STOP<->NOMINAL / YIELD<->NOMINAL
    // chatter (spec section 15) and forces a staged STOP->CREEP->NOMINAL restart
    // (spec section 14). Tightening (braking harder) is always applied at once.
    // Set <= 0 to disable smoothing (raw rule output each cycle).
    double action_hold_time = 0.4;

    //==============(死锁的部分参数)=========================
    bool enable_priority_tiebreak = true;  // who proceeds in a symmetric conflict
    bool deadlock_enabled = true;
    double deadlock_retreat_distance = 0.50;  // 每次固定沿 s- 退让 0.5 m
    int deadlock_retreat_max_attempts = 3;    // 最多退让 3 次
    double deadlock_confirm_time = 4.0;          // s
    double deadlock_retreat_speed = 0.10;        // m/s; RETREAT only

    // 实车模式:位置来自动捕 /object(替代 advanceVehicles 积分),输出 /traj_i + /coord_speed_i
    // 给 pure_pursuit。协调(updateDwellAndTasks/decide/到库DWELL/预测错峰)与 sim 逐字节一致。
    bool real_mode = false;
    // Real mode only: physical vehicle IDs selected for this experiment.
    // vehicle_count is derived from this list; simulation keeps V0..V(N-1).
    std::vector<int> vehicle_ids;
    bool one_shot_traj = false;          // true=一次性规划，false=滚动时域
    // 实车安全/到点阈值(仅 real_mode 用,现场可调,不影响 sim):
    double real_pose_timeout = 0.5;   // s,某车动捕失联>此值→强制其 coord_speed=0(防盲走)
    double real_arrive_tol = 0.05;    // m,path_s 距 length<此值即判到点(>PP的2cm硬停容差,防卡死)
    double real_emergency_margin = 0;  // m,0=关闭实车硬护栏，相当于安全检查两层。1.规划层的enforceForwardClearance() + 2.实车执行层的膨胀检测
    double lat_accel_max = 0.10;          // m/s² 侧向加速度上限:越小弯道越慢、跟得越紧

    bool show_paths = true;
    bool show_prediction_conflicts = true;
    bool skip_arc_fallback_paths = true;
    bool reject_curvature_discontinuity = true;
    bool reject_boundary_violations = true;
    bool reject_shelf_collisions = true;
    bool reject_path_kinks = true;        // reject paths with a kinematic kink
    double kink_min_angle = 0.61;         // rad (~35deg): below = smooth travel
    double kink_cusp_angle = 2.53;        // rad (~145deg): above = clean reverse cusp
    double path_validation_step = 0.01;
    bool precompute_task_filter = true;
    bool log_invalid_task_pairs = false;
    bool quiet_task_filter_precompute = true;

    // 简单测试版(eight-veh-sim):每车选「路径最短且全程前进(无 REVERSE 段=无尖点)」的目标,
    // 一把开进去、走得近。覆盖默认的跨排/分散选靶逻辑。仅用于实车链路冒烟测试。
    bool simple_forward_demo = false;
    bool use_a1_cycle = false;  // true: execute B->A1 and A1->B as separate legs
    std::string a1_cycle_catalog_file;
    bool save_a1_cycle_catalog = true;

    int recent_target_memory = 5;
    int recent_row_memory = 4;

    std::vector<int> start_slots;
    // 简单测试版:每车预设终点库位(与 start_slots 同序,车 i → target_slots[i])。空=自动选最近前进目标。
    std::vector<int> target_slots;
    bool randomize_start = false;  // true: draw distinct start slots from random_seed

    static MultiVehicleConfig fromROSParam(ros::NodeHandle& nh);
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
