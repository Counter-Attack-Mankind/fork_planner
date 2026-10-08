#pragma once

#include <std_msgs/ColorRGBA.h>

#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/path_track.h"
#include "forklift_planner/multi_vehicle/traffic_resource.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class VehicleAction {
    STOP = 0,
    CREEP = 1,
    YIELD = 2,
    NOMINAL = 3,
    BOOST = 4,
};

enum class VehicleMode {
    NEED_TASK,
    ACTIVE,
    DWELL,
};

// VehicleMode describes motion; MissionPhase describes the logistics workflow.
// In A1-cycle mode a transport is deliberately split into two independent
// tracks: B->A1 (empty) and, only after pickup, A1->B (loaded).
enum class MissionPhase {
    DIRECT_TO_B,          // legacy B->B mode
    TO_A1,
    PICKUP_DWELL,
    WAIT_DROPOFF_TASK,
    TO_B,
    UNLOAD_DWELL,
};

enum class LegTargetKind {
    B_SLOT,
    A1,
};

inline const char* actionName(VehicleAction action) {
    switch (action) {
        case VehicleAction::STOP: return "STOP";
        case VehicleAction::CREEP: return "CREEP";
        case VehicleAction::YIELD: return "YIELD";
        case VehicleAction::NOMINAL: return "NOMINAL";
        case VehicleAction::BOOST: return "BOOST";
    }
    return "UNKNOWN";
}

struct VehicleAgent {
    int id = 0;
    int current_slot = 0;
    int target_slot = 0;
    int task_count = 0;
    int blocker_id = -1;
    bool loaded = false;

    VehicleMode mode = VehicleMode::NEED_TASK;
    MissionPhase mission_phase = MissionPhase::DIRECT_TO_B;
    LegTargetKind leg_target = LegTargetKind::B_SLOT;
    VehicleAction action = VehicleAction::STOP;
    VehicleAction requested_action = VehicleAction::STOP;
    double action_hold_remaining = 0.0;  // s left before action may relax
    // TTC safety STOP commitment. Unlike action_hold_remaining (smoothing),
    // this blocks replanning for one complete rolling decision period.
    double ttc_stop_hold_remaining = 0.0;
    double wait_time = 0.0;
    double dwell_remaining = 0.0;
    double path_s = 0.0;
    double current_speed = 0.0;
    // Rear-axle path coordinate at which the complete body has cleared the
    // source B-slot sweep. Zero means this leg has no slot-departure phase.
    double slot_departure_clear_s = 0.0;

    // Simulation A1 preview: the B destination and A1->B path are prepared
    // as soon as pickup dwell starts, but they are not executed until the
    // physical 5 s pickup has completed.
    int pending_dropoff_slot = -1;
    PathTrack pending_dropoff_track;
    bool pending_dropoff_valid = false;

    // A narrowly-scoped priority for the first A1 departure maneuver. It is
    // cleared after the complete body has passed the first REVERSE->FORWARD
    // cusp; it must never turn the whole A1->B route into a privileged path.
    bool a1_departure_committed = false;
    double a1_departure_priority_until_s = 0.0;

    // 实车显示用真实位姿:real_mode 下由 /object 填(后轴中心)。RViz 据此显示车的【实际位置】
    // 而非投影到路径上的位置——这样跟踪误差(车偏离路径多少)一眼可见。sim 下不置位,行为不变。
    bool real_pose_valid = false;
    double real_x = 0.0, real_y = 0.0, real_yaw = 0.0;

    // 轨迹版本号:每次 track 被重设(分配新任务)即自增(见 task_allocator tryPlan)。
    // 唯一标识"当前这条固定路径实例",供 RuleEngine 的 C_ij 冲突块缓存判失效——比
    // task_count 可靠(task_count 在到达时即自增,而新轨迹在 dwell 后才设,二者不同步)。
    int path_gen = 0;

    PathTrack track;
    std_msgs::ColorRGBA color;
    std::vector<int> recent_targets;
    std::vector<int> recent_rows;
    std::string reason;

    // Phase 2 资源模型:当前固定路径经过的资源占用弧长区间(由 TrafficResourceMap
    // 算出,track 变化时刷新)。spans_track_len 用于检测 track 是否更换需重算。
    std::vector<ResourceSpan> resource_spans;
    double spans_track_len = -1.0;

    bool active() const { return mode == VehicleMode::ACTIVE && !track.empty(); }
    double remainingS() const { return track.empty() ? 0.0 : track.length() - path_s; }
};

inline bool moreRestrictive(VehicleAction a, VehicleAction b) {
    return static_cast<int>(a) < static_cast<int>(b);
}

inline VehicleAction minAction(VehicleAction a, VehicleAction b) {
    return moreRestrictive(a, b) ? a : b;
}

// One step toward less restrictive, for staged restart (spec section 14):
// STOP -> CREEP -> NOMINAL. YIELD also steps straight to NOMINAL.
inline VehicleAction relaxOneStep(VehicleAction from) {
    switch (from) {
        case VehicleAction::STOP:  return VehicleAction::CREEP;
        case VehicleAction::CREEP: return VehicleAction::NOMINAL;
        case VehicleAction::YIELD: return VehicleAction::NOMINAL;
        default:                   return VehicleAction::NOMINAL;
    }
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
