#include "forklift_planner/diagnostics/diagnostic_snapshot.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>

namespace forklift_planner {
namespace diagnostics {

const char* modeName(multi_vehicle::VehicleMode mode) {
    switch (mode) {
        case multi_vehicle::VehicleMode::NEED_TASK: return "NEED_TASK";
        case multi_vehicle::VehicleMode::ACTIVE: return "ACTIVE";
        case multi_vehicle::VehicleMode::DWELL: return "DWELL";
    }
    return "UNKNOWN";
}

const char* missionPhaseName(multi_vehicle::MissionPhase phase) {
    switch (phase) {
        case multi_vehicle::MissionPhase::DIRECT_TO_B: return "DIRECT_TO_B";
        case multi_vehicle::MissionPhase::TO_A1: return "TO_A1";
        case multi_vehicle::MissionPhase::PICKUP_DWELL: return "PICKUP_DWELL";
        case multi_vehicle::MissionPhase::WAIT_DROPOFF_TASK: return "WAIT_DROPOFF_TASK";
        case multi_vehicle::MissionPhase::TO_B: return "TO_B";
        case multi_vehicle::MissionPhase::UNLOAD_DWELL: return "UNLOAD_DWELL";
    }
    return "UNKNOWN";
}

const char* legTargetName(multi_vehicle::LegTargetKind target) {
    switch (target) {
        case multi_vehicle::LegTargetKind::B_SLOT: return "B_SLOT";
        case multi_vehicle::LegTargetKind::A1: return "A1";
    }
    return "UNKNOWN";
}

std::string readableSimTime(double seconds) {
    const double nonnegative = std::max(0.0, seconds);
    const long long tenths = static_cast<long long>(std::llround(nonnegative * 10.0));
    const long long minutes = tenths / 600;
    const double remainder = static_cast<double>(tenths % 600) / 10.0;
    char text[80];
    std::snprintf(text, sizeof(text), "%lldmin%.1fs", minutes, remainder);
    return text;
}

std::string formatVehicleCompact(const multi_vehicle::VehicleAgent& vehicle) {
    char text[420];
    const double length = vehicle.track.empty() ? 0.0 : vehicle.track.length();
    const double remaining = vehicle.track.empty() ? 0.0 : vehicle.remainingS();
    double x = std::numeric_limits<double>::quiet_NaN();
    double y = x;
    double yaw = x;
    if (vehicle.real_pose_valid) {
        x = vehicle.real_x;
        y = vehicle.real_y;
        yaw = vehicle.real_yaw;
    } else if (!vehicle.track.empty()) {
        const RoughWp pose = vehicle.track.poseAtS(
            std::min(vehicle.path_s, vehicle.track.length()));
        x = pose.x;
        y = pose.y;
        yaw = pose.theta;
    }
    std::snprintf(text, sizeof(text),
                  "  V%d mode=%s phase=%s action=%s requested=%s reason=%s "
                  "blocker=%d task=%d slot=%d->%d s=%.3f/%.3f rem=%.3f "
                  "pose=(%.3f,%.3f,%.3f) speed=%.3f wait=%.1f dwell=%.1f gen=%d",
                  vehicle.id, modeName(vehicle.mode), missionPhaseName(vehicle.mission_phase),
                  multi_vehicle::actionName(vehicle.action),
                  multi_vehicle::actionName(vehicle.requested_action),
                  vehicle.reason.empty() ? "-" : vehicle.reason.c_str(), vehicle.blocker_id,
                  vehicle.task_count, vehicle.current_slot, vehicle.target_slot, vehicle.path_s,
                  length, remaining, x, y, yaw, vehicle.current_speed, vehicle.wait_time,
                  vehicle.dwell_remaining, vehicle.path_gen);
    return text;
}

std::string formatFleetSnapshot(const std::vector<multi_vehicle::VehicleAgent>& vehicles,
                                unsigned long long tick) {
    std::string snapshot = "tick=" + std::to_string(tick);
    for (const auto& vehicle : vehicles) snapshot += "\n" + formatVehicleCompact(vehicle);
    return snapshot;
}

std::string formatStressSnapshot(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const multi_vehicle::RuleEngine& rule_engine, unsigned long long tick,
    double sim_time, bool include_geometry) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "tick=" << tick << " sim_t=" << sim_time;
    for (const auto& vehicle : vehicles) out << "\n" << formatVehicleCompact(vehicle);

    const auto state = rule_engine.snapshot();
    for (const auto& item : state.reservations) {
        const auto& key = item.first;
        const auto& reservation = item.second;
        out << "\n  reservation=V" << key.first << "/V" << key.second
            << " owner=V" << reservation.owner_id
            << " gen=" << reservation.gen_lo << "/" << reservation.gen_hi
            << " lo=[" << reservation.enter_lo << "," << reservation.exit_lo << "]"
            << " hi=[" << reservation.enter_hi << "," << reservation.exit_hi << "]"
            << " raw=" << reservation.raw_zone_index;
    }
    for (const auto& item : state.a1.departure_clusters) {
        const auto& cluster = item.second;
        out << "\n  departure_cluster=V" << item.first.first << "/V" << item.first.second
            << " owner=V" << cluster.owner_id << " owner_gen=" << cluster.owner_path_gen
            << " other=V" << cluster.other_id << " other_gen=" << cluster.other_path_gen
            << " active=" << (cluster.active ? 1 : 0)
            << " intervals=" << cluster.intervals.size()
            << " stop_boundary=" << cluster.waiter_stop_boundary_s
            << " stop_s=" << cluster.waiter_stop_s
            << " release=" << cluster.owner_release_exit_s << "/"
            << cluster.other_release_exit_s;
    }
    const auto& future = rule_engine.futureA1Commitment();
    out << "\n  future_a1=";
    if (future.valid()) {
        out << "owner=V" << future.owner_id << " gen=" << future.owner_path_gen
            << " arrival=" << future.predicted_a1_arrival_time
            << " to_b=" << future.predicted_to_b_time;
    } else {
        out << "none";
    }
    const auto& reserved = state.a1.reserved_a1_commitment;
    out << "\n  reserved_a1=";
    if (reserved.valid()) {
        out << "owner=V" << reserved.owner_id
            << " gen=" << reserved.owner_path_gen
            << " arrival=" << reserved.predicted_a1_arrival_time
            << " to_b=" << reserved.predicted_to_b_time
            << " selection_reason=" << state.a1.reservation_selection_reason
            << " cohort=[";
        bool first = true;
        for (const auto& entry : state.a1.reservation_cohort_path_gen) {
            if (!first) out << ",";
            first = false;
            out << "V" << entry.first << ":gen" << entry.second;
        }
        out << "]";
    } else {
        out << "none";
    }
    const auto& candidate = state.deadlock.candidate;
    const auto& transaction = state.deadlock.transaction;
    out << "\n  deadlock_candidate=";
    if (candidate.valid) {
        out << "V" << candidate.vehicle_a << "/V" << candidate.vehicle_b
            << " gen=" << candidate.path_gen_a << "/" << candidate.path_gen_b
            << " duration=" << candidate.duration
            << " anchor_s=" << candidate.anchor_s_a << "/"
            << candidate.anchor_s_b;
    } else {
        out << "none";
    }
    out << "\n  deadlock_recovery phase="
        << multi_vehicle::recoveryPhaseName(transaction.phase)
        << " retreat=V" << transaction.retreat_vehicle_id
        << " pass=V" << transaction.pass_vehicle_id
        << " attempt=" << transaction.retreat_attempt
        << " target_s=" << transaction.retreat_target_s
        << " retreat_hold_elapsed=" << transaction.retreat_hold_elapsed
        << " retreat_clear_elapsed=" << transaction.retreat_clear_elapsed
        << " pass_clear_elapsed=" << transaction.pass_clear_elapsed
        << " reason=" << transaction.reason;

    if (include_geometry) {
        const auto markers = rule_engine.conflictResourceMarkers(vehicles);
        for (const auto& marker : markers) {
            out << "\n  zone="
                << (marker.kind == multi_vehicle::ConflictMarkerKind::CONFLICT_RESERVATION
                        ? "RESERVED" : "POTENTIAL")
                << " pair=V" << marker.vehicle_a << "/V" << marker.vehicle_b
                << " raw=" << marker.raw_zone_index << " active=" << marker.active_zone_index
                << " a=[" << marker.s_a_enter << "," << marker.s_a_exit << "]"
                << " b=[" << marker.s_b_enter << "," << marker.s_b_exit << "]"
                << " holder=" << marker.holder_id << " waiter=" << marker.waiter_id;
        }
    }
    return out.str();
}

}  // namespace diagnostics
}  // namespace forklift_planner
