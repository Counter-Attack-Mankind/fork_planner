#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

constexpr double kPi = 3.14159265358979323846;

int fail(const std::string& message) {
    std::cerr << "prediction_execution_consistency_test: " << message << '\n';
    return 1;
}

RoughWp wp(double x, double y, double theta) {
    return RoughWp{x, y, theta, WpType::FORWARD};
}

double actionTarget(VehicleAction action, const MultiVehicleConfig& config) {
    switch (action) {
        case VehicleAction::STOP: return 0.0;
        case VehicleAction::CREEP:
            return config.nominal_speed * config.creep_ratio;
        case VehicleAction::YIELD:
            return config.nominal_speed * config.yield_ratio;
        case VehicleAction::NOMINAL: return config.nominal_speed;
        case VehicleAction::BOOST:
            return config.enable_boost
                ? std::min(config.max_speed,
                           config.nominal_speed * config.boost_ratio)
                : config.nominal_speed;
    }
    return 0.0;
}

// Independent reference for the current simulation implementation in
// MultiVehiclePatrolNode::curvatureSpeed(), limitedSpeed(), and
// advanceVehicles().  Keep this test-side reference independent from the
// predictor so drift between the two paths remains observable.
double executionCurvatureSpeed(const VehicleAgent& vehicle,
                               const MultiVehicleConfig& config) {
    if (config.lat_accel_max <= 0.0 || vehicle.track.empty()) return 1e9;
    const double length = vehicle.track.length();
    const double s = std::clamp(vehicle.path_s, 0.0, length);
    constexpr double ds = 0.05;
    const RoughWp a = vehicle.track.poseAtS(std::max(0.0, s - ds));
    const RoughWp b = vehicle.track.poseAtS(s);
    const RoughWp c = vehicle.track.poseAtS(std::min(length, s + ds));
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double acx = c.x - a.x;
    const double acy = c.y - a.y;
    const double lab = std::hypot(abx, aby);
    const double lbc = std::hypot(c.x - b.x, c.y - b.y);
    const double lac = std::hypot(acx, acy);
    if (lab < 1e-4 || lbc < 1e-4 || lac < 1e-4) return 1e9;
    const double kappa =
        2.0 * std::abs(abx * acy - aby * acx) / (lab * lbc * lac);
    if (kappa < 1e-3) return 1e9;
    return std::max(std::sqrt(config.lat_accel_max / kappa),
                    config.nominal_speed * config.creep_ratio);
}

void advanceExecution(VehicleAgent& vehicle, VehicleAction action,
                      const MultiVehicleConfig& config, double dt) {
    if (!vehicle.active() || vehicle.track.empty()) return;
    const double desired = std::min(actionTarget(action, config),
                                    executionCurvatureSpeed(vehicle, config));
    if (desired > vehicle.current_speed) {
        vehicle.current_speed =
            std::min(desired, vehicle.current_speed + config.max_accel * dt);
    } else {
        vehicle.current_speed =
            std::max(desired, vehicle.current_speed - config.max_decel * dt);
    }
    vehicle.path_s = std::min(
        vehicle.track.length(), vehicle.path_s + vehicle.current_speed * dt);
    if (vehicle.path_s >= vehicle.track.length() - 1e-9) {
        vehicle.current_speed = 0.0;
    }
}

VehicleAgent straightVehicle(double length, double speed) {
    VehicleAgent vehicle;
    vehicle.id = 0;
    vehicle.mode = VehicleMode::ACTIVE;
    vehicle.action = VehicleAction::NOMINAL;
    vehicle.requested_action = VehicleAction::NOMINAL;
    vehicle.current_speed = speed;
    vehicle.path_gen = 1;
    vehicle.track.set(RoughPath{wp(0.0, 0.0, 0.0),
                                wp(length, 0.0, 0.0)});
    return vehicle;
}

VehicleAgent crossingVehicle(int id, double approach, bool vertical,
                             double speed) {
    VehicleAgent vehicle;
    vehicle.id = id;
    vehicle.mode = VehicleMode::ACTIVE;
    vehicle.action = VehicleAction::NOMINAL;
    vehicle.requested_action = VehicleAction::NOMINAL;
    vehicle.current_speed = speed;
    vehicle.path_gen = 1;
    vehicle.track.set(vertical
        ? RoughPath{wp(0.0, -approach, kPi * 0.5),
                    wp(0.0, 2.0, kPi * 0.5)}
        : RoughPath{wp(-approach, 0.0, 0.0),
                    wp(2.0, 0.0, 0.0)});
    return vehicle;
}

PotentialConflictZone broadZone(const VehicleAgent& a,
                                const VehicleAgent& b) {
    PotentialConflictZone zone;
    zone.s_self_enter = 0.0;
    zone.s_self_exit = a.track.length();
    zone.s_other_enter = 0.0;
    zone.s_other_exit = b.track.length();
    return zone;
}

struct Difference {
    double speed = 0.0;
    double path_s = 0.0;
    double position = 0.0;
    double yaw = 0.0;
};

Difference comparePrefix(const MapParam& map_param,
                         const MultiVehicleConfig& config,
                         VehicleAgent initial, VehicleAction action,
                         double execution_step = 0.1) {
    const auto predicted = predictTrajectory(
        initial, map_param, config, action, 2.0);
    VehicleAgent executed = initial;
    for (double t = 0.0; t < 2.0 - 1e-12; t += execution_step) {
        advanceExecution(executed, action, config, execution_step);
    }
    const RoughWp predicted_pose = initial.track.poseAtS(predicted.back().s);
    const RoughWp executed_pose = executed.track.poseAtS(executed.path_s);
    return Difference{
        std::abs(predicted.back().speed - executed.current_speed),
        std::abs(predicted.back().s - executed.path_s),
        std::hypot(predicted_pose.x - executed_pose.x,
                   predicted_pose.y - executed_pose.y),
        std::abs(std::remainder(predicted_pose.theta - executed_pose.theta,
                                2.0 * kPi))};
}

bool conflictAfterExecutionPrefix(
    const MapParam& map_param, const MultiVehicleConfig& config,
    VehicleAgent a, VehicleAgent b, VehicleAction action_a,
    VehicleAction action_b, bool* initial_clear) {
    const std::vector<PotentialConflictZone> zones{broadZone(a, b)};
    const PairInteractionResult full = detectPairInteractionFromPredictions(
        a, b, zones,
        predictTrajectory(a, map_param, config, action_a, 15.0),
        predictTrajectory(b, map_param, config, action_b, 15.0));
    *initial_clear = !full.event.valid;
    for (int step = 0; step < 20; ++step) {
        advanceExecution(a, action_a, config, 0.1);
        advanceExecution(b, action_b, config, 0.1);
    }
    const PairInteractionResult remaining = detectPairInteractionFromPredictions(
        a, b, zones,
        predictTrajectory(a, map_param, config, action_a, 13.0),
        predictTrajectory(b, map_param, config, action_b, 13.0));
    return remaining.event.valid;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_step = 0.05;
    config.prediction_horizon = 15.0;

    struct Case {
        const char* name;
        VehicleAction action;
        double speed;
    };
    const std::vector<Case> cases{
        {"NOMINAL", VehicleAction::NOMINAL, 0.0},
        {"YIELD", VehicleAction::YIELD, config.nominal_speed},
        {"CREEP", VehicleAction::CREEP, config.nominal_speed},
        {"STOP", VehicleAction::STOP, config.nominal_speed},
        {"BOOST", VehicleAction::BOOST, config.nominal_speed}};

    double max_path_difference = 0.0;
    std::cout << std::fixed << std::setprecision(9);
    for (const Case& item : cases) {
        const Difference difference = comparePrefix(
            map_param, config, straightVehicle(5.0, item.speed), item.action);
        max_path_difference = std::max(max_path_difference, difference.path_s);
        std::cout << "[CONSISTENCY] action=" << item.name
                  << " speed_diff=" << difference.speed
                  << " path_s_diff=" << difference.path_s
                  << " pose_diff=" << difference.position
                  << " yaw_diff=" << difference.yaw << '\n';
        if (difference.speed > 1e-9 || difference.path_s > 0.015 + 1e-9 ||
            difference.position > 0.015 + 1e-9 || difference.yaw > 1e-9) {
            return fail(std::string(item.name) +
                        " exceeded the documented two-second discretization bound");
        }
    }

    // A deliberately slow transition proves that a target action can span
    // more than one 2 s rolling period without resetting current_speed.
    MultiVehicleConfig slow = config;
    slow.max_decel = 0.02;
    VehicleAgent multi_period = straightVehicle(5.0, slow.nominal_speed);
    for (int step = 0; step < 20; ++step) {
        advanceExecution(multi_period, VehicleAction::YIELD, slow, 0.1);
    }
    const double speed_at_two_seconds = multi_period.current_speed;
    if (!(speed_at_two_seconds > slow.nominal_speed * slow.yield_ratio)) {
        return fail("slow YIELD unexpectedly completed inside one rolling period");
    }
    for (int step = 0; step < 20; ++step) {
        advanceExecution(multi_period, VehicleAction::YIELD, slow, 0.1);
    }
    if (!(multi_period.current_speed < speed_at_two_seconds)) {
        return fail("YIELD did not continue from the intermediate speed");
    }
    std::cout << "[ROLLING-CONTINUITY] speed_t2=" << speed_at_two_seconds
              << " speed_t4=" << multi_period.current_speed
              << " target="
              << slow.nominal_speed * slow.yield_ratio << '\n';

    // Deterministic near-boundary sweep: a candidate that is CLEAR in the
    // 0.05 s predictor must not become conflicting solely because the first
    // 2 s are executed with the current 0.1 s simulation integrator.
    unsigned long long clear_candidates = 0;
    unsigned long long unsafe_flips = 0;
    for (double offset = 0.0; offset <= 2.5 + 1e-12; offset += 0.005) {
        for (double initial_speed : {0.0, 0.05, 0.10, 0.20}) {
            for (VehicleAction yielding_action :
                 {VehicleAction::YIELD, VehicleAction::CREEP}) {
                bool initial_clear = false;
                const bool remaining_conflict = conflictAfterExecutionPrefix(
                    map_param, config,
                    crossingVehicle(0, 0.30 + offset, false, initial_speed),
                    crossingVehicle(1, 0.70 + offset, true, initial_speed),
                    VehicleAction::NOMINAL, yielding_action, &initial_clear);
                if (!initial_clear) continue;
                ++clear_candidates;
                if (remaining_conflict) ++unsafe_flips;
            }
        }
    }
    std::cout << "[CONFLICT-CONSISTENCY] clear_candidates="
              << clear_candidates << " unsafe_clear_to_conflict="
              << unsafe_flips << " max_path_s_diff="
              << max_path_difference << '\n';
    if (clear_candidates == 0) {
        return fail("boundary sweep did not exercise any clear candidates");
    }
    if (unsafe_flips != 0) {
        return fail("predictor CLEAR became conflicting after 0.1 s execution prefix");
    }

    std::cout << "prediction_execution_consistency_test: PASS\n";
    return 0;
}
