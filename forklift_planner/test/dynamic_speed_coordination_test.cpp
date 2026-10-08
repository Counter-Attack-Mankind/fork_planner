#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

int fail(const std::string& message) {
    std::cerr << "dynamic_speed_coordination_test: " << message << '\n';
    return 1;
}

RoughWp wp(double x, double y, double theta) {
    return RoughWp{x, y, theta, WpType::FORWARD};
}

VehicleAgent crossingVehicle(int id, double approach, bool vertical,
                             double speed = 0.0) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = 1;
    result.current_speed = speed;
    result.track.set(vertical
        ? RoughPath{wp(0.0, -approach, 1.5707963267948966),
                    wp(0.0, 2.0, 1.5707963267948966)}
        : RoughPath{wp(-approach, 0.0, 0.0),
                    wp(2.0, 0.0, 0.0)});
    return result;
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

PairInteractionResult baseline(const VehicleAgent& a, const VehicleAgent& b,
                               const MapParam& map_param,
                               const MultiVehicleConfig& config,
                               double horizon) {
    const std::vector<PotentialConflictZone> zones{broadZone(a, b)};
    return detectPairInteractionFromPredictions(
        a, b, zones,
        predictTrajectory(a, map_param, config, VehicleAction::NOMINAL,
                          horizon),
        predictTrajectory(b, map_param, config, VehicleAction::NOMINAL,
                          horizon));
}

PairSpeedCoordinationResult evaluate(
    const VehicleAgent& a, const VehicleAgent& b,
    const MapParam& map_param, const MultiVehicleConfig& config) {
    const auto nominal = baseline(a, b, map_param, config, 15.0);
    return evaluatePairSpeedCoordination(
        a, b, nominal, PriorityPhysicalTtcEvaluation{}, config, 0);
}

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_horizon = 15.0;
    config.prediction_step = 0.05;

    constexpr double eps = 1e-6;
    if (classifyDynamicInterventionBand(10.0, config) !=
            DynamicInterventionBand::FAR ||
        classifyDynamicInterventionBand(10.0 - eps, config) !=
            DynamicInterventionBand::MID ||
        classifyDynamicInterventionBand(5.0, config) !=
            DynamicInterventionBand::MID ||
        classifyDynamicInterventionBand(5.0 - eps, config) !=
            DynamicInterventionBand::NEAR) {
        return fail("configured FAR/MID/NEAR boundaries changed");
    }
    if (selectRollingSpeedAction(DynamicInterventionBand::FAR, false) !=
            VehicleAction::NOMINAL ||
        selectRollingSpeedAction(DynamicInterventionBand::MID, false) !=
            VehicleAction::YIELD ||
        selectRollingSpeedAction(DynamicInterventionBand::NEAR, false) !=
            VehicleAction::CREEP ||
        selectRollingSpeedAction(DynamicInterventionBand::FAR, true) !=
            VehicleAction::STOP) {
        return fail("rolling band-to-action mapping changed");
    }

    const VehicleAgent far_a = crossingVehicle(0, 2.50, false);
    VehicleAgent far_b = crossingVehicle(1, 2.90, true);
    far_b.path_s = 0.40;
    const auto far_result = evaluate(far_a, far_b, map_param, config);
    if (!far_result.action_selected ||
        far_result.selected_action_a != VehicleAction::NOMINAL ||
        far_result.selected_action_b != VehicleAction::NOMINAL) {
        return fail("FAR did not select NOMINAL/NOMINAL");
    }

    const VehicleAgent mid_a = crossingVehicle(0, 1.50, false);
    VehicleAgent mid_b = crossingVehicle(1, 1.90, true);
    mid_b.path_s = 0.40;
    const auto mid_baseline = baseline(mid_a, mid_b, map_param, config, 15.0);
    const auto mid_result = evaluate(mid_a, mid_b, map_param, config);
    if (!mid_baseline.event.valid ||
        classifyDynamicInterventionBand(mid_baseline.event.ttc_b, config) !=
            DynamicInterventionBand::MID ||
        !mid_result.action_selected ||
        mid_result.selected_action_a != VehicleAction::NOMINAL ||
        mid_result.selected_action_b != VehicleAction::YIELD) {
        return fail("MID did not select exactly one YIELD target");
    }

    // Role selection consumes the yielding vehicle's own TTC. Deliberately
    // make the priority TTC NEAR and the yielding TTC MID; the band must stay
    // MID instead of being driven by A or by first_overlap_t.
    PairInteractionResult split_baseline = mid_baseline;
    split_baseline.event.ttc_a = 1.0;
    split_baseline.event.ttc_b = 6.0;
    const auto split_result = evaluatePairSpeedCoordination(
        mid_a, mid_b, split_baseline, PriorityPhysicalTtcEvaluation{},
        config, mid_a.id);
    if (!split_result.effective_ttc_a || !split_result.effective_ttc_b ||
        near(*split_result.effective_ttc_a, *split_result.effective_ttc_b) ||
        !split_result.yielding_effective_ttc ||
        !near(*split_result.yielding_effective_ttc, 6.0) ||
        split_result.yielding_band != DynamicInterventionBand::MID) {
        return fail("yielding band still consumes a shared pair TTC");
    }

    const VehicleAgent near_a = crossingVehicle(0, 0.80, false);
    VehicleAgent near_b = crossingVehicle(1, 1.05, true);
    near_b.path_s = 0.25;
    const auto near_baseline = baseline(
        near_a, near_b, map_param, config, 15.0);
    const auto near_result = evaluate(near_a, near_b, map_param, config);
    if (!near_baseline.event.valid ||
        classifyDynamicInterventionBand(near_baseline.event.ttc_b, config) !=
            DynamicInterventionBand::NEAR ||
        near_result.selected_action_b != VehicleAction::CREEP) {
        return fail("NEAR did not jump directly to CREEP");
    }

    PairInteractionResult emergency_baseline = mid_baseline;
    emergency_baseline.event.ttc_a = 1.0;
    emergency_baseline.event.ttc_b = 1.0;
    PriorityPhysicalTtcEvaluation safe_priority_physical;
    safe_priority_physical.valid = true;
    safe_priority_physical.safety_ttc = 8.0;
    const auto emergency = evaluatePairSpeedCoordination(
        mid_a, mid_b, emergency_baseline, safe_priority_physical,
        config, mid_a.id);
    if (!emergency.emergency_stop ||
        emergency.selected_action_a != VehicleAction::NOMINAL ||
        emergency.selected_action_b != VehicleAction::STOP ||
        emergency.priority_safety_stop ||
        !emergency.yielding_safety_stop) {
        return fail("priority still stopped from effective TTC");
    }

    // Priority safety observes the other vehicle as one current physical OBB,
    // not as the other vehicle's NOMINAL future. A held/stationary vehicle
    // therefore remains visible, and an opposing-path hit reuses the existing
    // one-sided bridge correction from the physical collision_s seed.
    VehicleAgent physical_priority = crossingVehicle(20, 2.0, false, 0.0);
    VehicleAgent physical_other = crossingVehicle(21, 0.0, false, 0.0);
    physical_other.track.set(RoughPath{
        wp(2.0, 0.0, 3.14159265358979323846),
        wp(-2.0, 0.0, 3.14159265358979323846)});
    physical_other.path_s = 2.0;
    physical_other.ttc_stop_hold_remaining = config.rolling_refresh_period;
    const auto physical_prediction = predictTrajectory(
        physical_priority, map_param, config, VehicleAction::NOMINAL, 15.0);
    const auto physical = evaluatePriorityPhysicalTtc(
        physical_priority, physical_other, physical_prediction,
        map_param, config);
    if (!physical.valid || !physical.bridge_related ||
        physical.collision_t <= 0.0 ||
        physical.safety_ttc > physical.collision_t + 1e-9 ||
        physical.safety_boundary_s > physical.collision_s + 1e-9) {
        return fail("future-vs-current bridge physical TTC was not corrected");
    }

    // A MID action is a rolling 2 s response, not a full-horizon reservation.
    // Even if YIELD does not clear the whole rollout, do not escalate it or
    // reverse the supplied priority inside the current decision.
    std::optional<PairSpeedCoordinationResult> direct_mid;
    std::optional<double> raw_yield_delay;
    for (double speed_a : {0.0, 0.2, 0.4, 0.6}) {
      for (double speed_b : {0.0, 0.2, 0.4, 0.6}) {
       for (double approach_a = 0.5;
             approach_a <= 4.0 && !direct_mid; approach_a += 0.1) {
        for (double approach_b = 0.5;
             approach_b <= 4.0; approach_b += 0.1) {
            const VehicleAgent a = crossingVehicle(
                0, approach_a, false, speed_a);
            const VehicleAgent b = crossingVehicle(
                1, approach_b, true, speed_b);
            const auto nominal = baseline(a, b, map_param, config, 15.0);
            if (!nominal.event.valid ||
                classifyDynamicInterventionBand(nominal.event.ttc_b,
                                                config) !=
                    DynamicInterventionBand::MID) {
                continue;
            }
            const auto raw_yield = evaluateSelectedAction(
                a, b, std::vector<PotentialConflictZone>{broadZone(a, b)},
                map_param, config, 15.0, VehicleAction::NOMINAL,
                VehicleAction::YIELD, nominal.event.first_overlap_t);
            if (!raw_yield.conflict_free && raw_yield.conflict_delay &&
                *raw_yield.conflict_delay > config.prediction_step) {
                const auto result = evaluate(a, b, map_param, config);
                if (result.action_selected &&
                    result.selected_winner_id == a.id &&
                    result.selected_action_a == VehicleAction::NOMINAL &&
                    result.selected_action_b == VehicleAction::YIELD) {
                    direct_mid = result;
                    raw_yield_delay = raw_yield.conflict_delay;
                }
                break;
            }
        }
       }
      }
    }
    if (!direct_mid || !raw_yield_delay) {
        return fail("remaining MID/YIELD conflict changed direct mapping");
    }

    // Classification metadata must not change generic rolling control.
    PairInteractionResult crossing_label = mid_baseline;
    crossing_label.type = PairInteractionType::CROSSING;
    PairInteractionResult opposing_label = mid_baseline;
    opposing_label.type = PairInteractionType::OPPOSING;
    PairInteractionResult following_label = mid_baseline;
    following_label.type = PairInteractionType::SAME_DIRECTION;
    const auto crossing_result = evaluatePairSpeedCoordination(
        mid_a, mid_b, crossing_label, PriorityPhysicalTtcEvaluation{},
        config, mid_a.id);
    const auto opposing_result = evaluatePairSpeedCoordination(
        mid_a, mid_b, opposing_label, PriorityPhysicalTtcEvaluation{},
        config, mid_a.id);
    const auto following_result = evaluatePairSpeedCoordination(
        mid_a, mid_b, following_label, PriorityPhysicalTtcEvaluation{},
        config, mid_a.id);
    const auto sameControl = [&](const PairSpeedCoordinationResult& result) {
        return result.selected_winner_id == crossing_result.selected_winner_id &&
               result.selected_action_a == crossing_result.selected_action_a &&
               result.selected_action_b == crossing_result.selected_action_b &&
               result.reason == crossing_result.reason;
    };
    if (!sameControl(opposing_result) || !sameControl(following_result)) {
        return fail("interaction label still changes generic pair control");
    }

    const auto fixed_order = evaluatePairSpeedCoordination(
        mid_a, mid_b, mid_baseline, PriorityPhysicalTtcEvaluation{},
        config, mid_b.id);
    if (fixed_order.selected_winner_id != mid_b.id ||
        fixed_order.selected_action_a != VehicleAction::YIELD ||
        fixed_order.selected_action_b != VehicleAction::NOMINAL) {
        return fail("generic coordinator changed supplied priority order");
    }

    VehicleAgent low = crossingVehicle(2, 1.0, false, 0.0);
    const auto nominal_prediction = predictTrajectory(
        low, map_param, config, VehicleAction::NOMINAL, 0.1);
    if (nominal_prediction.size() < 2 ||
        !near(nominal_prediction[1].speed,
              config.max_accel * config.prediction_step)) {
        return fail("NOMINAL acceleration constraint changed");
    }
    VehicleAgent fast = crossingVehicle(
        3, 1.0, false, config.nominal_speed);
    const auto yielding = predictTrajectory(
        fast, map_param, config, VehicleAction::YIELD, 0.1);
    if (yielding.size() < 2 ||
        !near(yielding[1].speed,
              config.nominal_speed -
                  config.max_decel * config.prediction_step)) {
        return fail("YIELD deceleration constraint changed");
    }

    const VehicleAgent before_a = mid_a;
    const VehicleAgent before_b = mid_b;
    for (int repeat = 0; repeat < 3; ++repeat) {
        (void)evaluate(mid_a, mid_b, map_param, config);
    }
    if (!near(mid_a.path_s, before_a.path_s) ||
        !near(mid_a.current_speed, before_a.current_speed) ||
        mid_a.action != before_a.action ||
        !near(mid_b.path_s, before_b.path_s) ||
        !near(mid_b.current_speed, before_b.current_speed) ||
        mid_b.action != before_b.action) {
        return fail("selected-action evaluation changed live input");
    }

    const auto base_stop = evaluateTtcStopBoundary(
        1.3, VehicleAction::CREEP, config);
    const double expected_threshold =
        config.rolling_refresh_period +
        config.nominal_speed * config.creep_ratio / config.max_decel +
        config.dynamic_stop_time_margin;
    if (!near(base_stop.stop_threshold, expected_threshold) ||
        !base_stop.stop_required ||
        evaluateTtcStopBoundary(3.8, VehicleAction::CREEP, config).
            stop_required ||
        !evaluateTtcStopBoundary(expected_threshold, VehicleAction::CREEP,
                                 config).stop_required) {
        return fail("TTC STOP boundary value or inclusive edge changed");
    }

    MultiVehicleConfig shorter_refresh = config;
    shorter_refresh.rolling_refresh_period = 1.0;
    if (!evaluateTtcStopBoundary(1.5, VehicleAction::CREEP, config).
            stop_required ||
        evaluateTtcStopBoundary(1.5, VehicleAction::CREEP,
                                shorter_refresh).stop_required) {
        return fail("rolling refresh period did not move TTC STOP boundary");
    }

    MultiVehicleConfig faster_creep = config;
    faster_creep.creep_ratio = 0.50;
    if (evaluateTtcStopBoundary(2.35, VehicleAction::CREEP, config).
            stop_required ||
        !evaluateTtcStopBoundary(2.35, VehicleAction::CREEP,
                                 faster_creep).stop_required) {
        return fail("creep ratio did not move TTC STOP boundary");
    }

    MultiVehicleConfig weaker_brake = config;
    weaker_brake.max_decel = 0.10;
    if (evaluateTtcStopBoundary(2.40, VehicleAction::CREEP, config).
            stop_required ||
        !evaluateTtcStopBoundary(2.40, VehicleAction::CREEP,
                                 weaker_brake).stop_required) {
        return fail("max deceleration did not move TTC STOP boundary");
    }

    // Distinct vehicle TTCs and action speeds must produce distinct safety
    // decisions: priority=NOMINAL needs more braking time here, while the
    // yielding CREEP vehicle remains outside its own STOP boundary.
    const double distinct_priority_ttc = 2.50;
    const double distinct_yielding_ttc = 3.00;
    const auto distinct_priority_stop = evaluateTtcStopBoundary(
        distinct_priority_ttc, VehicleAction::NOMINAL, config);
    const auto distinct_yielding_stop = evaluateTtcStopBoundary(
        distinct_yielding_ttc, VehicleAction::CREEP, config);
    if (near(distinct_priority_ttc, distinct_yielding_ttc) ||
        !distinct_priority_stop.stop_required ||
        distinct_yielding_stop.stop_required) {
        return fail("priority/yield STOP decisions still share one TTC");
    }

    // An immediate corrected baseline conflict independently stops priority
    // and yielding; priority identity remains stable.
    VehicleAgent immediate_a = crossingVehicle(10, 0.0, false, 0.0);
    VehicleAgent immediate_b = crossingVehicle(11, 0.0, false, 0.0);
    immediate_b.track.set(RoughPath{
        wp(0.0, 0.0, 3.14159265358979323846),
        wp(-2.0, 0.0, 3.14159265358979323846)});
    PairInteractionResult immediate_baseline;
    immediate_baseline.type = PairInteractionType::OPPOSING;
    immediate_baseline.event.valid = true;
    immediate_baseline.event.first_overlap_t = 0.0;
    immediate_baseline.event.collision_s_a = 0.0;
    immediate_baseline.event.collision_s_b = 0.0;
    immediate_baseline.event.danger_s_a = 0.0;
    immediate_baseline.event.danger_s_b = 0.0;
    immediate_baseline.event.ttc_a = 0.0;
    immediate_baseline.event.ttc_b = 0.0;
    PriorityPhysicalTtcEvaluation immediate_physical;
    immediate_physical.valid = true;
    immediate_physical.safety_ttc = 0.0;
    const auto unresolved = evaluatePairSpeedCoordination(
        immediate_a, immediate_b, immediate_baseline, immediate_physical,
        config, immediate_b.id);
    if (unresolved.selected_action_a != VehicleAction::STOP ||
        unresolved.selected_action_b != VehicleAction::STOP ||
        !unresolved.priority_safety_stop ||
        !unresolved.priority_physical_ttc ||
        !evaluateTtcStopBoundary(
             *unresolved.priority_physical_ttc,
             VehicleAction::NOMINAL, config).stop_required ||
        unresolved.selected_winner_id != immediate_b.id ||
        unresolved.reason != "rolling_emergency_stop") {
        return fail("braking emergency changed stable-priority response");
    }

    std::cout << "[MID-DIRECT] first_overlap_t="
              << *direct_mid->first_overlap_t
              << " raw_yield_delay=" << *raw_yield_delay
              << " selected_action="
              << static_cast<int>(direct_mid->selected_action_b) << '\n';
    std::cout << "dynamic_speed_coordination_test: PASS\n";
    return 0;
}
