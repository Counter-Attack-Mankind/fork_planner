#include "forklift_planner/multi_vehicle/rule_engine.h"

#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

constexpr double kHalfPi = 1.5707963267948966;

int fail(const std::string& message) {
    std::cerr << "rolling_decision_timing_test: " << message << '\n';
    return 1;
}

RoughWp wp(double x, double y, double theta) {
    return RoughWp{x, y, theta, WpType::FORWARD};
}

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

VehicleAgent crossingVehicle(int id, double approach, bool vertical,
                             double speed = 0.0) {
    VehicleAgent vehicle;
    vehicle.id = id;
    vehicle.mode = VehicleMode::ACTIVE;
    vehicle.action = VehicleAction::NOMINAL;
    vehicle.requested_action = VehicleAction::NOMINAL;
    vehicle.path_gen = 1;
    vehicle.current_speed = speed;
    vehicle.track.set(vertical
        ? RoughPath{wp(0.0, -approach, kHalfPi),
                    wp(0.0, 2.0, kHalfPi)}
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

void advanceByAction(VehicleAgent& vehicle, VehicleAction target,
                     const MapParam& map_param,
                     const MultiVehicleConfig& config, double dt) {
    const auto prediction = predictTrajectory(
        vehicle, map_param, config, target, dt);
    vehicle.path_s = prediction.back().s;
    vehicle.current_speed = prediction.back().speed;
    vehicle.action = target;
}

struct PeriodTarget {
    VehicleAction action = VehicleAction::NOMINAL;
    int blocker_id = -1;
    std::string reason;
};

std::vector<PeriodTarget> captureTargets(
    const std::vector<VehicleAgent>& vehicles) {
    std::vector<PeriodTarget> targets;
    for (const VehicleAgent& vehicle : vehicles) {
        targets.push_back(PeriodTarget{
            vehicle.requested_action, vehicle.blocker_id, vehicle.reason});
    }
    return targets;
}

void restoreTargets(std::vector<VehicleAgent>& vehicles,
                    const std::vector<PeriodTarget>& targets) {
    for (size_t index = 0; index < vehicles.size(); ++index) {
        vehicles[index].requested_action = targets[index].action;
        vehicles[index].blocker_id = targets[index].blocker_id;
        vehicles[index].reason = targets[index].reason;
    }
}

std::optional<std::vector<VehicleAgent>> findFarBoundaryFixture(
    RuleEngine& engine, const MultiVehicleConfig& config) {
    for (double approach_a = 1.7; approach_a <= 2.5; approach_a += 0.02) {
        for (double approach_b = approach_a + 0.2;
             approach_b <= approach_a + 0.8; approach_b += 0.02) {
            std::vector<VehicleAgent> vehicles{
                crossingVehicle(0, approach_a, false),
                crossingVehicle(1, approach_b, true)};
            const auto baseline = engine.detectPairInteraction(
                vehicles[0], vehicles[1], 15.0);
            if (baseline.event.valid && baseline.event.ttc_b >= 10.05 &&
                baseline.event.ttc_b <= 10.25) {
                return vehicles;
            }
        }
    }
    return std::nullopt;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_horizon = 15.0;
    config.prediction_step = 0.05;

    // FAR near the 10 s boundary: frame 0 decides NOMINAL once.  Even after
    // the predicted true state enters MID inside the same 2 s period, reuse
    // mode must retain that ordinary target.
    RuleEngine far_engine(map_param, config);
    auto far_fixture = findFarBoundaryFixture(far_engine, config);
    if (!far_fixture) return fail("could not construct FAR boundary fixture");
    std::vector<VehicleAgent> far = *far_fixture;
    const double far_start_t = far_engine.detectPairInteraction(
        far[0], far[1], 15.0).event.first_overlap_t;
    far_engine.decide(far, 0.1, 15.0);
    if (!far_engine.lastRollingDynamicDecision().valid ||
        far_engine.lastRollingDynamicDecision().band !=
            DynamicInterventionBand::FAR ||
        far[0].requested_action != VehicleAction::NOMINAL ||
        far[1].requested_action != VehicleAction::NOMINAL) {
        return fail("period start did not select FAR/NOMINAL");
    }
    const std::vector<PeriodTarget> far_targets = captureTargets(far);
    const auto far_decision = far_engine.lastRollingDynamicDecision();
    const auto mid_before = far_engine.dynamicSpeedMetrics().mid_decisions;
    bool observed_mid_state = false;
    double observed_mid_t = -1.0;
    for (int frame = 0; frame < 20; ++frame) {
        if (frame > 0) {
            restoreTargets(far, far_targets);
            const auto future_baseline = far_engine.detectPairInteraction(
                far[0], far[1], 15.0);
            if (future_baseline.event.valid &&
                future_baseline.event.ttc_b >=
                    config.dynamic_speed_near_threshold &&
                future_baseline.event.ttc_b <
                    config.dynamic_speed_far_threshold) {
                observed_mid_state = true;
                observed_mid_t = future_baseline.event.ttc_b;
            }
            far_engine.decide(far, 0.1, 15.0 - frame * 0.1,
                              /*reuse_ordinary_coordination=*/true,
                              &far_decision);
            if (far[0].requested_action != VehicleAction::NOMINAL ||
                far[1].requested_action != VehicleAction::NOMINAL) {
                return fail("future MID state changed the FAR period target");
            }
        }
        advanceByAction(far[0], far[0].requested_action,
                        map_param, config, 0.1);
        advanceByAction(far[1], far[1].requested_action,
                        map_param, config, 0.1);
    }
    if (!observed_mid_state ||
        far_engine.dynamicSpeedMetrics().mid_decisions != mid_before) {
        return fail("reuse period did not exercise a skipped MID decision");
    }
    const auto refresh_baseline = far_engine.detectPairInteraction(
        far[0], far[1], 15.0);
    if (!refresh_baseline.event.valid ||
        refresh_baseline.event.ttc_b >= config.dynamic_speed_far_threshold) {
        return fail("2 s refresh fixture did not enter MID/NEAR");
    }
    far_engine.decide(far, 0.1, 15.0);
    if (far[0].requested_action == VehicleAction::NOMINAL &&
        far[1].requested_action == VehicleAction::NOMINAL) {
        return fail("new rolling period did not make a new intervention");
    }

    // MID target persists for the whole period even if a future re-evaluation
    // would report CLEAR and previously recover to NOMINAL.
    RuleEngine mid_engine(map_param, config);
    std::vector<VehicleAgent> mid{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    mid[1].path_s = 0.40;
    mid_engine.decide(mid, 0.1, 15.0);
    const std::vector<PeriodTarget> mid_targets = captureTargets(mid);
    const auto mid_decision = mid_engine.lastRollingDynamicDecision();
    if (mid_targets[0].action == VehicleAction::NOMINAL &&
        mid_targets[1].action == VehicleAction::NOMINAL) {
        return fail("MID fixture did not select YIELD/CREEP");
    }
    for (int frame = 0; frame < 20; ++frame) {
        if (frame > 0) {
            restoreTargets(mid, mid_targets);
            mid_engine.decide(mid, 0.1, 15.0 - frame * 0.1, true,
                              &mid_decision);
            if (mid[0].requested_action != mid_targets[0].action ||
                mid[1].requested_action != mid_targets[1].action) {
                return fail("MID target changed inside one rolling period");
            }
        }
        advanceByAction(mid[0], mid[0].requested_action,
                        map_param, config, 0.1);
        advanceByAction(mid[1], mid[1].requested_action,
                        map_param, config, 0.1);
    }

    // The 15 s NOMINAL baseline classifies the conflict, while the selected
    // MID response remains a direct YIELD for the next rolling period even if
    // that action alone would not clear the complete horizon.
    bool found_delayed_candidate = false;
    double delayed_candidate_t = -1.0;
    for (double speed_a : {0.0, 0.2, 0.4, 0.6}) {
      for (double speed_b : {0.0, 0.2, 0.4, 0.6}) {
       for (double approach_a = 0.5;
            approach_a <= 4.0 && !found_delayed_candidate;
            approach_a += 0.1) {
        for (double approach_b = 0.5;
             approach_b <= 4.0; approach_b += 0.1) {
            VehicleAgent candidate_a = crossingVehicle(
                0, approach_a, false, speed_a);
            VehicleAgent candidate_b = crossingVehicle(
                1, approach_b, true, speed_b);
            const std::vector<PotentialConflictZone> candidate_zones{
                broadZone(candidate_a, candidate_b)};
            const auto candidate_baseline =
                detectPairInteractionFromPredictions(
                    candidate_a, candidate_b, candidate_zones,
                    predictTrajectory(candidate_a, map_param, config,
                                      VehicleAction::NOMINAL, 15.0),
                    predictTrajectory(candidate_b, map_param, config,
                                      VehicleAction::NOMINAL, 15.0));
            if (!candidate_baseline.event.valid) continue;
            if (classifyDynamicInterventionBand(
                    candidate_baseline.event.ttc_b, config) !=
                DynamicInterventionBand::MID) {
                continue;
            }
            const auto candidate_result = evaluatePairSpeedCoordination(
                candidate_a, candidate_b, candidate_baseline,
                PriorityPhysicalTtcEvaluation{}, config, 0);
            const auto raw_yield = evaluateSelectedAction(
                candidate_a, candidate_b, candidate_zones, map_param, config,
                15.0, VehicleAction::NOMINAL, VehicleAction::YIELD,
                candidate_baseline.event.first_overlap_t);
            if (candidate_result.action_selected &&
                candidate_result.selected_winner_id == candidate_a.id &&
                candidate_result.selected_action_a == VehicleAction::NOMINAL &&
                candidate_result.selected_action_b == VehicleAction::YIELD &&
                !raw_yield.conflict_free) {
                found_delayed_candidate = true;
                delayed_candidate_t = candidate_baseline.event.first_overlap_t;
                break;
            }
        }
       }
      }
    }
    if (!found_delayed_candidate) {
        return fail("MID response still depends on full-horizon clearance");
    }

    // Reproduce the stage-3.1 contamination shape: frame 0 selects a normal
    // NEAR/CREEP action, then future 0.1 s frames move into the old geometric
    // interval. Reuse must preserve the period motion target and must not
    // create an already-inside reservation.
    RuleEngine inside_engine(map_param, config);
    std::vector<VehicleAgent> inside{
        crossingVehicle(0, 0.80, false),
        crossingVehicle(1, 1.05, true)};
    inside[1].path_s = 0.25;
    inside_engine.decide(inside, 0.1, 15.0);
    const auto inside_decision =
        inside_engine.lastRollingDynamicDecision();
    const auto inside_targets = captureTargets(inside);
    if (!inside_decision.valid ||
        inside_decision.band != DynamicInterventionBand::NEAR ||
        inside_engine.snapshot().reservations.size() != 0) {
        return fail("already-inside rollout fixture did not start dynamic");
    }
    for (int frame = 0; frame < 5; ++frame) {
        if (frame > 0) {
            restoreTargets(inside, inside_targets);
            inside_engine.decide(inside, 0.1, 15.0 - frame * 0.1, true,
                                 &inside_decision);
            if (!inside_engine.snapshot().reservations.empty() ||
                inside[0].requested_action != inside_targets[0].action ||
                inside[1].requested_action != inside_targets[1].action) {
                return fail("future inside frame polluted rolling action");
            }
        }
        advanceByAction(inside[0], inside[0].requested_action,
                        map_param, config, 0.1);
        advanceByAction(inside[1], inside[1].requested_action,
                        map_param, config, 0.1);
    }

    // Pair-wide A1 reservations from an older snapshot are obsolete. Reuse
    // mode must purge them instead of skipping ordinary rolling coordination.
    RuleEngine safety_engine(map_param, config);
    std::vector<VehicleAgent> safety{
        crossingVehicle(0, 0.30, false, config.nominal_speed),
        crossingVehicle(1, 0.70, true, config.nominal_speed)};
    RuleEngine::SimSnapshot safety_state;
    RuleEngine::ConflictReservation reservation;
    reservation.owner_id = 0;
    reservation.gen_lo = 1;
    reservation.gen_hi = 1;
    reservation.enter_lo = 0.10;
    reservation.exit_lo = 0.70;
    reservation.enter_hi = 0.05;
    reservation.exit_hi = 1.00;
    reservation.create_reason = "a1_related";
    safety_state.reservations[{0, 1}] = reservation;
    safety_engine.restore(safety_state);
    safety_engine.decide(safety, 0.1, 15.0, true);
    if (safety[1].requested_action != VehicleAction::NOMINAL ||
        !safety_engine.snapshot().reservations.empty()) {
        return fail("obsolete A1 reservation still overrode reused NOMINAL");
    }

    // A TTC safety STOP owns one complete rolling period. Its prediction
    // decelerates from the current real speed before becoming stationary; the
    // planning permission returns only after all 20 x 0.1 s frames elapsed.
    RuleEngine hold_engine(map_param, config);
    std::vector<VehicleAgent> held{
        crossingVehicle(30, 2.0, false, config.nominal_speed)};
    held[0].ttc_stop_hold_remaining = config.rolling_refresh_period;
    const auto stop_prediction = predictTrajectory(
        held[0], map_param, config, VehicleAction::STOP, 1.0);
    if (stop_prediction.size() < 2 ||
        !near(stop_prediction.front().speed, config.nominal_speed) ||
        stop_prediction[1].speed <= 0.0 ||
        stop_prediction[1].speed >= stop_prediction.front().speed ||
        stop_prediction[1].s <= stop_prediction.front().s) {
        return fail("held STOP prediction froze a moving vehicle instantly");
    }
    for (int frame = 0; frame < 20; ++frame) {
        hold_engine.decide(held, 0.1, 15.0,
                           /*reuse_ordinary_coordination=*/true);
        if (held[0].requested_action != VehicleAction::STOP ||
            held[0].action != VehicleAction::STOP) {
            return fail("TTC STOP hold released inside the rolling period");
        }
    }
    if (!near(held[0].ttc_stop_hold_remaining, 0.0, 1e-8)) {
        return fail("TTC STOP hold did not consume exactly one period");
    }
    hold_engine.decide(held, 0.1, 15.0);
    if (held[0].requested_action != VehicleAction::NOMINAL) {
        return fail("expired TTC STOP hold did not restore planning permission");
    }

    std::cout << "[ROLLING-FAR] start_first_t=" << far_start_t
              << " future_mid_first_t=" << observed_mid_t
              << " frames_nominal=20\n";
    std::cout << "[ROLLING-REFRESH] first_t="
              << refresh_baseline.event.first_overlap_t << " new_target="
              << actionName(far[0].requested_action) << "/"
              << actionName(far[1].requested_action) << '\n';
    std::cout << "[FULL-HORIZON-ACTION] baseline_conflict_t="
              << delayed_candidate_t << " escalated_clear=true\n";
    std::cout << "rolling_decision_timing_test: PASS\n";
    return 0;
}
