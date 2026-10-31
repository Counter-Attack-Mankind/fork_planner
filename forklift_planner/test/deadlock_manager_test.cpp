#include <cmath>
#include <iostream>
#include <vector>

#include "forklift_planner/multi_vehicle/deadlock/deadlock_manager.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"

namespace {
int fail(const char* message) {
    std::cerr << "deadlock_manager_test: " << message << "\n";
    return 1;
}
}  // namespace

int main() {
    using namespace forklift_planner::multi_vehicle;

    MapParam map;
    MultiVehicleConfig config;
    config.deadlock_retreat_distance = 0.50;
    config.deadlock_retreat_max_attempts = 3;
    config.rolling_refresh_period = 2.0;
    config.prediction_horizon = 15.0;

    VehicleAgent retreat;
    retreat.id = 0;
    retreat.mode = VehicleMode::ACTIVE;
    retreat.action = VehicleAction::STOP;
    retreat.requested_action = VehicleAction::STOP;
    retreat.blocker_id = 1;
    retreat.path_gen = 3;
    retreat.path_s = 1.50;
    retreat.track.set(RoughPath{
        RoughWp{0.0, 0.0, 0.0, WpType::FORWARD},
        RoughWp{4.0, 0.0, 0.0, WpType::FORWARD}});

    VehicleAgent passer;
    passer.id = 1;
    passer.mode = VehicleMode::ACTIVE;
    passer.action = VehicleAction::STOP;
    passer.requested_action = VehicleAction::STOP;
    passer.blocker_id = 0;
    passer.path_gen = 7;
    passer.path_s = 2.00;
    passer.track.set(RoughPath{
        RoughWp{0.0, 5.0, 0.0, WpType::FORWARD},
        RoughWp{4.0, 5.0, 0.0, WpType::FORWARD}});

    std::vector<VehicleAgent> vehicles{retreat, passer};

    DeadlockPairGeometry geometry;
    geometry.vehicle_a = retreat.id;
    geometry.vehicle_b = passer.id;
    geometry.path_gen_a = retreat.path_gen;
    geometry.path_gen_b = passer.path_gen;
    geometry.preferred_priority_vehicle_id = passer.id;

    MultiVehicleConfig selection_config = config;
    selection_config.deadlock_confirm_time = 0.20;
    VehicleAgent b_retreat = retreat;
    b_retreat.path_s = 1.40;
    b_retreat.slot_departure_clear_s = 0.30;
    std::vector<VehicleAgent> selection_vehicles{b_retreat, passer};
    DeadlockManager selection_manager(map, selection_config);
    selection_manager.update(selection_vehicles, {geometry}, 0.10, false);
    selection_manager.update(selection_vehicles, {geometry}, 0.10, false);
    if (selection_manager.directive().phase != RecoveryPhase::RETREAT ||
        selection_manager.directive().retreat_target_s != 0.0 ||
        selection_manager.directive().retreat_attempt != 1 ||
        selection_manager.directive().nlpTakeover()) {
        return fail("B retreat within 1.5 m did not target path start");
    }
    const int original_path_gen = selection_vehicles[0].path_gen;
    selection_vehicles[0].path_s = 0.0;
    selection_manager.update(selection_vehicles, {geometry}, 0.10, false);
    if (selection_manager.directive().phase != RecoveryPhase::RETREAT_HOLD ||
        selection_vehicles[0].mode != VehicleMode::ACTIVE ||
        selection_vehicles[0].path_gen != original_path_gen) {
        return fail("return to B start changed task identity or skipped HOLD");
    }
    for (int i = 0; i < 20; ++i) {
        selection_manager.update(selection_vehicles, {geometry}, 0.10, false);
    }
    selection_vehicles[1].action = VehicleAction::NOMINAL;
    for (int i = 0; i < 40; ++i) {
        selection_manager.update(selection_vehicles, {geometry}, 0.10, false);
    }
    if (selection_manager.directive().phase != RecoveryPhase::NONE ||
        selection_manager.directive().nlpTakeover()) {
        return fail("successful fixed retreat incorrectly triggered NLP preparation");
    }

    MultiVehicleConfig special_pass_config = config;
    special_pass_config.deadlock_confirm_time = 0.20;
    VehicleAgent held_at_b = retreat;
    held_at_b.path_s = 0.0;
    VehicleAgent stopped_passer = passer;
    stopped_passer.path_s = 0.0;
    stopped_passer.blocker_id = 2;
    VehicleAgent third_party = retreat;
    third_party.id = 2;
    third_party.blocker_id = -1;
    third_party.track.set(RoughPath{
        RoughWp{0.0, 10.0, 0.0, WpType::FORWARD},
        RoughWp{4.0, 10.0, 0.0, WpType::FORWARD}});
    std::vector<VehicleAgent> special_pass_vehicles{
        held_at_b, stopped_passer, third_party};
    DeadlockManager::Snapshot special_pass_state;
    special_pass_state.transaction.phase = RecoveryPhase::PASS;
    special_pass_state.transaction.retreat_vehicle_id = held_at_b.id;
    special_pass_state.transaction.pass_vehicle_id = stopped_passer.id;
    special_pass_state.transaction.retreat_path_gen = held_at_b.path_gen;
    special_pass_state.transaction.pass_path_gen = stopped_passer.path_gen;
    special_pass_state.transaction.pass_conflict_exit_s = 0.50;
    special_pass_state.transaction.hold_retreat_at_b_start = true;
    DeadlockManager special_pass_manager(map, special_pass_config);
    special_pass_manager.restore(special_pass_state);
    for (int i = 0; i < 50; ++i) {
        special_pass_manager.update(special_pass_vehicles, {}, 0.10, false);
    }
    if (special_pass_manager.directive().phase != RecoveryPhase::PASS ||
        special_pass_manager.directive().motionFor(held_at_b.id) !=
            RecoveryMotion::HOLD ||
        special_pass_manager.directive().nlpTakeover()) {
        return fail("third-party STOP incorrectly ended B-start special PASS");
    }

    special_pass_vehicles[1].path_s = 4.0;
    special_pass_manager.update(special_pass_vehicles, {}, 0.10, false);
    if (special_pass_manager.directive().phase != RecoveryPhase::NONE) {
        return fail("passer clearing the original conflict did not end special PASS");
    }

    special_pass_vehicles[1].path_s = 0.0;
    special_pass_vehicles[1].blocker_id = third_party.id;
    special_pass_vehicles[2].blocker_id = stopped_passer.id;
    DeadlockManager persistent_cycle_manager(map, special_pass_config);
    persistent_cycle_manager.restore(special_pass_state);
    persistent_cycle_manager.update(special_pass_vehicles, {}, 0.10, false);
    if (persistent_cycle_manager.directive().nlpTakeover()) {
        return fail("special PASS wait cycle skipped confirmation time");
    }
    persistent_cycle_manager.update(special_pass_vehicles, {}, 0.10, false);
    if (!persistent_cycle_manager.directive().nlpTakeover() ||
        persistent_cycle_manager.directive().deadlock_cycle_ids !=
            std::vector<int>({1, 2})) {
        return fail("persistent wait cycle during special PASS did not enter NLP");
    }

    VehicleAgent non_b_retreat = retreat;
    non_b_retreat.path_s = 1.40;
    non_b_retreat.slot_departure_clear_s = 0.0;
    std::vector<VehicleAgent> non_b_vehicles{non_b_retreat, passer};
    DeadlockManager non_b_manager(map, selection_config);
    non_b_manager.update(non_b_vehicles, {geometry}, 0.10, false);
    non_b_manager.update(non_b_vehicles, {geometry}, 0.10, false);
    if (std::abs(non_b_manager.directive().retreat_target_s - 0.90) > 1e-9) {
        return fail("non-B path incorrectly used direct return retreat");
    }

    VehicleAgent far_b_retreat = b_retreat;
    far_b_retreat.path_s = 1.60;
    std::vector<VehicleAgent> far_b_vehicles{far_b_retreat, passer};
    DeadlockManager far_b_manager(map, selection_config);
    far_b_manager.update(far_b_vehicles, {geometry}, 0.10, false);
    far_b_manager.update(far_b_vehicles, {geometry}, 0.10, false);
    if (std::abs(far_b_manager.directive().retreat_target_s - 1.10) > 1e-9) {
        return fail("B retreat beyond 1.5 m incorrectly targeted path start");
    }

    DeadlockManager manager(map, config);
    DeadlockManager::Snapshot state;
    state.transaction.phase = RecoveryPhase::RETREAT;
    state.transaction.retreat_attempt = 1;
    state.transaction.retreat_vehicle_id = retreat.id;
    state.transaction.pass_vehicle_id = passer.id;
    state.transaction.retreat_path_gen = retreat.path_gen;
    state.transaction.pass_path_gen = passer.path_gen;
    state.transaction.retreat_target_s = retreat.path_s;
    state.transaction.retreat_distance = config.deadlock_retreat_distance;
    manager.restore(state);

    manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::RETREAT_HOLD ||
        manager.directive().motionFor(retreat.id) != RecoveryMotion::HOLD ||
        manager.directive().motionFor(passer.id) != RecoveryMotion::NORMAL) {
        return fail("RETREAT completion did not hold only the retreat vehicle");
    }

    for (int i = 0; i < 19; ++i) manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::RETREAT_HOLD) {
        return fail("RETREAT_HOLD ended before 2.0 seconds");
    }
    manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::PASS ||
        manager.directive().motionFor(retreat.id) != RecoveryMotion::NORMAL ||
        manager.snapshot().transaction.retreat_clear_elapsed != 0.0 ||
        manager.snapshot().transaction.pass_clear_elapsed != 0.0) {
        return fail("RETREAT_HOLD did not enter a fresh PASS observation");
    }

    manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::RETREAT ||
        manager.directive().retreat_attempt != 2) {
        return fail("failed PASS did not start the second retreat");
    }
    vehicles[0].path_s = manager.directive().retreat_target_s;
    manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::RETREAT_HOLD) {
        return fail("second retreat did not enter RETREAT_HOLD");
    }
    for (int i = 0; i < 20; ++i) manager.update(vehicles, {}, 0.10, false);
    manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::RETREAT ||
        manager.directive().retreat_attempt != 3) {
        return fail("failed PASS did not start the third retreat");
    }
    vehicles[0].path_s = manager.directive().retreat_target_s;
    manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::RETREAT_HOLD) {
        return fail("third retreat did not enter RETREAT_HOLD");
    }
    for (int i = 0; i < 20; ++i) manager.update(vehicles, {}, 0.10, false);
    manager.update(vehicles, {}, 0.10, false);
    if (manager.directive().phase != RecoveryPhase::NLP_PREP ||
        manager.directive().nlp_vehicle_ids != std::vector<int>({0, 1})) {
        return fail("third failed PASS did not enter two-vehicle NLP preparation");
    }

    VehicleAgent upstream = retreat;
    upstream.id = 0;
    upstream.blocker_id = 1;
    VehicleAgent pair_first = retreat;
    pair_first.id = 1;
    pair_first.blocker_id = 2;
    VehicleAgent pair_second = passer;
    pair_second.id = 2;
    pair_second.blocker_id = 1;
    std::vector<VehicleAgent> associated_vehicles{
        upstream, pair_first, pair_second};
    DeadlockManager::Snapshot associated_state;
    associated_state.transaction.phase = RecoveryPhase::PASS;
    associated_state.transaction.retreat_attempt =
        config.deadlock_retreat_max_attempts;
    associated_state.transaction.retreat_vehicle_id = pair_first.id;
    associated_state.transaction.pass_vehicle_id = pair_second.id;
    associated_state.transaction.retreat_path_gen = pair_first.path_gen;
    associated_state.transaction.pass_path_gen = pair_second.path_gen;
    DeadlockManager associated_manager(map, config);
    associated_manager.restore(associated_state);
    associated_manager.update(associated_vehicles, {}, 0.10, false);
    if (associated_manager.directive().phase != RecoveryPhase::NLP_PREP ||
        associated_manager.directive().nlp_vehicle_ids !=
            std::vector<int>({0, 1, 2})) {
        return fail("upstream stopped blocker was omitted from NLP participants");
    }
    if (associated_manager.directive().deadlock_cycle_ids !=
            std::vector<int>({1, 2}) ||
        associated_manager.directive().frozen_vehicle_ids !=
            std::vector<int>({0, 1, 2})) {
        return fail("original cycle and expanded frozen set were not separated");
    }

    associated_vehicles[0].mission_phase = MissionPhase::TO_A1;
    associated_vehicles[0].path_s = 0.0;
    VehicleAgent parked_at_b = upstream;
    parked_at_b.id = 3;
    parked_at_b.mission_phase = MissionPhase::TO_B;
    parked_at_b.path_s = parked_at_b.track.length();
    VehicleAgent unloading = upstream;
    unloading.id = 4;
    unloading.mission_phase = MissionPhase::UNLOAD_DWELL;
    associated_vehicles.push_back(parked_at_b);
    associated_vehicles.push_back(unloading);
    DeadlockManager parked_manager(map, config);
    parked_manager.restore(associated_state);
    parked_manager.update(associated_vehicles, {}, 0.10, false);
    if (parked_manager.directive().nlp_vehicle_ids !=
        std::vector<int>({0, 1, 2, 3, 4})) {
        return fail("parked or not-yet-departed waiters were omitted from frozen obstacles");
    }

    DeadlockManager ordinary_stop_manager(map, config);
    vehicles[0].blocker_id = -1;
    vehicles[1].blocker_id = -1;
    for (int i = 0; i < 30; ++i) {
        ordinary_stop_manager.update(vehicles, {}, 0.10, false);
    }
    if (ordinary_stop_manager.directive().phase != RecoveryPhase::NONE) {
        return fail("ordinary STOP incorrectly entered RETREAT_HOLD");
    }

    MultiVehicleConfig cycle_config = config;
    cycle_config.deadlock_confirm_time = 0.20;
    VehicleAgent cycle_a = retreat;
    VehicleAgent cycle_b = passer;
    VehicleAgent cycle_c = retreat;
    cycle_a.id = 2;
    cycle_a.blocker_id = 5;
    cycle_a.path_gen = 12;
    cycle_b.id = 5;
    cycle_b.blocker_id = 9;
    cycle_b.path_gen = 15;
    cycle_c.id = 9;
    cycle_c.blocker_id = 2;
    cycle_c.path_gen = 19;
    cycle_c.track.set(RoughPath{
        RoughWp{0.0, 10.0, 0.0, WpType::FORWARD},
        RoughWp{4.0, 10.0, 0.0, WpType::FORWARD}});
    std::vector<VehicleAgent> three_cycle{cycle_c, cycle_a, cycle_b};
    DeadlockManager three_cycle_manager(map, cycle_config);
    three_cycle_manager.update(three_cycle, {}, 0.10, false);
    if (three_cycle_manager.directive().nlpTakeover()) {
        return fail("three-vehicle cycle skipped confirmation time");
    }
    three_cycle_manager.update(three_cycle, {}, 0.10, false);
    if (!three_cycle_manager.directive().nlpTakeover() ||
        three_cycle_manager.directive().nlp_vehicle_ids !=
            std::vector<int>({2, 5, 9})) {
        return fail("non-contiguous three-vehicle cycle did not enter NLP preparation");
    }
    if (three_cycle_manager.directive().deadlock_cycle_ids !=
            std::vector<int>({2, 5, 9})) {
        return fail("three-vehicle original cycle was not retained");
    }
    RuleEngine three_cycle_engine(map, cycle_config);
    three_cycle_engine.observeDeadlock(three_cycle, 0.10, false);
    three_cycle_engine.observeDeadlock(three_cycle, 0.10, false);
    if (!three_cycle_engine.recoveryDirective().nlpTakeover() ||
        three_cycle_engine.recoveryDirective().nlp_vehicle_ids !=
            std::vector<int>({2, 5, 9})) {
        return fail("RuleEngine did not forward the three-vehicle wait graph");
    }
    three_cycle[0].action = VehicleAction::NOMINAL;
    three_cycle_manager.update(three_cycle, {}, 10.0, false);
    if (!three_cycle_manager.directive().nlpTakeover() ||
        three_cycle_manager.directive().nlp_vehicle_ids !=
            std::vector<int>({2, 5, 9})) {
        return fail("NLP preparation did not remain latched");
    }

    DeadlockManager moving_candidate_manager(map, cycle_config);
    std::vector<VehicleAgent> moving_cycle{cycle_a, cycle_b, cycle_c};
    moving_candidate_manager.update(moving_cycle, {}, 0.10, false);
    moving_cycle[0].path_s += 0.10;
    moving_candidate_manager.update(moving_cycle, {}, 0.10, false);
    if (moving_candidate_manager.directive().nlpTakeover()) {
        return fail("progressing three-vehicle cycle bypassed candidate continuity");
    }

    DeadlockManager open_chain_manager(map, cycle_config);
    std::vector<VehicleAgent> open_chain{cycle_a, cycle_b, cycle_c};
    open_chain[2].blocker_id = -1;
    for (int i = 0; i < 30; ++i) {
        open_chain_manager.update(open_chain, {}, 0.10, false);
    }
    if (open_chain_manager.directive().phase != RecoveryPhase::NONE) {
        return fail("open blocker chain was misclassified as a deadlock cycle");
    }

    VehicleAgent measured_hold = retreat;
    measured_hold.real_pose_valid = true;
    measured_hold.real_x = 10.0;
    measured_hold.real_y = -3.0;
    measured_hold.real_yaw = 0.7;
    const auto stationary = predictStationaryTrajectory(
        measured_hold, map, config, config.prediction_horizon);
    if (stationary.empty() ||
        std::abs(stationary.back().t - 15.0) > 1e-9) {
        return fail("stationary HOLD prediction did not cover 15 seconds");
    }
    for (const PredictedKinematicSample& sample : stationary) {
        if (sample.speed != 0.0 || sample.s != measured_hold.path_s ||
            std::abs(sample.body.x - stationary.front().body.x) > 1e-9 ||
            std::abs(sample.body.y - stationary.front().body.y) > 1e-9 ||
            std::abs(sample.body.theta - stationary.front().body.theta) > 1e-9) {
            return fail("stationary HOLD prediction changed pose or speed");
        }
    }

    RuleEngine engine(map, config);
    RuleEngine::SimSnapshot engine_state = engine.snapshot();
    engine_state.deadlock.transaction.phase = RecoveryPhase::RETREAT_HOLD;
    engine_state.deadlock.transaction.retreat_attempt = 1;
    engine_state.deadlock.transaction.retreat_vehicle_id = retreat.id;
    engine_state.deadlock.transaction.pass_vehicle_id = passer.id;
    engine_state.deadlock.transaction.retreat_path_gen = retreat.path_gen;
    engine_state.deadlock.transaction.pass_path_gen = passer.path_gen;
    engine_state.deadlock.transaction.retreat_target_s = retreat.path_s;
    engine_state.deadlock.directive.phase = RecoveryPhase::RETREAT_HOLD;
    engine_state.deadlock.directive.retreat_attempt = 1;
    engine_state.deadlock.directive.retreat_vehicle_id = retreat.id;
    engine_state.deadlock.directive.pass_vehicle_id = passer.id;
    engine_state.deadlock.directive.retreat_path_gen = retreat.path_gen;
    engine_state.deadlock.directive.pass_path_gen = passer.path_gen;
    engine_state.deadlock.directive.retreat_target_s = retreat.path_s;
    engine.restore(engine_state);

    vehicles[0].action = VehicleAction::NOMINAL;
    vehicles[0].requested_action = VehicleAction::NOMINAL;
    vehicles[0].current_speed = config.nominal_speed;
    vehicles[0].path_s = retreat.path_s;
    vehicles[1].action = VehicleAction::NOMINAL;
    vehicles[1].requested_action = VehicleAction::NOMINAL;
    vehicles[1].current_speed = config.nominal_speed;
    engine.decide(vehicles, 0.10);
    engine.applyRecoveryDirectiveToOutput(vehicles);
    if (vehicles[0].action != VehicleAction::STOP ||
        vehicles[0].requested_action != VehicleAction::STOP ||
        vehicles[0].current_speed != 0.0 ||
        vehicles[1].action == VehicleAction::STOP ||
        engine.motionOverrideFor(vehicles[0].id).motion !=
            RecoveryMotion::HOLD ||
        engine.motionOverrideFor(vehicles[1].id).motion !=
            RecoveryMotion::NORMAL) {
        return fail("RETREAT_HOLD output did not stop only the retreat vehicle");
    }

    const double elapsed_before_freeze =
        engine.snapshot().deadlock.transaction.retreat_hold_elapsed;
    engine.setRolloutRecoveryHoldFrozen(true);
    for (int i = 0; i < 150; ++i) engine.decide(vehicles, 0.10);
    engine.setRolloutRecoveryHoldFrozen(false);
    const auto frozen_state = engine.snapshot().deadlock;
    if (frozen_state.directive.phase != RecoveryPhase::RETREAT_HOLD ||
        std::abs(frozen_state.transaction.retreat_hold_elapsed -
                 elapsed_before_freeze) > 1e-9) {
        return fail("15-second rollout advanced live RETREAT_HOLD time");
    }

    DeadlockManager nlp_manager(map, config);
    DeadlockManager::Snapshot nlp_state;
    nlp_state.transaction.phase = RecoveryPhase::NLP_PREP;
    nlp_state.transaction.recovery_id = 42;
    nlp_state.transaction.nlp_vehicle_ids = {retreat.id, passer.id};
    nlp_state.directive.phase = RecoveryPhase::NLP_PREP;
    nlp_state.directive.recovery_id = 42;
    nlp_state.directive.nlp_vehicle_ids = {retreat.id, passer.id};
    nlp_manager.restore(nlp_state);
    if (nlp_manager.completeNlpRecovery(41) ||
        nlp_manager.directive().phase != RecoveryPhase::NLP_PREP) {
        return fail("mismatched NLP recovery identity cleared the transaction");
    }
    if (!nlp_manager.completeNlpRecovery(42) ||
        nlp_manager.directive().phase != RecoveryPhase::NONE ||
        nlp_manager.directive().recovery_id != 0) {
        return fail("valid NLP recovery completion did not clear the transaction");
    }

    std::cout << "deadlock_manager_test: PASS\n";
    return 0;
}
