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
    config.rolling_refresh_period = 0.10;

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
    if (manager.directive().phase != RecoveryPhase::UNRESOLVED) {
        return fail("third failed PASS did not stop at the retry limit");
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

    std::cout << "deadlock_manager_test: PASS\n";
    return 0;
}
