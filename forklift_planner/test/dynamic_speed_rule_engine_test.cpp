#include "forklift_planner/multi_vehicle/rule_engine.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <ros/time.h>

using namespace forklift_planner::multi_vehicle;

namespace {

int fail(const std::string& message) {
    std::cerr << "dynamic_speed_rule_engine_test: " << message << '\n';
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
    result.mission_phase = MissionPhase::TO_A1;
    result.path_gen = 1;
    result.current_speed = speed;
    result.track.set(vertical
        ? RoughPath{wp(0.0, -approach, 1.5707963267948966),
                    wp(0.0, 2.0, 1.5707963267948966)}
        : RoughPath{wp(-approach, 0.0, 0.0),
                    wp(2.0, 0.0, 0.0)});
    return result;
}

VehicleAgent diagonalVehicle(int id, double approach, double speed = 0.0) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = 1;
    result.current_speed = speed;
    constexpr double kQuarterPi = 0.7853981633974483;
    result.track.set(RoughPath{
        wp(-approach, -approach, kQuarterPi),
        wp(2.0, 2.0, kQuarterPi)});
    return result;
}

VehicleAgent laneVehicle(int id, double path_s, double speed) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = 1;
    result.path_s = path_s;
    result.current_speed = speed;
    result.track.set(RoughPath{wp(0.0, 0.0, 0.0),
                               wp(4.0, 0.0, 0.0)});
    return result;
}

bool hasDynamicReason(const std::vector<VehicleAgent>& vehicles,
                      VehicleAction action) {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.requested_action == action &&
            vehicle.reason.rfind("dynamic_speed_", 0) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    ros::Time::init();
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_horizon = 15.0;
    config.prediction_step = 0.05;

    // Ordinary priority is stable across rolling periods: diagnostic waiting
    // time cannot reverse it. Safety/resource prerequisites remain explicit
    // overrides ahead of the loaded/task-count/id total order.
    RuleEngine priority_engine(map_param, config);
    VehicleAgent priority_a = crossingVehicle(0, 2.0, false);
    VehicleAgent priority_b = crossingVehicle(1, 2.0, true);
    priority_a.current_slot = 10;
    priority_a.target_slot = 11;
    priority_b.current_slot = 20;
    priority_b.target_slot = 21;
    priority_b.wait_time = 1000.0;
    if (priority_engine.priorityWinner(priority_a, priority_b) != priority_a.id) {
        return fail("wait_time reversed ordinary stable priority");
    }
    priority_b.loaded = true;
    if (priority_engine.priorityWinner(priority_a, priority_b) != priority_b.id) {
        return fail("loaded vehicle lost ordinary priority");
    }
    priority_b.loaded = false;
    priority_a.task_count = 4;
    priority_b.task_count = 1;
    if (priority_engine.priorityWinner(priority_a, priority_b) != priority_b.id) {
        return fail("task-count priority order changed");
    }
    priority_a.task_count = 0;
    priority_b.task_count = 0;
    priority_a.target_slot = priority_b.current_slot;
    if (priority_engine.priorityWinner(priority_a, priority_b) != priority_b.id) {
        return fail("slot dependency prerequisite lost precedence");
    }
    priority_a.target_slot = 11;

    // Initial Future-A1 competition includes a vehicle whose next service is
    // known while it is still completing TO_B. Its commitment is keyed to the
    // next pickup path generation and remains locked through unload and the
    // subsequent TO_A1 activation.
    MultiVehicleConfig future_config = config;
    future_config.unload_dwell_time = 2.0;
    future_config.pickup_dwell_time = 1.0;
    RuleEngine future_engine(map_param, future_config);
    VehicleAgent future_to_b = laneVehicle(30, 1.0, 1.0);
    future_to_b.track.set(
        RoughPath{wp(0.0, 0.0, 0.0), wp(2.0, 0.0, 0.0)});
    future_to_b.mission_phase = MissionPhase::TO_B;
    future_to_b.leg_target = LegTargetKind::B_SLOT;
    future_to_b.target_slot = 12;
    future_to_b.path_gen = 7;
    future_to_b.loaded = true;
    VehicleAgent later_to_a1 = laneVehicle(31, 0.0, 0.0);
    later_to_a1.track.set(
        RoughPath{wp(0.0, 1.0, 0.0), wp(5.0, 1.0, 0.0)});
    later_to_a1.mission_phase = MissionPhase::TO_A1;
    later_to_a1.leg_target = LegTargetKind::A1;
    later_to_a1.pending_dropoff_valid = true;
    later_to_a1.pending_dropoff_track.set(
        RoughPath{wp(0.0, 2.0, 0.0), wp(1.0, 2.0, 0.0)});
    std::vector<VehicleAgent> future_vehicles{future_to_b, later_to_a1};
    RuleEngine::A1ArrivalKinematics future_kinematics;
    future_kinematics.dt = 0.1;
    future_kinematics.desired_speed =
        [](const VehicleAgent&) { return 1.0; };
    future_kinematics.limited_speed =
        [](double, double desired, double) { return desired; };
    future_kinematics.pickup_leg_track = [](int slot, PathTrack& out) {
        if (slot != 12) return false;
        out.set(RoughPath{wp(0.0, 3.0, 0.0), wp(1.0, 3.0, 0.0)});
        return true;
    };
    future_engine.refreshA1PlanningContext(
        future_vehicles, 10.0, 0.0, future_kinematics);
    const auto first_future_owner = future_engine.futureA1Commitment();
    if (first_future_owner.owner_id != 30 ||
        first_future_owner.owner_path_gen != 8 ||
        std::abs(first_future_owner.predicted_a1_arrival_time - 4.0) > 1e-9) {
        return fail("TO_B next-A1 ETA or future path generation is incorrect");
    }
    future_vehicles[0].mode = VehicleMode::DWELL;
    future_vehicles[0].mission_phase = MissionPhase::UNLOAD_DWELL;
    future_vehicles[0].current_slot = 12;
    future_vehicles[0].dwell_remaining = 1.5;
    future_engine.refreshA1PlanningContext(
        future_vehicles, 10.0, 0.1, future_kinematics);
    if (future_engine.futureA1Commitment().owner_id != 30 ||
        future_engine.futureA1Commitment().owner_path_gen != 8) {
        return fail("next-A1 owner was not locked through UNLOAD_DWELL");
    }
    future_vehicles[0].mode = VehicleMode::ACTIVE;
    future_vehicles[0].mission_phase = MissionPhase::TO_A1;
    future_vehicles[0].leg_target = LegTargetKind::A1;
    future_vehicles[0].path_gen = 8;
    future_vehicles[0].path_s = 0.0;
    future_vehicles[0].current_speed = 0.0;
    future_vehicles[0].track.set(
        RoughPath{wp(0.0, 3.0, 0.0), wp(1.0, 3.0, 0.0)});
    future_vehicles[0].pending_dropoff_valid = true;
    future_vehicles[0].pending_dropoff_track.set(
        RoughPath{wp(0.0, 4.0, 0.0), wp(1.0, 4.0, 0.0)});
    future_engine.refreshA1PlanningContext(
        future_vehicles, 10.0, 0.2, future_kinematics);
    if (future_engine.futureA1Commitment().owner_id != 30 ||
        future_engine.futureA1Commitment().owner_path_gen != 8) {
        return fail("next-A1 owner did not hand off to the pickup leg");
    }

    // Slot departure admission gives an already ACTIVE road vehicle priority
    // only when synchronized OBB overlap occurs before the candidate clears
    // its source-slot prefix. A later conflict remains rolling-coordinator
    // work and must not hold the parked vehicle.
    RuleEngine admission_engine(map_param, config);
    VehicleAgent launch_candidate = crossingVehicle(20, 0.0, false);
    launch_candidate.current_slot = 20;
    launch_candidate.track.set(RoughPath{
        wp(0.0, 0.0, 0.0), wp(2.0, 0.0, 0.0)});
    launch_candidate.slot_departure_clear_s = 0.5;
    VehicleAgent immediate_occupant = crossingVehicle(21, 0.0, true);
    immediate_occupant.track.set(RoughPath{
        wp(0.0, 0.0, 1.5707963267948966),
        wp(0.0, 2.0, 1.5707963267948966)});
    const auto immediate_admission =
        admission_engine.checkSlotDepartureAdmission(
            nullptr, launch_candidate, {immediate_occupant}, 15.0);
    if (immediate_admission.clear ||
        !immediate_admission.ordinary_road_conflict ||
        immediate_admission.blocker_id != immediate_occupant.id ||
        immediate_admission.candidate_conflict_s >
            launch_candidate.slot_departure_clear_s + 1e-9) {
        return fail("immediate slot departure conflict was not held");
    }

    VehicleAgent later_occupant = crossingVehicle(22, 0.0, true);
    later_occupant.track.set(RoughPath{
        wp(1.5, -1.5, 1.5707963267948966),
        wp(1.5, 2.0, 1.5707963267948966)});
    const auto later_admission =
        admission_engine.checkSlotDepartureAdmission(
            nullptr, launch_candidate, {later_occupant}, 15.0);
    if (!later_admission.clear ||
        later_admission.ordinary_road_conflict) {
        return fail("post-slot future conflict over-held departure");
    }

    VehicleAgent service_owner = crossingVehicle(23, 1.0, true);
    service_owner.pending_dropoff_valid = true;
    service_owner.pending_dropoff_track = later_occupant.track;
    service_owner.a1_departure_priority_until_s =
        service_owner.pending_dropoff_track.length();
    const auto far_a1 = admission_engine.checkA1LaunchAdmission(
        service_owner, launch_candidate);
    if (far_a1.departure_resource_conflict) {
        return fail("far A1 departure closure over-held slot launch");
    }
    VehicleAgent b0_b9_candidate = launch_candidate;
    b0_b9_candidate.current_slot = 6;
    const auto source_slot_a1 = admission_engine.checkA1LaunchAdmission(
        service_owner, b0_b9_candidate);
    if (!source_slot_a1.departure_resource_conflict ||
        !source_slot_a1.source_slot_hold ||
        source_slot_a1.protected_zone_count == 0) {
        return fail("B0-B9 A1 departure resource did not hold at source");
    }
    service_owner.pending_dropoff_track = immediate_occupant.track;
    service_owner.a1_departure_priority_until_s =
        service_owner.pending_dropoff_track.length();
    const auto near_a1 = admission_engine.checkA1LaunchAdmission(
        service_owner, launch_candidate);
    if (!near_a1.departure_resource_conflict) {
        return fail("immediate A1 departure prefix conflict was not held");
    }

    RuleEngine far_engine(map_param, config);
    std::vector<VehicleAgent> far{
        crossingVehicle(0, 2.50, false),
        crossingVehicle(1, 2.90, true)};
    far[1].path_s = 0.40;
    far_engine.decide(far, 0.1, 15.0);
    if (far_engine.dynamicSpeedMetrics().far_decisions == 0 ||
        far[0].requested_action != VehicleAction::NOMINAL ||
        far[1].requested_action != VehicleAction::NOMINAL ||
        !far_engine.snapshot().reservations.empty()) {
        return fail("FAR did not remain reservation-free NOMINAL");
    }

    RuleEngine mid_engine(map_param, config);
    std::vector<VehicleAgent> mid{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    mid[1].path_s = 0.40;
    mid_engine.decide(mid, 0.1, 15.0);
    if (mid_engine.dynamicSpeedMetrics().mid_decisions == 0 ||
        !hasDynamicReason(mid, VehicleAction::YIELD) ||
        !mid_engine.snapshot().reservations.empty()) {
        return fail("MID did not accept one reservation-free YIELD");
    }

    RuleEngine near_engine(map_param, config);
    std::vector<VehicleAgent> near{
        crossingVehicle(0, 0.80, false),
        crossingVehicle(1, 1.05, true)};
    near[1].path_s = 0.25;
    near_engine.decide(near, 0.1, 15.0);
    if (near_engine.dynamicSpeedMetrics().near_decisions == 0 ||
        !hasDynamicReason(near, VehicleAction::CREEP) ||
        !near_engine.snapshot().reservations.empty()) {
        return fail("NEAR did not jump directly to reservation-free CREEP");
    }

    // Braking safety remains a direct STOP motion action, but no longer
    // creates ordinary-road holder/waiter ownership.
    RuleEngine emergency_engine(map_param, config);
    std::vector<VehicleAgent> emergency{
        crossingVehicle(0, 0.30, false, config.nominal_speed),
        crossingVehicle(1, 0.30, true, config.nominal_speed)};
    emergency_engine.decide(emergency, 0.1, 15.0);
    const auto emergency_state = emergency_engine.snapshot();
    if (emergency_engine.dynamicSpeedMetrics().emergency_stop_decisions == 0 ||
        !hasDynamicReason(emergency, VehicleAction::STOP) ||
        !emergency_state.reservations.empty()) {
        return fail("braking emergency did not remain reservation-free STOP");
    }
    int held_index = -1;
    for (size_t index = 0; index < emergency.size(); ++index) {
        if (emergency[index].ttc_stop_hold_remaining > 1e-9) {
            held_index = static_cast<int>(index);
            break;
        }
    }
    if (held_index < 0) {
        return fail("TTC emergency did not arm the rolling STOP hold");
    }
    const auto conflicts_before_hold =
        emergency_engine.dynamicSpeedMetrics().baseline_conflicts;
    for (int frame = 1; frame < 20; ++frame) {
        emergency_engine.decide(emergency, 0.1, 15.0);
        if (emergency[held_index].requested_action != VehicleAction::STOP) {
            return fail("TTC STOP replanned before the rolling period ended");
        }
    }
    if (emergency[held_index].ttc_stop_hold_remaining > 1e-8) {
        return fail("repeated pair checks continuously re-armed TTC STOP");
    }
    if (emergency_engine.dynamicSpeedMetrics().baseline_conflicts <=
        conflicts_before_hold) {
        return fail("held STOP vehicle disappeared from pair prediction");
    }

    // A real head-on corridor conflict is corrected to each vehicle's local
    // near boundary before the unchanged priority policy selects the yielding
    // side. No bridge ownership or reservation is created.
    RuleEngine bridge_engine(map_param, config);
    std::vector<std::string> bridge_logs;
    bridge_engine.setCoordLogSink(
        [&](const std::string& line) { bridge_logs.push_back(line); });
    VehicleAgent bridge_a = crossingVehicle(0, 0.0, false);
    VehicleAgent bridge_b = crossingVehicle(1, 0.0, false);
    bridge_a.track.set(RoughPath{wp(0.0, 0.0, 0.0),
                                 wp(4.0, 0.0, 0.0)});
    bridge_b.track.set(RoughPath{wp(4.0, 0.04, 3.14159265358979323846),
                                 wp(0.0, 0.04, 3.14159265358979323846)});
    std::vector<VehicleAgent> bridge_pair{bridge_a, bridge_b};
    bridge_engine.decide(bridge_pair, 0.1, 15.0);
    const auto& bridge_metrics = bridge_engine.dynamicSpeedMetrics();
    bool saw_bridge_log = false;
    bool saw_vehicle_ttc_log = false;
    for (const std::string& line : bridge_logs) {
        saw_bridge_log = saw_bridge_log ||
            (line.find("[BRIDGE-TTC]") != std::string::npos &&
             line.find("bridge_a=true") != std::string::npos &&
             line.find("bridge_b=true") != std::string::npos &&
             line.find("V0_original_ttc=") != std::string::npos &&
             line.find("V1_original_ttc=") != std::string::npos &&
             line.find("V0_corrected_ttc=0.000") != std::string::npos);
        saw_vehicle_ttc_log = saw_vehicle_ttc_log ||
            (line.find("[DYN-TTC]") != std::string::npos &&
             line.find("first_overlap_t=") != std::string::npos &&
             line.find("collision_s_a=") != std::string::npos &&
             line.find("collision_s_b=") != std::string::npos &&
             line.find("original_ttc_a=") != std::string::npos &&
             line.find("original_ttc_b=") != std::string::npos &&
             line.find("effective_ttc_a=") != std::string::npos &&
             line.find("effective_ttc_b=") != std::string::npos &&
             line.find("priority_physical_ttc=") != std::string::npos &&
             line.find("yield_effective_ttc=") != std::string::npos);
    }
    if (bridge_metrics.bridge_checked_pairs != 1 ||
        bridge_metrics.opposing_conflicts != 1 ||
        bridge_metrics.bridge_corrected_pairs != 1 ||
        bridge_metrics.bridge_nearest_evaluations == 0 ||
        !saw_bridge_log || !saw_vehicle_ttc_log ||
        !hasDynamicReason(bridge_pair, VehicleAction::STOP) ||
        !bridge_engine.snapshot().reservations.empty()) {
        return fail("head-on bridge TTC did not drive reservation-free action");
    }

    // Entering the corrected near-boundary first is diagnostic only. It must
    // not turn the ordinary yielding vehicle into an effective winner.
    RuleEngine entered_bridge_engine(map_param, config);
    std::vector<std::string> entered_bridge_logs;
    entered_bridge_engine.setCoordLogSink(
        [&](const std::string& line) { entered_bridge_logs.push_back(line); });
    VehicleAgent bridge_priority = crossingVehicle(0, 0.0, false);
    bridge_priority.track.set(RoughPath{
        wp(0.0, -1.0, 1.5707963267948966),
        wp(0.0, 0.0, 0.0), wp(4.0, 0.0, 0.0)});
    VehicleAgent bridge_yielding = crossingVehicle(1, 0.0, false);
    bridge_yielding.track.set(RoughPath{
        wp(4.0, 0.04, 3.14159265358979323846),
        wp(0.0, 0.04, 3.14159265358979323846),
        wp(0.0, 1.0, 1.5707963267948966)});
    bridge_yielding.path_s = 1.0;
    std::vector<VehicleAgent> entered_bridge_pair{
        bridge_priority, bridge_yielding};
    entered_bridge_engine.decide(entered_bridge_pair, 0.1, 15.0);
    bool kept_ordinary_bridge_priority = false;
    for (const std::string& line : entered_bridge_logs) {
        kept_ordinary_bridge_priority = kept_ordinary_bridge_priority ||
            (line.find("[BRIDGE-TTC]") != std::string::npos &&
             line.find("priority_vehicle=V0") != std::string::npos &&
             line.find("yielding_vehicle=V1") != std::string::npos);
    }
    if (!kept_ordinary_bridge_priority) {
        return fail("Bridge near-boundary entry reversed ordinary priority");
    }

    // A nominally clear pair does not even enter bridge matching and emits no
    // BRIDGE-TTC record.
    RuleEngine clear_engine(map_param, config);
    std::vector<std::string> clear_logs;
    clear_engine.setCoordLogSink(
        [&](const std::string& line) { clear_logs.push_back(line); });
    std::vector<VehicleAgent> clear_pair{
        laneVehicle(10, 0.0, 0.0), laneVehicle(11, 0.0, 0.0)};
    clear_pair[1].track.set(
        RoughPath{wp(0.0, 2.0, 0.0), wp(4.0, 2.0, 0.0)});
    clear_engine.decide(clear_pair, 0.1, 15.0);
    for (const std::string& line : clear_logs) {
        if (line.find("[BRIDGE-TTC]") != std::string::npos) {
            return fail("clear baseline emitted bridge log");
        }
    }
    if (clear_engine.dynamicSpeedMetrics().bridge_checked_pairs != 0 ||
        clear_pair[0].requested_action != VehicleAction::NOMINAL ||
        clear_pair[1].requested_action != VehicleAction::NOMINAL ||
        !clear_engine.snapshot().reservations.empty()) {
        return fail("clear baseline activated bridge coordination");
    }

    RuleEngine reserved_engine(map_param, config);
    std::vector<VehicleAgent> reserved{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    reserved[1].path_s = 0.40;
    RuleEngine::SimSnapshot reservation_state;
    RuleEngine::ConflictReservation reservation;
    reservation.owner_id = 0;
    reservation.gen_lo = 1;
    reservation.gen_hi = 1;
    reservation.enter_lo = 0.10;
    reservation.exit_lo = 0.70;
    reservation.enter_hi = 0.40;
    reservation.exit_hi = 1.00;
    reservation.create_reason = "already_inside";
    reservation_state.reservations[{0, 1}] = reservation;
    reserved_engine.restore(reservation_state);
    reserved_engine.decide(reserved, 0.1, 15.0);
    if (!reserved_engine.snapshot().reservations.empty() ||
        reserved_engine.dynamicSpeedMetrics().reservation_deletes == 0 ||
        reserved_engine.dynamicSpeedMetrics().existing_reservation_skips != 0 ||
        reserved_engine.dynamicSpeedMetrics().baseline_conflicts == 0) {
        return fail("ordinary already-inside reservation was not retired");
    }

    // A single vehicle departure flag is not pair-level A1 authority.
    RuleEngine departure_flag_engine(map_param, config);
    std::vector<VehicleAgent> departure_flag{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    departure_flag[1].path_s = 0.40;
    departure_flag[0].mission_phase = MissionPhase::TO_B;
    departure_flag[1].mission_phase = MissionPhase::TO_B;
    departure_flag[0].a1_departure_committed = true;
    departure_flag[0].a1_departure_priority_until_s = 1.0;
    departure_flag_engine.decide(departure_flag, 0.1, 15.0);
    if (!departure_flag_engine.snapshot().reservations.empty() ||
        departure_flag_engine.dynamicSpeedMetrics().baseline_conflicts == 0 ||
        departure_flag_engine.dynamicSpeedMetrics().a1_fallbacks != 0) {
        return fail("single A1 departure flag still captured an ordinary pair");
    }

    // Future owner identity without a real future-exit conflict remains an
    // ordinary crossing and must use rolling dynamic coordination.
    RuleEngine identity_engine(map_param, config);
    std::vector<VehicleAgent> identity{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    identity[1].path_s = 0.40;
    identity[0].pending_dropoff_valid = true;
    identity[0].pending_dropoff_track.set(
        RoughPath{wp(10.0, 10.0, 0.0), wp(12.0, 10.0, 0.0)});
    identity[0].a1_departure_priority_until_s = 1.0;
    RuleEngine::FutureA1Commitment identity_owner;
    identity_owner.owner_id = 0;
    identity_owner.owner_path_gen = 1;
    identity_owner.predicted_a1_arrival_time = 5.0;
    identity_owner.predicted_to_b_time = 10.0;
    identity_engine.setFutureA1Commitment(identity_owner);
    identity_engine.decide(identity, 0.1, 15.0);
    if (!identity_engine.snapshot().reservations.empty() ||
        identity_engine.dynamicSpeedMetrics().mid_decisions == 0 ||
        identity_engine.dynamicSpeedMetrics().a1_fallbacks != 0) {
        return fail("Future A1 owner identity still captured an ordinary pair");
    }

    // An active departure transaction must not capture a current-road event
    // outside its frozen protected intervals. That event remains ordinary
    // synchronized-OBB/bridge TTC and creates no pair reservation.
    RuleEngine active_cluster_engine(map_param, config);
    std::vector<std::string> active_cluster_logs;
    active_cluster_engine.setCoordLogSink(
        [&](const std::string& line) { active_cluster_logs.push_back(line); });
    std::vector<VehicleAgent> active_cluster{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    active_cluster[1].path_s = 0.40;
    active_cluster[0].mission_phase = MissionPhase::TO_B;
    RuleEngine::SimSnapshot active_state;
    RuleEngine::DepartureClusterCommitment active_commitment;
    active_commitment.owner_id = 0;
    active_commitment.owner_path_gen = 1;
    active_commitment.other_id = 1;
    active_commitment.other_path_gen = 1;
    active_commitment.intervals.push_back(
        FutureA1ConflictInterval{2.10, 2.50, 2.40, 2.80});
    active_commitment.waiter_stop_boundary_s = 2.40;
    active_commitment.waiter_stop_s = 2.30;
    active_commitment.owner_release_exit_s = 1.50;
    active_commitment.other_release_exit_s = 2.80;
    active_commitment.active = true;
    active_state.a1.departure_clusters[{0, 1}] = active_commitment;
    active_cluster_engine.restore(active_state);
    active_cluster_engine.decide(active_cluster, 0.1, 15.0);
    bool saw_dynamic_speed = false;
    bool saw_bridge_ttc = false;
    bool saw_a1_skip = false;
    for (const std::string& line : active_cluster_logs) {
        saw_dynamic_speed = saw_dynamic_speed ||
            line.find("[DYN-SPEED]") != std::string::npos;
        saw_bridge_ttc = saw_bridge_ttc ||
            line.find("[BRIDGE-TTC]") != std::string::npos;
        saw_a1_skip = saw_a1_skip ||
            line.find("a1_protected") != std::string::npos ||
            line.find("reservation_reason=a1_related") != std::string::npos;
    }
    if (!active_cluster_engine.snapshot().reservations.empty() ||
        active_cluster_engine.dynamicSpeedMetrics().baseline_conflicts == 0 ||
        active_cluster_engine.dynamicSpeedMetrics().a1_fallbacks != 0 ||
        !saw_dynamic_speed || !saw_bridge_ttc || saw_a1_skip ||
        active_cluster[1].reason == "time_brake_V0") {
        return fail("active A1 pair captured an ordinary current-road event");
    }

    // A nominal overlap beyond the waiter's frozen stop line is not executable:
    // clip only that event and keep both vehicles NOMINAL while the waiter is
    // still outside the physical braking distance of stop_s.
    RuleEngine clipped_engine(map_param, config);
    std::vector<std::string> clipped_logs;
    clipped_engine.setCoordLogSink(
        [&](const std::string& line) { clipped_logs.push_back(line); });
    std::vector<VehicleAgent> clipped{
        crossingVehicle(0, 1.50, false, config.nominal_speed),
        crossingVehicle(1, 1.50, true, config.nominal_speed)};
    clipped[0].mission_phase = MissionPhase::TO_B;
    clipped[1].loaded = true;
    RuleEngine::SimSnapshot clipped_state;
    RuleEngine::DepartureClusterCommitment clipped_commitment;
    clipped_commitment.owner_id = 0;
    clipped_commitment.owner_path_gen = 1;
    clipped_commitment.other_id = 1;
    clipped_commitment.other_path_gen = 1;
    clipped_commitment.intervals.push_back(
        FutureA1ConflictInterval{0.40, 0.80, 0.70, 1.00});
    clipped_commitment.waiter_stop_boundary_s = 0.70;
    clipped_commitment.waiter_stop_s = 0.60;
    clipped_commitment.owner_release_exit_s = 3.00;
    clipped_commitment.other_release_exit_s = 1.00;
    clipped_commitment.active = true;
    clipped_state.a1.departure_clusters[{0, 1}] = clipped_commitment;
    clipped_engine.restore(clipped_state);
    clipped_engine.decide(clipped, 0.1, 15.0);
    bool saw_a1_stop_ttc = false;
    bool saw_a1_stop_clip = false;
    for (const std::string& line : clipped_logs) {
        saw_a1_stop_ttc = saw_a1_stop_ttc ||
            line.find("[A1-STOP-TTC]") != std::string::npos;
        saw_a1_stop_clip = saw_a1_stop_clip ||
            line.find("[A1-STOP-CLIP]") != std::string::npos;
    }
    if (clipped[0].requested_action != VehicleAction::NOMINAL) {
        return fail("post-stop-line overlap still constrained the A1 owner");
    }
    if (clipped[1].requested_action != VehicleAction::NOMINAL ||
        clipped[1].reason != "clear") {
        return fail("far A1 stop_s applied a premature speed action");
    }
    if (clipped_engine.dynamicSpeedMetrics().baseline_conflicts != 0 ||
        saw_a1_stop_ttc || !saw_a1_stop_clip) {
        return fail(
            "A1 stop boundary did not clip the unreachable overlap: baseline=" +
            std::to_string(clipped_engine.dynamicSpeedMetrics().
                               baseline_conflicts) +
            " ttc_log=" + std::to_string(saw_a1_stop_ttc) +
            " clip_log=" + std::to_string(saw_a1_stop_clip));
    }

    // The same change must not weaken the frozen departure stop boundary.
    RuleEngine frozen_stop_engine(map_param, config);
    std::vector<VehicleAgent> frozen_stop{
        laneVehicle(0, 0.20, config.nominal_speed),
        laneVehicle(1, 0.89, config.nominal_speed)};
    frozen_stop[0].mission_phase = MissionPhase::TO_B;
    frozen_stop[1].mission_phase = MissionPhase::TO_A1;
    frozen_stop[1].track.set(
        RoughPath{wp(0.0, 10.0, 0.0), wp(4.0, 10.0, 0.0)});
    RuleEngine::SimSnapshot frozen_state;
    RuleEngine::DepartureClusterCommitment frozen_commitment;
    frozen_commitment.owner_id = 0;
    frozen_commitment.transaction_owner_path_gen = 0;
    frozen_commitment.owner_path_gen = 1;
    frozen_commitment.other_id = 1;
    frozen_commitment.other_path_gen = 1;
    frozen_commitment.intervals.push_back(
        FutureA1ConflictInterval{0.50, 1.00, 1.00, 1.50});
    frozen_commitment.waiter_stop_boundary_s = 1.00;
    frozen_commitment.waiter_stop_s = 0.90;
    frozen_commitment.owner_release_exit_s = 1.00;
    frozen_commitment.other_release_exit_s = 1.50;
    frozen_commitment.active = true;
    frozen_state.a1.departure_clusters[{0, 1}] = frozen_commitment;
    frozen_stop_engine.restore(frozen_state);
    frozen_stop_engine.decide(frozen_stop, 0.1, 15.0);
    if (frozen_stop[1].requested_action != VehicleAction::STOP ||
        frozen_stop[1].reason != "departure_cluster_priority" ||
        frozen_stop_engine.snapshot().a1.departure_clusters.empty() ||
        !frozen_stop_engine.snapshot().reservations.empty()) {
        return fail("frozen departure stop boundary protection was weakened");
    }

    // PICKUP_DWELL is inactive for pairwise motion, but its already-known
    // future A1->B geometry must be frozen and active immediately.
    RuleEngine pickup_engine(map_param, config);
    std::vector<VehicleAgent> pickup{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    pickup[0].mode = VehicleMode::DWELL;
    pickup[0].mission_phase = MissionPhase::PICKUP_DWELL;
    pickup[0].pending_dropoff_valid = true;
    pickup[0].pending_dropoff_track = pickup[0].track;
    pickup[0].a1_departure_priority_until_s = 1.0;
    RuleEngine::FutureA1Commitment pickup_owner;
    pickup_owner.owner_id = 0;
    pickup_owner.owner_path_gen = 1;
    pickup_owner.predicted_a1_arrival_time = 0.0;
    pickup_owner.predicted_to_b_time = 5.0;
    pickup_engine.setFutureA1Commitment(pickup_owner);
    pickup_engine.decide(pickup, 0.1, 15.0);
    const auto pickup_state = pickup_engine.snapshot();
    if (pickup_state.a1.departure_clusters.empty() ||
        !pickup_state.a1.departure_clusters.begin()->second.active ||
        pickup_state.a1.departure_clusters.begin()->second.
                transaction_owner_path_gen != 1 ||
        pickup_state.a1.departure_clusters.begin()->second.owner_path_gen != 2 ||
        pickup_state.a1.departure_clusters.begin()->second.
                frozen_owner_track.empty()) {
        return fail("PICKUP_DWELL future departure protection was not frozen");
    }

    // Three mutually crossing vehicles exercise all three pairwise dynamic
    // calls. Their requests must aggregate without a legacy reservation.
    RuleEngine multi_engine(map_param, config);
    std::vector<VehicleAgent> multi{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.70, true),
        diagonalVehicle(2, 1.10)};
    multi_engine.decide(multi, 0.1, 15.0);
    const auto& multi_metrics = multi_engine.dynamicSpeedMetrics();
    if (!multi_engine.snapshot().reservations.empty() ||
        multi_metrics.baseline_conflicts < 3 ||
        multi_metrics.reservation_create_multi_vehicle != 0 ||
        multi_engine.lastRollingDynamicDecision().targets.empty()) {
        return fail("three-vehicle pairs did not use dynamic aggregation");
    }
    std::vector<int> target_ids;
    for (const auto& target :
         multi_engine.lastRollingDynamicDecision().targets) {
        for (int id : target_ids) {
            if (id == target.vehicle_id) {
                return fail("multi-pair aggregate stored duplicate target");
            }
        }
        target_ids.push_back(target.vehicle_id);
    }

    // Collinear geometry is an ordinary generic timed conflict. It must use
    // the same deterministic priority as every other ordinary pair rather
    // than transferring authority to a front/rear classifier.
    RuleEngine following_engine(map_param, config);
    std::vector<std::string> following_logs;
    following_engine.setCoordLogSink(
        [&](const std::string& line) { following_logs.push_back(line); });
    std::vector<VehicleAgent> following{
        laneVehicle(0, 0.20, config.nominal_speed),
        laneVehicle(1, 0.48, 0.0)};
    following_engine.decide(following, 0.1, 15.0);
    bool generic_log = false;
    bool classified_log = false;
    for (const std::string& line : following_logs) {
        if (line.find("[DYN-SPEED]") == std::string::npos) continue;
        generic_log = generic_log ||
            line.find("interaction=GENERIC_TIMED_CONFLICT") !=
                std::string::npos;
        classified_log = classified_log ||
            line.find("interaction=OPPOSING") != std::string::npos ||
            line.find("interaction=CROSSING") != std::string::npos ||
            line.find("interaction=SAME_DIRECTION") != std::string::npos;
    }
    if (following_engine.dynamicSpeedMetrics().same_direction_conflicts != 0 ||
        following_engine.dynamicSpeedMetrics().crossing_conflicts == 0 ||
        !generic_log || classified_log ||
        !following_engine.snapshot().reservations.empty() ||
        following_engine.dynamicSpeedMetrics().
                duplicate_pair_authority_overrides != 0) {
        std::cerr << "same_direction_conflicts="
                  << following_engine.dynamicSpeedMetrics().
                         same_direction_conflicts
                  << " rear_action="
                  << static_cast<int>(following[0].requested_action)
                  << " front_action="
                  << static_cast<int>(following[1].requested_action)
                  << " rear_blocker=" << following[0].blocker_id
                  << " generic_log=" << generic_log
                  << " classified_log=" << classified_log
                  << " duplicate="
                  << following_engine.dynamicSpeedMetrics().
                         duplicate_pair_authority_overrides
                  << '\n';
        return fail("collinear pair did not use generic priority authority");
    }

    // A clear next rolling period returns to NOMINAL and reports recovery.
    RuleEngine recovery_engine(map_param, config);
    std::vector<VehicleAgent> recovery{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    recovery[1].path_s = 0.40;
    recovery_engine.decide(recovery, 0.1, 15.0);
    const auto prefix_a = predictTrajectory(
        recovery[0], map_param, config, VehicleAction::NOMINAL, 15.0);
    recovery[0].path_s = prefix_a.back().s;
    recovery[1].track.set(RoughPath{wp(10.0, 10.0, 0.0),
                                    wp(12.0, 10.0, 0.0)});
    recovery[1].path_gen += 1;
    recovery_engine.decide(recovery, 0.1, 15.0);
    if (hasDynamicReason(recovery, VehicleAction::YIELD) ||
        hasDynamicReason(recovery, VehicleAction::CREEP) ||
        !recovery_engine.snapshot().reservations.empty()) {
        return fail("next real rolling decision did not return to NOMINAL");
    }

    // A late frozen-closure intrusion from B0-B9 retreats to the original
    // pickup path origin. If another vehicle occupies that reverse sweep, the
    // correction must HOLD with the concrete blocker for DeadlockManager.
    VehicleAgent intrusion_owner = laneVehicle(10, 0.0, 0.0);
    intrusion_owner.mission_phase = MissionPhase::TO_B;
    intrusion_owner.track.set(
        RoughPath{wp(10.0, 10.0, 0.0), wp(12.0, 10.0, 0.0)});
    VehicleAgent intrusion_waiter = laneVehicle(11, 2.30, 0.0);
    intrusion_waiter.current_slot = 6;
    intrusion_waiter.mission_phase = MissionPhase::TO_A1;
    intrusion_waiter.track.set(
        RoughPath{wp(-2.0, 0.0, 0.0), wp(2.0, 0.0, 0.0)});
    RuleEngine::DepartureClusterCommitment intrusion_commitment;
    intrusion_commitment.owner_id = intrusion_owner.id;
    intrusion_commitment.owner_path_gen = intrusion_owner.path_gen;
    intrusion_commitment.other_id = intrusion_waiter.id;
    intrusion_commitment.other_path_gen = intrusion_waiter.path_gen;
    intrusion_commitment.frozen_owner_track.set(
        RoughPath{wp(0.0, -1.0, 1.5707963267948966),
                  wp(0.0, 1.0, 1.5707963267948966)});
    intrusion_commitment.frozen_waiter_track = intrusion_waiter.track;
    intrusion_commitment.intervals.push_back(
        FutureA1ConflictInterval{0.80, 1.20, 1.90, 2.20});
    intrusion_commitment.waiter_stop_boundary_s = 2.10;
    intrusion_commitment.waiter_stop_s = 2.00;
    intrusion_commitment.owner_release_exit_s = 1.20;
    intrusion_commitment.other_release_exit_s = 2.20;
    intrusion_commitment.active = true;
    RuleEngine::SimSnapshot intrusion_state;
    intrusion_state.a1.departure_clusters[
        {intrusion_owner.id, intrusion_waiter.id}] = intrusion_commitment;

    RuleEngine intrusion_engine(map_param, config);
    intrusion_engine.restore(intrusion_state);
    std::vector<VehicleAgent> intrusion_pair{
        intrusion_owner, intrusion_waiter};
    intrusion_engine.refreshA1IntrusionCorrections(intrusion_pair, 0.1);
    const auto clear_corrections =
        intrusion_engine.captureLiveA1IntrusionCorrections();
    const auto clear_correction = clear_corrections.find(intrusion_waiter.id);
    if (clear_correction == clear_corrections.end() ||
        clear_correction->second.motion !=
            A1Coordinator::IntrusionCorrectionMotion::RETREAT ||
        std::abs(clear_correction->second.target_s) > 1e-9 ||
        intrusion_engine.motionOverrideFor(intrusion_waiter.id).motion !=
            RecoveryMotion::RETREAT) {
        return fail("B0-B9 intrusion did not retreat to path s=0");
    }

    VehicleAgent sweep_blocker = laneVehicle(12, 0.10, 0.0);
    sweep_blocker.mode = VehicleMode::DWELL;
    sweep_blocker.track.set(
        RoughPath{wp(-1.1, 0.0, 0.0), wp(-1.0, 0.0, 0.0)});
    RuleEngine blocked_intrusion_engine(map_param, config);
    blocked_intrusion_engine.restore(intrusion_state);
    std::vector<VehicleAgent> blocked_intrusion{
        intrusion_owner, intrusion_waiter, sweep_blocker};
    blocked_intrusion_engine.refreshA1IntrusionCorrections(
        blocked_intrusion, 0.1);
    const auto blocked_corrections =
        blocked_intrusion_engine.captureLiveA1IntrusionCorrections();
    const auto blocked_correction =
        blocked_corrections.find(intrusion_waiter.id);
    if (blocked_correction == blocked_corrections.end() ||
        blocked_correction->second.motion !=
            A1Coordinator::IntrusionCorrectionMotion::HOLD ||
        blocked_correction->second.blocker_id != sweep_blocker.id ||
        blocked_intrusion[1].blocker_id != sweep_blocker.id ||
        blocked_intrusion[1].reason !=
            "a1_intrusion_retreat_sweep_blocked") {
        return fail("A1 blocked retreat lost its concrete blocker");
    }
    RuleEngine::SimSnapshot deadlock_handoff_state =
        blocked_intrusion_engine.snapshot();
    deadlock_handoff_state.deadlock.directive.phase =
        RecoveryPhase::RETREAT;
    deadlock_handoff_state.deadlock.directive.retreat_vehicle_id =
        intrusion_waiter.id;
    deadlock_handoff_state.deadlock.directive.pass_vehicle_id =
        intrusion_owner.id;
    deadlock_handoff_state.deadlock.directive.retreat_target_s = 0.50;
    blocked_intrusion_engine.restore(deadlock_handoff_state);
    blocked_intrusion_engine.restoreLiveA1IntrusionCorrections(
        blocked_corrections);
    const auto handed_off_motion =
        blocked_intrusion_engine.motionOverrideFor(intrusion_waiter.id);
    if (handed_off_motion.motion != RecoveryMotion::RETREAT ||
        handed_off_motion.a1_intrusion ||
        std::abs(handed_off_motion.target_s - 0.50) > 1e-9) {
        return fail("same-pair Deadlock did not take over A1 HOLD");
    }
    deadlock_handoff_state.deadlock.directive = RecoveryDirective{};
    deadlock_handoff_state.deadlock.priority_override.active = true;
    deadlock_handoff_state.deadlock.priority_override.vehicle_a =
        intrusion_owner.id;
    deadlock_handoff_state.deadlock.priority_override.vehicle_b =
        intrusion_waiter.id;
    deadlock_handoff_state.deadlock.priority_override.path_gen_a =
        intrusion_owner.path_gen;
    deadlock_handoff_state.deadlock.priority_override.path_gen_b =
        intrusion_waiter.path_gen;
    deadlock_handoff_state.deadlock.priority_override.winner_id =
        intrusion_waiter.id;
    blocked_intrusion_engine.restore(deadlock_handoff_state);
    blocked_intrusion_engine.restoreLiveA1IntrusionCorrections(
        blocked_corrections);
    const auto priority_handoff_motion =
        blocked_intrusion_engine.motionOverrideFor(intrusion_waiter.id);
    if (priority_handoff_motion.motion != RecoveryMotion::NORMAL ||
        priority_handoff_motion.a1_intrusion) {
        return fail("Deadlock priority pass did not take over A1 HOLD");
    }

    std::cout << "dynamic_speed_rule_engine_test: PASS\n";
    return 0;
}
