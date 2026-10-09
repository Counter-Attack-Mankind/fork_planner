#include "forklift_planner/multi_vehicle/a1/a1_coordinator.h"

#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

int fail(const std::string& message) {
    std::cerr << "a1_scheduling_test: " << message << '\n';
    return 1;
}

RoughWp wp(double x) {
    return RoughWp{x, 0.0, 0.0, WpType::FORWARD};
}

VehicleAgent requestVehicle(int id, double distance) {
    VehicleAgent vehicle;
    vehicle.id = id;
    vehicle.mode = VehicleMode::ACTIVE;
    vehicle.mission_phase = MissionPhase::TO_A1;
    vehicle.leg_target = LegTargetKind::A1;
    vehicle.path_gen = 1;
    vehicle.track.set(RoughPath{wp(0.0), wp(distance)});
    return vehicle;
}

VehicleAgent departingOwner(int id) {
    VehicleAgent vehicle;
    vehicle.id = id;
    vehicle.mode = VehicleMode::ACTIVE;
    vehicle.mission_phase = MissionPhase::TO_B;
    vehicle.leg_target = LegTargetKind::B_SLOT;
    vehicle.path_gen = 2;
    vehicle.loaded = true;
    vehicle.target_slot = 12;
    vehicle.a1_departure_committed = true;
    vehicle.a1_departure_priority_until_s = 1.0;
    vehicle.track.set(RoughPath{wp(0.0), wp(2.0)});
    return vehicle;
}

A1Coordinator::FutureA1Commitment commitment(int owner_id, int path_gen) {
    A1Coordinator::FutureA1Commitment result;
    result.owner_id = owner_id;
    result.owner_path_gen = path_gen;
    result.predicted_a1_arrival_time = 0.0;
    result.predicted_to_b_time = 0.0;
    return result;
}

void seedDepartureTransaction(A1Coordinator& coordinator, int owner_id,
                              int owner_path_gen, int waiter_id,
                              int waiter_path_gen) {
    A1Coordinator::Snapshot snapshot = coordinator.snapshot();
    snapshot.future_a1_commitment = commitment(owner_id, owner_path_gen);
    A1Coordinator::DepartureClusterCommitment cluster;
    cluster.owner_id = owner_id;
    cluster.transaction_owner_path_gen = owner_path_gen - 1;
    cluster.owner_path_gen = owner_path_gen;
    cluster.other_id = waiter_id;
    cluster.other_path_gen = waiter_path_gen;
    cluster.owner_release_exit_s = 1.0;
    cluster.active = true;
    snapshot.departure_clusters[{std::min(owner_id, waiter_id),
                                 std::max(owner_id, waiter_id)}] = cluster;
    coordinator.restore(snapshot);
}

A1Coordinator::ArrivalKinematics kinematics() {
    A1Coordinator::ArrivalKinematics result;
    result.dt = 0.1;
    result.desired_speed = [](const VehicleAgent&) { return 1.0; };
    result.limited_speed = [](double, double desired, double) {
        return desired;
    };
    return result;
}

void seedBypassCounts(A1Coordinator& coordinator,
                      const std::map<int, int>& counts) {
    A1Coordinator::Snapshot snapshot = coordinator.snapshot();
    snapshot.bypass_count = counts;
    for (const auto& item : counts) {
        snapshot.bypass_request_path_gen[item.first] = 1;
    }
    coordinator.restore(snapshot);
}

A1Coordinator makeCoordinator(const MapParam& map_param,
                              const MultiVehicleConfig& config) {
    A1Coordinator::Dependencies dependencies;
    dependencies.compute_full_conflict_zones =
        [](const VehicleAgent&, const VehicleAgent&) {
            return std::vector<A1Coordinator::ConflictZone>{};
        };
    dependencies.current_conflict_zones =
        dependencies.compute_full_conflict_zones;
    dependencies.unified_priority =
        [](const VehicleAgent& lhs, const VehicleAgent& rhs) {
            return std::min(lhs.id, rhs.id);
        };
    return A1Coordinator(map_param, config, std::move(dependencies));
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_step = 0.05;
    const auto arrival_kinematics = kinematics();
    const std::vector<VehicleAgent> candidates{
        requestVehicle(10, 5.0), requestVehicle(20, 2.0)};

    A1Coordinator normal = makeCoordinator(map_param, config);
    normal.refreshPlanningContext(candidates, 45.0, 0.0,
                                  arrival_kinematics);
    if (normal.futureA1Commitment().owner_id != 20 ||
        normal.bypassCount(10) != 1 || normal.bypassCount(20) != 0) {
        return fail("normal ETA selection or CREATE count update failed");
    }

    A1Coordinator count_two = makeCoordinator(map_param, config);
    seedBypassCounts(count_two, {{10, 2}, {20, 0}});
    count_two.refreshPlanningContext(candidates, 45.0, 0.0,
                                     arrival_kinematics);
    if (count_two.futureA1Commitment().owner_id != 20 ||
        count_two.bypassCount(10) != 3) {
        return fail("count 2 incorrectly overrode ETA");
    }

    A1Coordinator count_three = makeCoordinator(map_param, config);
    seedBypassCounts(count_three, {{10, 3}, {20, 0}});
    count_three.refreshPlanningContext(candidates, 45.0, 0.0,
                                       arrival_kinematics);
    if (count_three.futureA1Commitment().owner_id != 10 ||
        count_three.bypassCount(10) != 0 ||
        count_three.bypassCount(20) != 1) {
        return fail("count 3 did not activate starvation priority");
    }

    A1Coordinator count_five = makeCoordinator(map_param, config);
    seedBypassCounts(count_five, {{10, 5}, {20, 0}});
    count_five.refreshPlanningContext(candidates, 45.0, 0.0,
                                      arrival_kinematics);
    if (count_five.futureA1Commitment().owner_id != 10) {
        return fail("count 5 did not activate starvation priority");
    }

    A1Coordinator multiple_starved = makeCoordinator(map_param, config);
    seedBypassCounts(multiple_starved, {{10, 3}, {20, 5}});
    multiple_starved.refreshPlanningContext(candidates, 45.0, 0.0,
                                            arrival_kinematics);
    if (multiple_starved.futureA1Commitment().owner_id != 20) {
        return fail("largest starvation count did not win");
    }

    A1Coordinator equal_starved = makeCoordinator(map_param, config);
    seedBypassCounts(equal_starved, {{10, 3}, {20, 3}});
    equal_starved.refreshPlanningContext(candidates, 45.0, 0.0,
                                         arrival_kinematics);
    if (equal_starved.futureA1Commitment().owner_id != 20) {
        return fail("ETA did not break an equal starvation count");
    }

    A1Coordinator priority_tie = makeCoordinator(map_param, config);
    std::vector<VehicleAgent> equal_eta{
        requestVehicle(10, 2.0), requestVehicle(20, 2.0)};
    seedBypassCounts(priority_tie, {{10, 3}, {20, 3}});
    priority_tie.refreshPlanningContext(equal_eta, 45.0, 0.0,
                                        arrival_kinematics);
    if (priority_tie.futureA1Commitment().owner_id != 10) {
        return fail("unified_priority did not break an equal ETA tie");
    }

    A1Coordinator hold = makeCoordinator(map_param, config);
    int scheduling_logs = 0;
    hold.setCoordLogSink([&](const std::string& line) {
        if (line.find("[A1_SCHEDULING]") != std::string::npos) {
            ++scheduling_logs;
        }
    });
    hold.refreshPlanningContext(candidates, 45.0, 0.0, arrival_kinematics);
    const auto after_create = hold.snapshot();
    hold.refreshPlanningContext(candidates, 45.0, 2.0, arrival_kinematics);
    if (hold.bypassCount(10) != 1 || hold.bypassCount(20) != 0 ||
        scheduling_logs != 1) {
        return fail("owner HOLD repeated CREATE counting or scheduling log");
    }

    A1Coordinator::Snapshot rollout_frame = after_create;
    rollout_frame.bypass_count[10] = 99;
    hold.restore(rollout_frame, false);
    if (hold.bypassCount(10) != 1) {
        return fail("rollout frame installation polluted live bypass counts");
    }

    hold.clearFutureA1Commitment();
    hold.refreshPlanningContext(candidates, 45.0, 4.0, arrival_kinematics);
    if (hold.bypassCount(10) != 2) {
        return fail("second CREATE did not increment exactly once");
    }
    hold.restore(after_create);
    if (hold.bypassCount(10) != 1 || hold.bypassCount(20) != 0) {
        return fail("rollout snapshot restore polluted live bypass counts");
    }

    A1Coordinator request_lifetime = makeCoordinator(map_param, config);
    seedBypassCounts(request_lifetime, {{10, 3}});
    std::vector<VehicleAgent> outside_horizon{requestVehicle(10, 100.0)};
    request_lifetime.refreshPlanningContext(outside_horizon, 45.0, 0.0,
                                            arrival_kinematics);
    if (request_lifetime.bypassCount(10) != 3) {
        return fail("temporary horizon exclusion cleared request history");
    }
    outside_horizon[0].mode = VehicleMode::NEED_TASK;
    request_lifetime.refreshPlanningContext(outside_horizon, 45.0, 2.0,
                                            arrival_kinematics);
    if (request_lifetime.bypassCount(10) != 0) {
        return fail("cancelled request retained bypass history");
    }

    A1Coordinator handoff = makeCoordinator(map_param, config);
    std::vector<VehicleAgent> handoff_vehicles{
        departingOwner(30), requestVehicle(10, 2.0),
        requestVehicle(20, 5.0)};
    seedDepartureTransaction(handoff, 30, 2, 10, 1);
    handoff.refreshPlanningContext(handoff_vehicles, 45.0, 0.0,
                                   arrival_kinematics);
    if (handoff.futureA1Commitment().owner_id != 30 ||
        handoff.reservedA1Commitment().owner_id != 10 ||
        handoff.bypassCount(20) != 0) {
        return fail("successor reservation changed authority or wait counts");
    }

    handoff_vehicles[1].track.set(RoughPath{wp(0.0), wp(10.0)});
    handoff_vehicles[2].track.set(RoughPath{wp(0.0), wp(1.0)});
    handoff.refreshPlanningContext(handoff_vehicles, 45.0, 2.0,
                                   arrival_kinematics);
    if (handoff.reservedA1Commitment().owner_id != 10) {
        return fail("reservation HOLD reranked the successor");
    }

    const A1Coordinator::Snapshot before_promotion = handoff.snapshot();
    handoff_vehicles[0].path_s = 1.01;
    handoff.refreshDepartureClusterCommitments(handoff_vehicles);
    if (handoff.futureA1Commitment().owner_id != 10 ||
        handoff.reservedA1Commitment().valid() ||
        !handoff.departureClusters().empty() ||
        handoff.bypassCount(10) != 0 || handoff.bypassCount(20) != 1) {
        return fail("last-cluster release did not atomically promote once");
    }
    const A1Coordinator::Snapshot after_promotion = handoff.snapshot();
    handoff.refreshDepartureClusterCommitments(handoff_vehicles);
    if (handoff.bypassCount(20) != 1) {
        return fail("PROMOTE updated starvation counts more than once");
    }
    handoff.restore(before_promotion);
    if (handoff.futureA1Commitment().owner_id != 30 ||
        handoff.reservedA1Commitment().owner_id != 10 ||
        handoff.bypassCount(20) != 0) {
        return fail("rollout snapshot did not restore handoff state");
    }
    handoff.restore(after_promotion);
    if (handoff.futureA1Commitment().owner_id != 10 ||
        handoff.reservedA1Commitment().valid() ||
        handoff.bypassCount(20) != 1) {
        return fail("executed handoff snapshot did not install promotion");
    }

    handoff_vehicles[1].path_s = 0.0;
    handoff_vehicles[1].requested_action = VehicleAction::NOMINAL;
    handoff_vehicles[2].path_s = 0.95;
    handoff_vehicles[2].requested_action = VehicleAction::NOMINAL;
    handoff.enforceA1EntryControl(
        handoff_vehicles, 0.1,
        [](VehicleAgent& vehicle, VehicleAction action,
           const std::string& reason, int blocker_id) {
            vehicle.requested_action = action;
            vehicle.reason = reason;
            vehicle.blocker_id = blocker_id;
        });
    if (handoff_vehicles[1].requested_action != VehicleAction::NOMINAL ||
        handoff_vehicles[2].requested_action != VehicleAction::STOP ||
        handoff_vehicles[2].reason != "a1_entry_owner_not_ready") {
        return fail("unprepared owner gate did not stop only entry traffic");
    }

    A1Coordinator ownerless = makeCoordinator(map_param, config);
    std::vector<VehicleAgent> ownerless_vehicles{
        requestVehicle(10, 1.0), requestVehicle(20, 10.0)};
    ownerless_vehicles[0].path_s = 0.95;
    ownerless_vehicles[0].requested_action = VehicleAction::NOMINAL;
    ownerless_vehicles[1].requested_action = VehicleAction::NOMINAL;
    ownerless.enforceA1EntryControl(
        ownerless_vehicles, 0.1,
        [](VehicleAgent& vehicle, VehicleAction action,
           const std::string& reason, int blocker_id) {
            vehicle.requested_action = action;
            vehicle.reason = reason;
            vehicle.blocker_id = blocker_id;
        });
    if (ownerless_vehicles[0].requested_action != VehicleAction::STOP ||
        ownerless_vehicles[0].reason != "a1_entry_no_owner" ||
        ownerless_vehicles[1].requested_action != VehicleAction::NOMINAL) {
        return fail("ownerless gate stopped distant traffic or left entry open");
    }

    A1Coordinator b0_reservation = makeCoordinator(map_param, config);
    VehicleAgent parked;
    parked.id = 40;
    parked.mode = VehicleMode::DWELL;
    parked.mission_phase = MissionPhase::UNLOAD_DWELL;
    parked.leg_target = LegTargetKind::B_SLOT;
    parked.current_slot = 0;
    parked.path_gen = 0;
    std::vector<VehicleAgent> parked_vehicles{departingOwner(30), parked};
    seedDepartureTransaction(b0_reservation, 30, 2, 40, 1);
    auto parked_kinematics = arrival_kinematics;
    parked_kinematics.pickup_leg_track = [](int, PathTrack& track) {
        track.set(RoughPath{wp(0.0), wp(2.0)});
        return true;
    };
    b0_reservation.refreshPlanningContext(parked_vehicles, 45.0, 0.0,
                                           parked_kinematics);
    if (b0_reservation.futureA1Commitment().owner_id != 30 ||
        b0_reservation.reservedA1Commitment().owner_id != 40 ||
        parked_vehicles[1].mode != VehicleMode::DWELL) {
        return fail("B0 reservation granted formal authority or launched early");
    }

    std::cout << "a1_scheduling_test: PASS\n";
    return 0;
}
