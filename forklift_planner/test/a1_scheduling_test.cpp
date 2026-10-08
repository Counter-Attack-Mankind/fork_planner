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

    std::cout << "a1_scheduling_test: PASS\n";
    return 0;
}
