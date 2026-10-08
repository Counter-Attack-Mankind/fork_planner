#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

int fail(const std::string& message) {
    std::cerr << "spatiotemporal_interaction_test: " << message << '\n';
    return 1;
}

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

RoughWp wp(double x, double y, double theta = 0.0) {
    return RoughWp{x, y, theta, WpType::FORWARD};
}

RoughPath line(double x0, double y0, double x1, double y1) {
    return RoughPath{wp(x0, y0, std::atan2(y1 - y0, x1 - x0)),
                     wp(x1, y1, std::atan2(y1 - y0, x1 - x0))};
}

VehicleAgent vehicle(int id, const RoughPath& path, int path_gen = 1,
                     double path_s = 0.0, double speed = 0.0) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = path_gen;
    result.path_s = path_s;
    result.current_speed = speed;
    result.track.set(path);
    return result;
}

PredictedKinematicSample sample(double t, double s, double x, double y) {
    PredictedKinematicSample result;
    result.t = t;
    result.s = s;
    result.body = OBB{x, y, 0.0, 0.10, 0.10};
    return result;
}

bool sameControlState(const VehicleAgent& lhs, const VehicleAgent& rhs) {
    return lhs.id == rhs.id && lhs.mode == rhs.mode &&
           lhs.mission_phase == rhs.mission_phase &&
           lhs.action == rhs.action &&
           lhs.requested_action == rhs.requested_action &&
           lhs.blocker_id == rhs.blocker_id &&
           near(lhs.action_hold_remaining, rhs.action_hold_remaining) &&
           near(lhs.ttc_stop_hold_remaining,
                rhs.ttc_stop_hold_remaining) &&
           near(lhs.wait_time, rhs.wait_time) &&
           near(lhs.dwell_remaining, rhs.dwell_remaining) &&
           near(lhs.path_s, rhs.path_s) &&
           near(lhs.current_speed, rhs.current_speed) &&
           lhs.path_gen == rhs.path_gen && lhs.reason == rhs.reason;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_step = 0.05;
    config.prediction_horizon = 20.0;
    RuleEngine engine(map_param, config);

    // 1. Separated complete paths have no potential geometry or timed event.
    VehicleAgent parallel_a = vehicle(0, line(0.0, 0.0, 2.0, 0.0));
    VehicleAgent parallel_b = vehicle(1, line(0.0, 1.0, 2.0, 1.0));
    const PairInteractionResult none =
        engine.detectPairInteraction(parallel_a, parallel_b, 12.0);
    if (!none.potential_zones.empty() || none.event.valid) {
        return fail("separated paths produced an interaction");
    }

    // 2. Complete paths cross, but the two baseline trajectories reach the
    // crossing at different times, so no TimedEvent is produced.
    VehicleAgent early = vehicle(2, line(-1.0, 0.0, 1.0, 0.0));
    VehicleAgent late = vehicle(3, line(0.0, -3.0, 0.0, 1.0));
    const PairInteractionResult offset =
        engine.detectPairInteraction(early, late, 20.0);
    if (offset.potential_zones.empty() || offset.event.valid) {
        return fail("time-offset crossing classification changed");
    }

    // 3. Equal-distance crossing paths yield one synchronized event.
    VehicleAgent crossing_a = vehicle(4, line(-1.0, 0.0, 1.0, 0.0));
    VehicleAgent crossing_b = vehicle(5, line(0.0, -1.0, 0.0, 1.0));
    const PairInteractionResult crossing =
        engine.detectPairInteraction(crossing_a, crossing_b, 12.0);
    if (crossing.potential_zones.empty() || !crossing.event.valid ||
        crossing.event.associated_zone_index != -1 ||
        crossing.type != PairInteractionType::CROSSING ||
        crossing.event.last_t < crossing.event.first_overlap_t ||
        crossing.event.timed_overlaps.empty()) {
        return fail("synchronized crossing event was not detected");
    }

    // Crossing truth is independent of static potential zones.
    const auto direct_prediction_a = predictTrajectory(
        crossing_a, map_param, config, VehicleAction::NOMINAL, 12.0);
    const auto direct_prediction_b = predictTrajectory(
        crossing_b, map_param, config, VehicleAction::NOMINAL, 12.0);
    const PairInteractionResult zone_free_crossing =
        detectPairInteractionFromPredictions(
            crossing_a, crossing_b, {}, direct_prediction_a,
            direct_prediction_b);
    if (!zone_free_crossing.event.valid ||
        zone_free_crossing.event.associated_zone_index != -1 ||
        zone_free_crossing.event.timed_overlaps.empty() ||
        !near(zone_free_crossing.event.ttc_a,
              predictionTimeAtS(
                  direct_prediction_a,
                  zone_free_crossing.event.collision_s_a)) ||
        !near(zone_free_crossing.event.ttc_b,
              predictionTimeAtS(
                  direct_prediction_b,
                  zone_free_crossing.event.collision_s_b))) {
        return fail("zone-free synchronized crossing was not detected");
    }

    // 4. Multiple separated overlap periods return the first contiguous one.
    std::vector<PotentialConflictZone> two_zones(2);
    two_zones[0].s_self_enter = 0.8;
    two_zones[0].s_self_exit = 1.2;
    two_zones[0].s_other_enter = 0.8;
    two_zones[0].s_other_exit = 1.2;
    two_zones[1].s_self_enter = 2.8;
    two_zones[1].s_self_exit = 3.2;
    two_zones[1].s_other_enter = 2.8;
    two_zones[1].s_other_exit = 3.2;
    const std::vector<PredictedKinematicSample> prediction_a{
        sample(0.0, 0.0, 0.0, 0.0), sample(1.0, 1.0, 1.0, 0.0),
        sample(2.0, 2.0, 2.0, 0.0), sample(3.0, 3.0, 3.0, 0.0)};
    const std::vector<PredictedKinematicSample> prediction_b{
        sample(0.0, 0.0, 0.0, 1.0), sample(1.0, 1.0, 1.0, 0.0),
        sample(2.0, 2.0, 2.0, 1.0), sample(3.0, 3.0, 3.0, 0.0)};
    const PairInteractionResult first_only =
        detectPairInteractionFromPredictions(
            crossing_a, crossing_b, two_zones, prediction_a, prediction_b);
    if (!first_only.event.valid ||
        !near(first_only.event.first_overlap_t, 1.0) ||
        !near(first_only.event.last_t, 1.0) ||
        first_only.event.timed_overlaps.size() != 1 ||
        first_only.event.associated_zone_index != 0) {
        return fail("detector did not retain only the first overlap event");
    }

    // 5. Baseline prediction accelerates from a low current speed toward the
    // existing NOMINAL target using the configured acceleration limit.
    VehicleAgent low_speed = vehicle(6, line(0.0, 0.0, 2.0, 0.0));
    const auto low_prediction =
        predictTrajectory(low_speed, map_param, config,
                          VehicleAction::NOMINAL, 1.0);
    if (low_prediction.size() < 2 ||
        !near(low_prediction[1].speed,
              config.max_accel * config.prediction_step, 1e-12) ||
        low_prediction[1].speed > config.nominal_speed + 1e-12) {
        return fail("low-speed baseline acceleration changed");
    }

    // 6. The existing curvature cap participates in the same predictor.
    RoughPath arc;
    constexpr double radius = 0.5;
    constexpr int arc_points = 101;
    for (int index = 0; index < arc_points; ++index) {
        const double angle = 1.5707963267948966 * index / (arc_points - 1);
        arc.push_back(wp(radius * std::cos(angle),
                         radius * std::sin(angle),
                         angle + 1.5707963267948966));
    }
    MultiVehicleConfig curve_config = config;
    curve_config.lat_accel_max = 0.001;
    VehicleAgent curved = vehicle(7, arc, 1, 0.20, config.nominal_speed);
    VehicleAgent straight = vehicle(
        8, line(0.0, 0.0, 2.0, 0.0), 1, 0.20, config.nominal_speed);
    const auto curved_prediction =
        predictTrajectory(curved, map_param, curve_config,
                          VehicleAction::NOMINAL, 0.10);
    const auto straight_prediction =
        predictTrajectory(straight, map_param, curve_config,
                          VehicleAction::NOMINAL, 0.10);
    if (curved_prediction.size() < 2 || straight_prediction.size() < 2 ||
        !(curved_prediction[1].speed < straight_prediction[1].speed) ||
        !near(straight_prediction[1].speed, config.nominal_speed)) {
        return fail("curvature speed cap did not participate in prediction");
    }

    // 7. Reaching the path endpoint clamps s and then forces speed to zero.
    VehicleAgent short_path = vehicle(9, line(0.0, 0.0, 0.005, 0.0));
    const auto endpoint_prediction =
        predictTrajectory(short_path, map_param, config,
                          VehicleAction::NOMINAL, 1.0);
    if (endpoint_prediction.empty() ||
        !near(endpoint_prediction.back().s, short_path.track.length()) ||
        !near(endpoint_prediction.back().speed, 0.0)) {
        return fail("path-end baseline behavior changed");
    }

    // 8. Association is based on the first event's arc-length sample, even
    // when two geometric conflict zones exist.
    std::vector<PredictedKinematicSample> near_second_a{
        sample(0.0, 0.0, 0.0, 1.0), sample(1.0, 3.0, 0.0, 0.0)};
    std::vector<PredictedKinematicSample> near_second_b{
        sample(0.0, 0.0, 1.0, 0.0), sample(1.0, 3.0, 0.0, 0.0)};
    const PairInteractionResult second_zone =
        detectPairInteractionFromPredictions(
            crossing_a, crossing_b, two_zones,
            near_second_a, near_second_b);
    if (!second_zone.event.valid ||
        second_zone.event.associated_zone_index != 1) {
        return fail("two-zone first-event association changed");
    }

    // 9. The geometry cache identity follows path_gen. Replacing a path and
    // incrementing its generation must not reuse the old crossing geometry.
    VehicleAgent cached_b = crossing_b;
    const PairInteractionResult cached_crossing =
        engine.detectPairInteraction(crossing_a, cached_b, 12.0);
    cached_b.track.set(line(-1.0, 1.0, 1.0, 1.0));
    ++cached_b.path_gen;
    const PairInteractionResult invalidated =
        engine.detectPairInteraction(crossing_a, cached_b, 12.0);
    if (cached_crossing.potential_zones.empty() ||
        !invalidated.potential_zones.empty() || invalidated.event.valid ||
        invalidated.path_gen_b != cached_b.path_gen) {
        return fail("path_gen did not invalidate potential-zone geometry");
    }

    // 10. A detection call may populate only the geometry cache. It must not
    // change reservations, commitments, tokens, markers, time, vehicle
    // actions/state-machine fields, or deadlock fields.
    RuleEngine::SimSnapshot seeded;
    RuleEngine::ConflictReservation reservation;
    reservation.owner_id = crossing_a.id;
    seeded.reservations[{crossing_a.id, crossing_b.id}] = reservation;
    RuleEngine::DepartureClusterCommitment departure;
    departure.owner_id = crossing_a.id;
    departure.other_id = crossing_b.id;
    seeded.a1.departure_clusters[{crossing_a.id, crossing_b.id}] = departure;
    seeded.following_pairs.insert({crossing_a.id, crossing_b.id});
    seeded.tokens.grant(77, crossing_a.id, 6.0);
    seeded.conflicts.push_back(ConflictMarker{});
    seeded.now = 7.5;
    engine.restore(seeded);
    RuleEngine::FutureA1Commitment future;
    future.owner_id = crossing_a.id;
    future.owner_path_gen = crossing_a.path_gen;
    future.predicted_a1_arrival_time = 2.0;
    future.predicted_to_b_time = 3.0;
    engine.setFutureA1Commitment(future);
    crossing_a.blocker_id = 99;
    crossing_a.wait_time = 4.0;
    crossing_a.reason = "sentinel";
    const VehicleAgent before_vehicle = crossing_a;
    const RuleEngine::SimSnapshot before_state = engine.snapshot();
    const RuleEngine::FutureA1Commitment before_future =
        engine.futureA1Commitment();
    (void)engine.detectPairInteraction(crossing_a, crossing_b, 12.0);
    const RuleEngine::SimSnapshot after_state = engine.snapshot();
    const RuleEngine::FutureA1Commitment after_future =
        engine.futureA1Commitment();
    if (!sameControlState(crossing_a, before_vehicle) ||
        before_state.reservations.size() != after_state.reservations.size() ||
        before_state.a1.departure_clusters.size() !=
            after_state.a1.departure_clusters.size() ||
        before_state.following_pairs != after_state.following_pairs ||
        before_state.tokens.holder(77) != after_state.tokens.holder(77) ||
        before_state.conflicts.size() != after_state.conflicts.size() ||
        !near(before_state.now, after_state.now) ||
        before_future.owner_id != after_future.owner_id ||
        before_future.owner_path_gen != after_future.owner_path_gen ||
        !near(before_future.predicted_a1_arrival_time,
              after_future.predicted_a1_arrival_time) ||
        !near(before_future.predicted_to_b_time,
              after_future.predicted_to_b_time)) {
        return fail("pure detector changed coordination state");
    }

    std::cout << "spatiotemporal_interaction_test: PASS\n";
    return 0;
}
