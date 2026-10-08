#include "forklift_planner/multi_vehicle/bridge_ttc_correction.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

constexpr double kPi = 3.14159265358979323846;

int fail(const std::string& message) {
    std::cerr << "bridge_ttc_correction_test: " << message << '\n';
    return 1;
}

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

RoughWp wp(double x, double y, double theta, WpType type) {
    return RoughWp{x, y, theta, type};
}

VehicleAgent vehicle(int id, const RoughPath& path) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.track.set(path);
    return result;
}

std::vector<PredictedKinematicSample> prediction(double length,
                                                  double duration) {
    std::vector<PredictedKinematicSample> result;
    for (int i = 0; i <= 10; ++i) {
        PredictedKinematicSample sample;
        sample.t = duration * i / 10.0;
        sample.s = length * i / 10.0;
        result.push_back(sample);
    }
    return result;
}

PairInteractionResult conflict(double t, double s_a, double s_b) {
    PairInteractionResult result;
    result.event.valid = true;
    result.event.first_overlap_t = t;
    result.event.collision_s_a = s_a;
    result.event.collision_s_b = s_b;
    result.event.danger_s_a = s_a;
    result.event.danger_s_b = s_b;
    result.event.ttc_a = t;
    result.event.ttc_b = t;
    return result;
}

}  // namespace

int main() {
    MapParam map;
    MultiVehicleConfig config;
    config.conflict_margin = 0.02;
    config.bridge_opposing_threshold = -0.5;
    config.bridge_backtrack_step = 0.1;

    const VehicleAgent forward = vehicle(
        1, {wp(0.0, 0.0, 0.0, WpType::FORWARD),
            wp(2.0, 0.0, 0.0, WpType::FORWARD)});
    const VehicleAgent opposing = vehicle(
        2, {wp(2.0, 0.05, kPi, WpType::FORWARD),
            wp(0.0, 0.05, kPi, WpType::FORWARD)});
    const auto forward_prediction = prediction(2.0, 10.0);
    const auto opposing_prediction = prediction(2.0, 10.0);

    // A clear nominal baseline must leave the bridge layer entirely dormant.
    const PairBridgeTtcCorrection clear = evaluateBridgeTtcCorrection(
        forward, opposing, forward_prediction, opposing_prediction, {},
        map, config);
    if (clear.baseline_conflict || clear.a.backtrack_samples != 0 ||
        clear.b.nearest_search_evaluations != 0) {
        return fail("clear baseline activated bridge evaluation");
    }

    // A sustained opposing relation backtracks independently and can only
    // move TTC earlier. The supplied nominal prediction is the time source.
    const PairBridgeTtcCorrection stable = evaluateBridgeTtcCorrection(
        forward, opposing, forward_prediction, opposing_prediction,
        conflict(8.0, 1.6, 0.4), map, config);
    if (!stable.a.bridge_related || !stable.b.bridge_related ||
        stable.a.near_boundary_s > 1e-9 ||
        stable.a.corrected_ttc > stable.a.original_ttc ||
        std::abs(stable.a.corrected_ttc) > 1e-9 ||
        stable.a.nearest_search_evaluations <= 0) {
        return fail("stable opposing bridge correction is incomplete");
    }

    // Geometric proximity without an opposing actual motion heading is not a
    // bridge relation.
    const VehicleAgent same_direction = vehicle(
        3, {wp(0.0, 0.04, 0.0, WpType::FORWARD),
            wp(2.0, 0.04, 0.0, WpType::FORWARD)});
    const PairBridgeTtcCorrection same = evaluateBridgeTtcCorrection(
        forward, same_direction, forward_prediction, forward_prediction,
        conflict(6.0, 1.0, 1.0), map, config);
    if (same.a.bridge_related || !near(same.a.original_ttc, 5.0) ||
        !near(same.a.corrected_ttc, 5.0)) {
        return fail("same-direction proximity was classified as bridge");
    }

    // REVERSE path heading is converted to actual motion heading before the
    // direction test. theta=0 REVERSE therefore opposes theta=0 FORWARD.
    const VehicleAgent reverse = vehicle(
        4, {wp(0.0, 0.03, 0.0, WpType::REVERSE),
            wp(2.0, 0.03, 0.0, WpType::REVERSE)});
    const PairBridgeTtcCorrection reverse_relation =
        evaluateBridgeTtcCorrection(
            forward, reverse, forward_prediction, forward_prediction,
            conflict(5.0, 1.0, 1.0), map, config);
    if (!reverse_relation.a.bridge_related ||
        reverse_relation.a.collision_direction_dot > -0.999) {
        return fail("REVERSE actual motion heading was not used");
    }

    // Distance is part of the predicate; far parallel/opposing paths cannot
    // be promoted solely from heading.
    const VehicleAgent far_opposing = vehicle(
        5, {wp(2.0, 0.5, kPi, WpType::FORWARD),
            wp(0.0, 0.5, kPi, WpType::FORWARD)});
    const PairBridgeTtcCorrection far = evaluateBridgeTtcCorrection(
        forward, far_opposing, forward_prediction, opposing_prediction,
        conflict(5.0, 1.0, 1.0), map, config);
    if (far.a.bridge_related) {
        return fail("far opposing paths passed bridge proximity");
    }

    // A traversal change is diagnostic, not a bridge boundary. When actual
    // motion remains opposing across a duplicated F/R cusp, backtracking must
    // continue to the real relation boundary at path start.
    const VehicleAgent consistent_cusp = vehicle(
        6, {wp(0.0, 0.0, kPi, WpType::REVERSE),
            wp(0.5, 0.0, kPi, WpType::REVERSE),
            wp(0.5, 0.0, 0.0, WpType::FORWARD),
            wp(2.0, 0.0, 0.0, WpType::FORWARD)});
    // REVERSE theta=pi has increasing-s actual motion heading zero.
    const PairBridgeTtcCorrection cusp_result = evaluateBridgeTtcCorrection(
        consistent_cusp, opposing, forward_prediction, opposing_prediction,
        conflict(8.0, 1.5, 0.5), map, config);
    if (!cusp_result.a.bridge_related ||
        cusp_result.a.near_boundary_s > 1e-9 ||
        cusp_result.a.self_traversal_changes != 1 ||
        cusp_result.a.backtrack_end_reason !=
            BridgeBacktrackEndReason::PATH_START) {
        return fail("F/R cusp incorrectly terminated a persistent relation");
    }

    // If actual motion really ceases to oppose after crossing the cusp, the
    // direction predicate--not the type transition--ends the relation.
    const VehicleAgent direction_change_cusp = vehicle(
        8, {wp(0.0, 0.0, 0.0, WpType::REVERSE),
            wp(0.5, 0.0, 0.0, WpType::REVERSE),
            wp(0.5, 0.0, 0.0, WpType::FORWARD),
            wp(2.0, 0.0, 0.0, WpType::FORWARD)});
    const PairBridgeTtcCorrection direction_lost =
        evaluateBridgeTtcCorrection(
            direction_change_cusp, opposing, forward_prediction,
            opposing_prediction, conflict(8.0, 1.5, 0.5), map, config);
    if (!direction_lost.a.bridge_related ||
        direction_lost.a.self_traversal_changes != 1 ||
        direction_lost.a.backtrack_end_reason !=
            BridgeBacktrackEndReason::RELATION_DIRECTION_LOST) {
        return fail("cusp direction change did not end by direction predicate");
    }

    // The other-path cursor may cross its own adjacent traversal when that is
    // the locally nearest geometry. It is not forced by a self type change.
    const VehicleAgent other_cusp = vehicle(
        9, {wp(2.0, 0.04, kPi, WpType::FORWARD),
            wp(1.0, 0.04, kPi, WpType::FORWARD),
            wp(1.0, 0.04, 0.0, WpType::REVERSE),
            wp(0.0, 0.04, 0.0, WpType::REVERSE)});
    const PairBridgeTtcCorrection other_cusp_result =
        evaluateBridgeTtcCorrection(
            forward, other_cusp, forward_prediction, opposing_prediction,
            conflict(8.0, 1.6, 0.4), map, config);
    if (!other_cusp_result.a.bridge_related ||
        other_cusp_result.a.near_boundary_s > 1e-9 ||
        other_cusp_result.a.self_traversal_changes != 0 ||
        other_cusp_result.a.nearest_other_traversal_changes != 1) {
        return fail("local nearest cursor could not cross other-path cusp");
    }

    // A and B are evaluated independently. Here A's collision point matches
    // B's opposing section, while B's own collision point matches A with the
    // same motion heading.
    const VehicleAgent mixed_b = vehicle(
        7, {wp(0.0, 0.03, 0.0, WpType::FORWARD),
            wp(1.0, 0.03, kPi, WpType::FORWARD),
            wp(2.0, 0.03, kPi, WpType::FORWARD)});
    const PairBridgeTtcCorrection one_side = evaluateBridgeTtcCorrection(
        forward, mixed_b, forward_prediction, forward_prediction,
        conflict(5.0, 1.0, 0.0), map, config);
    if (!one_side.a.bridge_related || one_side.b.bridge_related) {
        return fail("per-vehicle bridge evaluation was not independent");
    }

    // original_ttc is independently inverted from each prediction and is not
    // copied from the pair-level first_overlap_t diagnostic.
    const PairBridgeTtcCorrection independent_original =
        evaluateBridgeTtcCorrection(
            forward, opposing, prediction(2.0, 10.0),
            prediction(2.0, 20.0), conflict(8.0, 1.2, 0.4),
            map, config);
    if (!near(independent_original.a.original_ttc, 6.0) ||
        !near(independent_original.b.original_ttc, 4.0) ||
        near(independent_original.a.original_ttc,
             independent_original.b.original_ttc)) {
        return fail("bridge original TTCs still share first_overlap_t");
    }

    std::cout << "bridge_ttc_correction_test: PASS\n";
    return 0;
}
