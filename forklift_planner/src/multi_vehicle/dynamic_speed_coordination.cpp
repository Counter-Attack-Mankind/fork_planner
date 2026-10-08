#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <algorithm>
#include <cmath>

#include "forklift_planner/multi_vehicle/bridge_ttc_correction.h"
#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

DynamicInterventionBand classifyDynamicInterventionBand(
    double vehicle_ttc, const MultiVehicleConfig& config) {
    if (vehicle_ttc >= config.dynamic_speed_far_threshold) {
        return DynamicInterventionBand::FAR;
    }
    if (vehicle_ttc >= config.dynamic_speed_near_threshold) {
        return DynamicInterventionBand::MID;
    }
    return DynamicInterventionBand::NEAR;
}

const char* dynamicInterventionBandName(DynamicInterventionBand band) {
    switch (band) {
        case DynamicInterventionBand::NEAR: return "NEAR";
        case DynamicInterventionBand::MID: return "MID";
        case DynamicInterventionBand::FAR: return "FAR";
    }
    return "UNKNOWN";
}

PriorityPhysicalTtcEvaluation evaluatePriorityPhysicalTtc(
    const VehicleAgent& priority, const VehicleAgent& other,
    const std::vector<PredictedKinematicSample>& priority_prediction,
    const MapParam& map_param, const MultiVehicleConfig& config) {
    PriorityPhysicalTtcEvaluation result;
    if (!priority.active() || !other.active() || priority.track.empty() ||
        other.track.empty() || priority_prediction.empty()) {
        return result;
    }
    RoughWp other_pose = other.track.poseAtS(std::max(
        0.0, std::min(other.path_s, other.track.length())));
    if (other.real_pose_valid) {
        other_pose.x = other.real_x;
        other_pose.y = other.real_y;
        other_pose.theta = other.real_yaw;
    }
    const OBB other_current_body = makeBody(other_pose, map_param, 0.0);
    for (const PredictedKinematicSample& sample : priority_prediction) {
        if (!overlaps(sample.body, other_current_body)) continue;
        result.valid = true;
        result.collision_t = sample.t;
        result.collision_s = sample.s;
        result.safety_ttc = sample.t;
        result.safety_boundary_s = sample.s;
        const VehicleBridgeTtcCorrection bridge =
            evaluateVehicleBridgeTtcCorrection(
                priority, other, priority_prediction, sample.s,
                other.path_s, sample.t, map_param, config);
        if (bridge.bridge_related) {
            result.bridge_related = true;
            result.safety_ttc = bridge.corrected_ttc;
            result.safety_boundary_s = bridge.near_boundary_s;
        }
        break;
    }
    return result;
}

VehicleAction selectRollingSpeedAction(
    DynamicInterventionBand band, bool emergency_stop) {
    if (emergency_stop) return VehicleAction::STOP;
    switch (band) {
        case DynamicInterventionBand::FAR:
            return VehicleAction::NOMINAL;
        case DynamicInterventionBand::MID:
            return VehicleAction::YIELD;
        case DynamicInterventionBand::NEAR:
            return VehicleAction::CREEP;
    }
    return VehicleAction::STOP;
}

SelectedSpeedActionEvaluation evaluateSelectedAction(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, VehicleAction action_a,
    VehicleAction action_b,
    std::optional<double> original_first_overlap_t) {
    SelectedSpeedActionEvaluation evaluation;
    evaluation.action_a = action_a;
    evaluation.action_b = action_b;
    const auto prediction_a = predictTrajectory(
        vehicle_a, map_param, config, action_a, prediction_horizon);
    const auto prediction_b = predictTrajectory(
        vehicle_b, map_param, config, action_b, prediction_horizon);
    // Candidate validation deliberately uses the same synchronized physical
    // OBB detector as the nominal baseline. Interaction labels are not a
    // separate control authority.
    const PairInteractionResult interaction =
        detectPairInteractionFromPredictions(
            vehicle_a, vehicle_b, potential_zones,
            prediction_a, prediction_b);
    evaluation.conflict_free = !interaction.event.valid;
    if (interaction.event.valid) {
        const PairBridgeTtcCorrection bridge = evaluateBridgeTtcCorrection(
            vehicle_a, vehicle_b, prediction_a, prediction_b, interaction,
            map_param, config);
        evaluation.first_overlap_t = interaction.event.first_overlap_t;
        evaluation.collision_s_a = interaction.event.collision_s_a;
        evaluation.collision_s_b = interaction.event.collision_s_b;
        evaluation.danger_s_a = bridge.a.near_boundary_s;
        evaluation.danger_s_b = bridge.b.near_boundary_s;
        evaluation.ttc_a = bridge.a.corrected_ttc;
        evaluation.ttc_b = bridge.b.corrected_ttc;
        if (original_first_overlap_t) {
            evaluation.conflict_delay =
                interaction.event.first_overlap_t -
                *original_first_overlap_t;
        }
    }
    return evaluation;
}

PairSpeedCoordinationResult evaluatePairSpeedCoordination(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const PairInteractionResult& nominal_baseline,
    const PriorityPhysicalTtcEvaluation& priority_physical,
    const MultiVehicleConfig& config, int preferred_winner_id) {
    PairSpeedCoordinationResult result;
    if (!nominal_baseline.event.valid) {
        result.reason = "baseline_clear";
        return result;
    }
    result.first_overlap_t = nominal_baseline.event.first_overlap_t;
    result.effective_ttc_a = nominal_baseline.event.ttc_a;
    result.effective_ttc_b = nominal_baseline.event.ttc_b;
    if (preferred_winner_id != vehicle_a.id &&
        preferred_winner_id != vehicle_b.id) {
        result.reason = "no_priority_winner";
        return result;
    }

    result.attempted = true;
    result.action_selected = true;
    result.selected_winner_id = preferred_winner_id;
    const bool a_is_priority = preferred_winner_id == vehicle_a.id;
    result.yielding_effective_ttc = a_is_priority
        ? nominal_baseline.event.ttc_b : nominal_baseline.event.ttc_a;
    if (priority_physical.valid) {
        result.priority_physical_ttc = priority_physical.safety_ttc;
        result.priority_physical_bridge = priority_physical.bridge_related;
        result.priority_physical_collision_s = priority_physical.collision_s;
        result.priority_physical_boundary_s =
            priority_physical.safety_boundary_s;
        const TtcStopBoundary priority_boundary = evaluateTtcStopBoundary(
            priority_physical.safety_ttc, VehicleAction::NOMINAL, config);
        result.priority_stop_threshold = priority_boundary.stop_threshold;
        result.priority_safety_stop = priority_boundary.stop_required;
    }

    result.yielding_band = classifyDynamicInterventionBand(
        *result.yielding_effective_ttc, config);
    VehicleAction yielding_action = selectRollingSpeedAction(
        result.yielding_band, false);
    const TtcStopBoundary baseline_yielding_boundary =
        evaluateTtcStopBoundary(
            *result.yielding_effective_ttc, yielding_action, config);
    result.yielding_stop_threshold =
        baseline_yielding_boundary.stop_threshold;
    if (baseline_yielding_boundary.stop_required) {
        result.yielding_safety_stop = true;
        yielding_action = VehicleAction::STOP;
    }
    const VehicleAction priority_action = result.priority_safety_stop
        ? VehicleAction::STOP : VehicleAction::NOMINAL;
    result.selected_action_a = a_is_priority
        ? priority_action : yielding_action;
    result.selected_action_b = a_is_priority
        ? yielding_action : priority_action;

    result.emergency_stop = result.yielding_safety_stop ||
                            result.priority_safety_stop;
    if (result.emergency_stop) {
        result.reason = "rolling_emergency_stop";
    } else if (result.yielding_band == DynamicInterventionBand::FAR) {
        result.reason = "rolling_far_nominal";
    } else if (result.yielding_band == DynamicInterventionBand::MID) {
        result.reason = "rolling_mid_yield";
    } else {
        result.reason = "rolling_near_creep";
    }
    return result;
}

TtcStopBoundary evaluateTtcStopBoundary(
    double vehicle_ttc, VehicleAction planned_action,
    const MultiVehicleConfig& config) {
    TtcStopBoundary result;
    result.planned_action = planned_action;
    switch (planned_action) {
        case VehicleAction::STOP:
            result.action_speed = 0.0;
            break;
        case VehicleAction::CREEP:
            result.action_speed = config.nominal_speed * config.creep_ratio;
            break;
        case VehicleAction::YIELD:
            result.action_speed = config.nominal_speed * config.yield_ratio;
            break;
        case VehicleAction::NOMINAL:
            result.action_speed = config.nominal_speed;
            break;
        case VehicleAction::BOOST:
            result.action_speed = config.enable_boost
                ? std::min(config.max_speed,
                           config.nominal_speed * config.boost_ratio)
                : config.nominal_speed;
            break;
    }
    result.action_speed = std::max(0.0, result.action_speed);
    result.decision_period = std::max(0.0, config.rolling_refresh_period);
    result.braking_time =
        result.action_speed / std::max(1e-6, config.max_decel);
    result.time_margin = std::max(0.0, config.dynamic_stop_time_margin);
    result.stop_threshold = result.decision_period + result.braking_time +
                            result.time_margin;
    result.stop_required =
        vehicle_ttc <= result.stop_threshold + 1e-9;
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
