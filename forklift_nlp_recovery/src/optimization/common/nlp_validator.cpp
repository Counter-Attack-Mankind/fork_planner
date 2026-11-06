#include "forklift_nlp_recovery/nlp_validator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace forklift_nlp_recovery {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Pose2d bodyCenter(const Pose2d& rear, double offset) {
    return {rear.x + offset * std::cos(rear.yaw),
            rear.y + offset * std::sin(rear.yaw), rear.yaw};
}

double pointAabbViolation(const Pose2d& point, const Aabb& box) {
    return std::max({box.min_x - point.x, point.x - box.max_x,
                     box.min_y - point.y, point.y - box.max_y, 0.0});
}

struct NumericState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double speed = 0.0;
    double steer = 0.0;
};

struct NumericControl {
    double acceleration = 0.0;
    double steer_rate = 0.0;
};

struct Vector2 { double x = 0.0; double y = 0.0; };

struct Obb {
    Vector2 center;
    Vector2 longitudinal_axis;
    Vector2 lateral_axis;
    double half_length = 0.0;
    double half_width = 0.0;
};

double dot(const Vector2& first, const Vector2& second) {
    return first.x * second.x + first.y * second.y;
}

Obb vehicleObb(const Pose2d& rear_axle, double length, double width,
               double rear_axle_to_center) {
    const double cosine = std::cos(rear_axle.yaw);
    const double sine = std::sin(rear_axle.yaw);
    return {{rear_axle.x + rear_axle_to_center * cosine,
             rear_axle.y + rear_axle_to_center * sine},
            {cosine, sine}, {-sine, cosine}, 0.5 * length, 0.5 * width};
}

double projectionRadius(const Obb& box, const Vector2& axis) {
    return box.half_length * std::abs(dot(box.longitudinal_axis, axis)) +
           box.half_width * std::abs(dot(box.lateral_axis, axis));
}

double obbSatSeparationMargin(const Obb& first, const Obb& second) {
    const Vector2 delta{second.center.x - first.center.x,
                        second.center.y - first.center.y};
    const Vector2 axes[] = {first.longitudinal_axis, first.lateral_axis,
                            second.longitudinal_axis, second.lateral_axis};
    double margin = -kInfinity;
    for (const Vector2& axis : axes) {
        const double center_distance = std::abs(dot(delta, axis));
        const double extent = projectionRadius(first, axis) +
                              projectionRadius(second, axis);
        margin = std::max(margin, center_distance - extent);
    }
    return margin;
}

NumericState derivative(const NumericState& state, const NumericControl& control,
                        double wheel_base) {
    return {state.speed * std::cos(state.yaw), state.speed * std::sin(state.yaw),
            state.speed * std::tan(state.steer) / wheel_base,
            control.acceleration, control.steer_rate};
}

NumericState addScaled(const NumericState& state, const NumericState& delta,
                       double scale) {
    return {state.x + scale * delta.x, state.y + scale * delta.y,
            state.yaw + scale * delta.yaw, state.speed + scale * delta.speed,
            state.steer + scale * delta.steer};
}

NumericState integrateEuler(const NumericState& state,
                            const NumericControl& control, double duration,
                            double wheel_base) {
    return addScaled(state, derivative(state, control, wheel_base), duration);
}

}  // namespace

NlpSolutionValidation validateSolution(const NlpOptimizationInput& input,
                                       const NlpSolveResult& result,
                                       const NlpSolverConfig& config,
                                       const CorridorBuilder& corridor_builder) {
    NlpSolutionValidation validation;
    validation.minimum_participant_obb_separation_margin = kInfinity;
    validation.minimum_static_obb_separation_margin = kInfinity;
    if (result.vehicles.size() != input.vehicles.size() || result.node_count < 2 ||
        result.total_time <= 0.0) {
        validation.reason = "solution dimensions are incomplete";
        return validation;
    }
    const double dt = result.total_time / static_cast<double>(result.node_count - 1);
    const size_t substeps = std::max<size_t>(1, config.validation_substeps);
    const bool cooperative = input.vehicles.size() == 3;
    std::vector<std::vector<NumericState>> samples(result.vehicles.size());
    for (size_t index = 0; index < result.vehicles.size(); ++index) {
        const auto& vehicle = result.vehicles[index];
        const auto& source = input.vehicles[index];
        if (vehicle.id != source.id || vehicle.nodes.size() != result.node_count ||
            (cooperative ? !source.stc_valid : source.corridor.size() != result.node_count)) {
            validation.reason = "vehicle ordering or node count mismatch";
            return validation;
        }
        if (!(source.parameters.length > 0.0) || !(source.parameters.width > 0.0) ||
            !std::isfinite(source.parameters.length) || !std::isfinite(source.parameters.width) ||
            !std::isfinite(source.parameters.rear_axle_to_center)) {
            validation.reason = "participant vehicle has invalid OBB geometry";
            return validation;
        }
        const auto& first = vehicle.nodes.front();
        const auto& last = vehicle.nodes.back();
        validation.max_start_error = std::max(validation.max_start_error,
            std::hypot(first.rear_axle_pose.x - source.initial_state.rear_axle_pose.x,
                       first.rear_axle_pose.y - source.initial_state.rear_axle_pose.y));
        validation.max_start_error = std::max(validation.max_start_error,
            std::abs(normalizeAngle(first.rear_axle_pose.yaw - source.initial_state.rear_axle_pose.yaw)));
        validation.max_start_error = std::max(validation.max_start_error, std::abs(first.signed_speed));
        if (source.frozen_initial_state.steering_valid) {
            validation.max_start_error = std::max(validation.max_start_error,
                std::abs(first.front_steer - source.frozen_initial_state.steering_angle));
        }
        TerminalStateDiagnostics terminal;
        terminal.vehicle_id = source.id;
        terminal.specified_pose = source.terminal.pose;
        terminal.optimized_pose = last.rear_axle_pose;
        terminal.position_error = std::hypot(
            last.rear_axle_pose.x - source.terminal.pose.x,
            last.rear_axle_pose.y - source.terminal.pose.y);
        terminal.heading_error = std::abs(normalizeAngle(
            last.rear_axle_pose.yaw - source.terminal.pose.yaw));
        terminal.speed_error = std::abs(last.signed_speed);
        if (cooperative) {
            terminal.penalty = config.terminal_position_weight *
                                   terminal.position_error * terminal.position_error +
                               config.terminal_heading_weight *
                                   terminal.heading_error * terminal.heading_error;
        }
        validation.terminal_states.push_back(terminal);
        validation.max_terminal_position_error = std::max(
            validation.max_terminal_position_error, terminal.position_error);
        validation.max_terminal_heading_error = std::max(
            validation.max_terminal_heading_error, terminal.heading_error);
        validation.max_terminal_speed_error = std::max(
            validation.max_terminal_speed_error, terminal.speed_error);
        validation.max_terminal_error = std::max(
            {validation.max_terminal_error, terminal.position_error,
             terminal.heading_error, terminal.speed_error});

        for (size_t node = 0; node < result.node_count; ++node) {
            const auto& value = vehicle.nodes[node];
            validation.max_speed_violation = std::max(validation.max_speed_violation,
                std::max(0.0, std::abs(value.signed_speed) - source.parameters.max_speed));
            validation.max_steer_violation = std::max(validation.max_steer_violation,
                std::max(0.0, std::abs(value.front_steer) - source.parameters.max_steer_angle));
            if (node + 1 < result.node_count) {
                validation.max_acceleration_violation = std::max(validation.max_acceleration_violation,
                    std::max({0.0, value.acceleration - source.parameters.max_accel,
                              -source.parameters.max_decel - value.acceleration}));
                validation.max_steer_rate_violation = std::max(validation.max_steer_rate_violation,
                    std::max(0.0, std::abs(value.steer_rate) - source.parameters.max_steer_rate));
            }
            if (!cooperative && value.aabb_index != static_cast<int>(node)) {
                ++validation.aabb_violation_samples;
                validation.max_aabb_violation = kInfinity;
                continue;
            }
            const Aabb& box = cooperative ? source.safe_stc : source.corridor[node];
            const double violation = pointAabbViolation(value.disc_center, box);
            validation.max_aabb_violation = std::max(validation.max_aabb_violation, violation);
            if (violation > config.validation_tolerance) ++validation.aabb_violation_samples;
        }

        for (size_t interval = 0; interval + 1 < result.node_count; ++interval) {
            const auto& current = vehicle.nodes[interval];
            const auto& next = vehicle.nodes[interval + 1];
            const NumericState initial{current.rear_axle_pose.x, current.rear_axle_pose.y,
                                       current.rear_axle_pose.yaw, current.signed_speed,
                                       current.front_steer};
            const NumericControl control{current.acceleration, current.steer_rate};
            const NumericState predicted = integrateEuler(
                initial, control, dt, source.parameters.wheel_base);
            validation.max_kinematic_position_residual = std::max(
                validation.max_kinematic_position_residual,
                std::hypot(predicted.x - next.rear_axle_pose.x, predicted.y - next.rear_axle_pose.y));
            validation.max_kinematic_heading_residual = std::max(
                validation.max_kinematic_heading_residual,
                std::abs(normalizeAngle(predicted.yaw - next.rear_axle_pose.yaw)));
            validation.max_kinematic_speed_residual = std::max(
                validation.max_kinematic_speed_residual,
                std::abs(predicted.speed - next.signed_speed));
            validation.max_kinematic_steer_residual = std::max(
                validation.max_kinematic_steer_residual,
                std::abs(predicted.steer - next.front_steer));
            for (size_t substep = 0; substep < substeps; ++substep) {
                const double fraction = static_cast<double>(substep) / static_cast<double>(substeps);
                const NumericState state = integrateEuler(
                    initial, control, dt * fraction, source.parameters.wheel_base);
                samples[index].push_back(state);
                const Pose2d center = bodyCenter({state.x, state.y, state.yaw},
                                                 source.parameters.rear_axle_to_center);
                const Aabb& box = cooperative ? source.safe_stc : source.corridor[interval];
                const double stc_violation = pointAabbViolation(center, box);
                validation.max_aabb_violation = std::max(
                    validation.max_aabb_violation, stc_violation);
                if (stc_violation > config.validation_tolerance) {
                    ++validation.aabb_violation_samples;
                }
                if (!corridor_builder.pointLegal(center)) {
                    ++validation.static_map_violation_samples;
                    PointLegalityViolation detail;
                    corridor_builder.diagnosePointViolation(center, detail);
                    validation.static_map_violations.push_back({
                        vehicle.id, interval, substep,
                        (static_cast<double>(interval) + fraction) * dt,
                        center, detail.type, detail.shelf_index, detail.distance});
                }
            }
        }
        const auto& last_node = vehicle.nodes.back();
        samples[index].push_back({last_node.rear_axle_pose.x, last_node.rear_axle_pose.y,
                                  last_node.rear_axle_pose.yaw, last_node.signed_speed,
                                  last_node.front_steer});
        if (!corridor_builder.pointLegal(last_node.disc_center)) {
            ++validation.static_map_violation_samples;
            PointLegalityViolation detail;
            corridor_builder.diagnosePointViolation(last_node.disc_center, detail);
            validation.static_map_violations.push_back({vehicle.id,
                result.node_count - 2, substeps, result.total_time,
                last_node.disc_center, detail.type, detail.shelf_index,
                detail.distance});
        }
    }

    for (size_t first = 0; first < samples.size(); ++first) {
        for (size_t second = first + 1; second < samples.size(); ++second) {
            if (samples[first].size() != samples[second].size()) {
                validation.reason = "participant trajectories do not share one validation time grid";
                return validation;
            }
            for (size_t sample = 0; sample < samples[first].size(); ++sample) {
                const NumericState& first_state = samples[first][sample];
                const NumericState& second_state = samples[second][sample];
                const VehicleParameters& first_parameters = input.vehicles[first].parameters;
                const VehicleParameters& second_parameters = input.vehicles[second].parameters;
                const Obb first_box = vehicleObb(
                    {first_state.x, first_state.y, first_state.yaw},
                    first_parameters.length, first_parameters.width,
                    first_parameters.rear_axle_to_center);
                const Obb second_box = vehicleObb(
                    {second_state.x, second_state.y, second_state.yaw},
                    second_parameters.length, second_parameters.width,
                    second_parameters.rear_axle_to_center);
                const double margin = obbSatSeparationMargin(first_box, second_box);
                if (margin < validation.minimum_participant_obb_separation_margin) {
                    validation.minimum_participant_obb_separation_margin = margin;
                    validation.minimum_participant_vehicle_a_id = input.vehicles[first].id;
                    validation.minimum_participant_vehicle_b_id = input.vehicles[second].id;
                    validation.minimum_participant_separation_time =
                        static_cast<double>(sample) * dt / static_cast<double>(substeps);
                }
                if (margin < -config.validation_tolerance) {
                    ++validation.participant_obb_collision_samples;
                }
            }
        }
    }

    for (size_t vehicle = 0; vehicle < samples.size(); ++vehicle) {
        for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
            if (!(obstacle.length > 0.0) || !(obstacle.width > 0.0) ||
                !std::isfinite(obstacle.length) || !std::isfinite(obstacle.width) ||
                !std::isfinite(obstacle.rear_axle_to_center)) {
                validation.reason = "static vehicle has invalid OBB geometry";
                return validation;
            }
            const Obb static_box = vehicleObb(obstacle.rear_axle_pose, obstacle.length,
                obstacle.width, obstacle.rear_axle_to_center);
            for (const NumericState& state : samples[vehicle]) {
                const VehicleParameters& parameters = input.vehicles[vehicle].parameters;
                const Obb moving_box = vehicleObb({state.x, state.y, state.yaw},
                    parameters.length, parameters.width, parameters.rear_axle_to_center);
                const double margin = obbSatSeparationMargin(moving_box, static_box);
                validation.minimum_static_obb_separation_margin = std::min(
                    validation.minimum_static_obb_separation_margin, margin);
                if (margin < -config.validation_tolerance) ++validation.static_obb_collision_samples;
            }
        }
    }
    if (!std::isfinite(validation.minimum_participant_obb_separation_margin)) {
        validation.minimum_participant_obb_separation_margin = 0.0;
    }
    if (!std::isfinite(validation.minimum_static_obb_separation_margin)) {
        validation.minimum_static_obb_separation_margin = 0.0;
    }
    const double tolerance = config.validation_tolerance;
    const bool terminal_hard_constraints_passed = cooperative ?
        validation.max_terminal_speed_error <= tolerance :
        validation.max_terminal_error <= tolerance;
    validation.passed = validation.max_start_error <= tolerance &&
        terminal_hard_constraints_passed &&
        validation.max_kinematic_position_residual <= tolerance &&
        validation.max_kinematic_heading_residual <= tolerance &&
        validation.max_kinematic_speed_residual <= tolerance &&
        validation.max_kinematic_steer_residual <= tolerance &&
        validation.max_speed_violation <= tolerance && validation.max_steer_violation <= tolerance &&
        validation.max_acceleration_violation <= tolerance &&
        validation.max_steer_rate_violation <= tolerance &&
        validation.max_aabb_violation <= tolerance &&
        validation.participant_obb_collision_samples == 0 &&
        validation.static_obb_collision_samples == 0 &&
        validation.aabb_violation_samples == 0 &&
        validation.static_map_violation_samples == 0;
    if (validation.passed) {
        validation.reason = cooperative ?
            "all hard constraints and safety checks passed; cooperative terminal pose errors are soft metrics" :
            "all terminal, STC, map, static OBB and synchronized participant OBB SAT checks passed";
    } else {
        validation.reason = cooperative ?
            "one or more hard dynamics, terminal speed, STC, map or OBB safety checks failed" :
            "one or more terminal, dynamics, STC, map or OBB safety checks failed";
    }
    return validation;
}

}  // namespace forklift_nlp_recovery
