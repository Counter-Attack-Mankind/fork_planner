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

NumericState integrateRk4(const NumericState& state,
                          const NumericControl& control, double duration,
                          double wheel_base) {
    const NumericState k1 = derivative(state, control, wheel_base);
    const NumericState k2 = derivative(addScaled(state, k1, duration * 0.5), control, wheel_base);
    const NumericState k3 = derivative(addScaled(state, k2, duration * 0.5), control, wheel_base);
    const NumericState k4 = derivative(addScaled(state, k3, duration), control, wheel_base);
    NumericState result = state;
    result.x += duration * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0;
    result.y += duration * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y) / 6.0;
    result.yaw += duration * (k1.yaw + 2.0 * k2.yaw + 2.0 * k3.yaw + k4.yaw) / 6.0;
    result.speed += duration * (k1.speed + 2.0 * k2.speed + 2.0 * k3.speed + k4.speed) / 6.0;
    result.steer += duration * (k1.steer + 2.0 * k2.steer + 2.0 * k3.steer + k4.steer) / 6.0;
    return result;
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
    std::vector<std::vector<NumericState>> samples(result.vehicles.size());
    for (size_t index = 0; index < result.vehicles.size(); ++index) {
        const auto& vehicle = result.vehicles[index];
        const auto& source = input.vehicles[index];
        if (vehicle.id != source.id || vehicle.nodes.size() != result.node_count ||
            source.corridor.size() != result.node_count) {
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
        validation.max_terminal_error = std::max(validation.max_terminal_error,
            std::hypot(last.rear_axle_pose.x - source.terminal.pose.x,
                       last.rear_axle_pose.y - source.terminal.pose.y));
        validation.max_terminal_error = std::max(validation.max_terminal_error,
            std::abs(normalizeAngle(last.rear_axle_pose.yaw - source.terminal.pose.yaw)));
        validation.max_terminal_error = std::max(validation.max_terminal_error, std::abs(last.signed_speed));

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
            if (value.aabb_index != static_cast<int>(node)) {
                ++validation.aabb_violation_samples;
                validation.max_aabb_violation = kInfinity;
                continue;
            }
            const double violation = pointAabbViolation(value.disc_center, source.corridor[node]);
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
            const NumericState predicted = integrateRk4(initial, control, dt, source.parameters.wheel_base);
            validation.max_kinematic_position_residual = std::max(
                validation.max_kinematic_position_residual,
                std::hypot(predicted.x - next.rear_axle_pose.x, predicted.y - next.rear_axle_pose.y));
            validation.max_kinematic_heading_residual = std::max(
                validation.max_kinematic_heading_residual,
                std::abs(normalizeAngle(predicted.yaw - next.rear_axle_pose.yaw)));
            for (size_t substep = 0; substep < substeps; ++substep) {
                const double fraction = static_cast<double>(substep) / static_cast<double>(substeps);
                const NumericState state = integrateRk4(initial, control, dt * fraction,
                                                         source.parameters.wheel_base);
                samples[index].push_back(state);
                const Pose2d center = bodyCenter({state.x, state.y, state.yaw},
                                                 source.parameters.rear_axle_to_center);
                if (!corridor_builder.pointLegal(center)) {
                    ++validation.static_map_violation_samples;
                }
            }
        }
        const auto& last_node = vehicle.nodes.back();
        samples[index].push_back({last_node.rear_axle_pose.x, last_node.rear_axle_pose.y,
                                  last_node.rear_axle_pose.yaw, last_node.signed_speed,
                                  last_node.front_steer});
    }

    const size_t sample_count = samples.front().size();
    for (size_t first = 0; first < samples.size(); ++first) {
        for (size_t second = first + 1; second < samples.size(); ++second) {
            for (size_t sample = 0; sample < sample_count; ++sample) {
                const NumericState& first_state = samples[first][sample];
                const NumericState& second_state = samples[second][sample];
                const VehicleParameters& first_parameters = input.vehicles[first].parameters;
                const VehicleParameters& second_parameters = input.vehicles[second].parameters;
                const Obb first_box = vehicleObb({first_state.x, first_state.y, first_state.yaw},
                    first_parameters.length, first_parameters.width,
                    first_parameters.rear_axle_to_center);
                const Obb second_box = vehicleObb({second_state.x, second_state.y, second_state.yaw},
                    second_parameters.length, second_parameters.width,
                    second_parameters.rear_axle_to_center);
                const double margin = obbSatSeparationMargin(first_box, second_box);
                validation.minimum_participant_obb_separation_margin = std::min(
                    validation.minimum_participant_obb_separation_margin, margin);
                if (margin < -config.validation_tolerance) ++validation.participant_obb_collision_samples;
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
    validation.passed = validation.max_start_error <= tolerance &&
        validation.max_terminal_error <= tolerance &&
        validation.max_kinematic_position_residual <= tolerance &&
        validation.max_kinematic_heading_residual <= tolerance &&
        validation.max_speed_violation <= tolerance && validation.max_steer_violation <= tolerance &&
        validation.max_acceleration_violation <= tolerance &&
        validation.max_steer_rate_violation <= tolerance &&
        validation.max_aabb_violation <= tolerance &&
        validation.participant_obb_collision_samples == 0 &&
        validation.static_obb_collision_samples == 0 &&
        validation.aabb_violation_samples == 0 &&
        validation.static_map_violation_samples == 0;
    validation.reason = validation.passed
        ? "all node AABB and sampled static-map checks passed (collision geometry: OBB SAT)"
        : "one or more independent safety checks failed (collision geometry: OBB SAT)";
    return validation;
}

}  // namespace forklift_nlp_recovery
