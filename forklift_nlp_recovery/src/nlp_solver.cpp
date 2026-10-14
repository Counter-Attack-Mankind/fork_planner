#include "forklift_nlp_recovery/nlp_solver.h"

#include <casadi/casadi.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace forklift_nlp_recovery {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double equivalentAngle(double angle, double reference) {
    return reference + normalizeAngle(angle - reference);
}

Pose2d bodyCenter(const Pose2d& rear, double offset) {
    return {rear.x + offset * std::cos(rear.yaw),
            rear.y + offset * std::sin(rear.yaw), rear.yaw};
}

bool boxesOverlap(const Aabb& first, const Aabb& second, double tolerance) {
    return std::min(first.max_x, second.max_x) + tolerance >=
               std::max(first.min_x, second.min_x) &&
           std::min(first.max_y, second.max_y) + tolerance >=
               std::max(first.min_y, second.min_y);
}

double pointAabbViolation(const Pose2d& point, const Aabb& box) {
    return std::max({box.min_x - point.x, point.x - box.max_x,
                     box.min_y - point.y, point.y - box.max_y, 0.0});
}

double pointAabbUnionViolation(const Pose2d& point, const Aabb& first,
                               const Aabb& second) {
    return std::min(pointAabbViolation(point, first),
                    pointAabbViolation(point, second));
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

struct Vector2 {
    double x = 0.0;
    double y = 0.0;
};

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

// Positive means separated on at least one SAT axis. Negative means overlap;
// its magnitude is the minimum translation distance on the tested OBB axes.
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
    return {state.speed * std::cos(state.yaw),
            state.speed * std::sin(state.yaw),
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
    const NumericState k2 = derivative(addScaled(state, k1, duration * 0.5),
                                       control, wheel_base);
    const NumericState k3 = derivative(addScaled(state, k2, duration * 0.5),
                                       control, wheel_base);
    const NumericState k4 = derivative(addScaled(state, k3, duration),
                                       control, wheel_base);
    NumericState result = state;
    result.x += duration * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0;
    result.y += duration * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y) / 6.0;
    result.yaw += duration * (k1.yaw + 2.0 * k2.yaw + 2.0 * k3.yaw + k4.yaw) / 6.0;
    result.speed += duration * (k1.speed + 2.0 * k2.speed + 2.0 * k3.speed + k4.speed) / 6.0;
    result.steer += duration * (k1.steer + 2.0 * k2.steer + 2.0 * k3.steer + k4.steer) / 6.0;
    return result;
}

casadi::MX derivative(const casadi::MX& state, const casadi::MX& control,
                      double wheel_base) {
    return casadi::MX::vertcat({state(3) * cos(state(2)),
                               state(3) * sin(state(2)),
                               state(3) * tan(state(4)) / wheel_base,
                               control(0), control(1)});
}

casadi::MX integrateRk4(const casadi::MX& state, const casadi::MX& control,
                        const casadi::MX& duration, double wheel_base) {
    const casadi::MX k1 = derivative(state, control, wheel_base);
    const casadi::MX k2 = derivative(state + duration * 0.5 * k1, control,
                                     wheel_base);
    const casadi::MX k3 = derivative(state + duration * 0.5 * k2, control,
                                     wheel_base);
    const casadi::MX k4 = derivative(state + duration * k3, control, wheel_base);
    return state + duration * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
}

casadi::MX symbolicCenter(const casadi::MX& state, double offset) {
    return casadi::MX::vertcat({state(0) + offset * cos(state(2)),
                               state(1) + offset * sin(state(2))});
}

const VehicleInitialGuess* findGuess(const NlpInitialGuess& guess, int id) {
    for (const VehicleInitialGuess& vehicle : guess.vehicles) {
        if (vehicle.id == id) return &vehicle;
    }
    return nullptr;
}

std::vector<double> unwrappedHeadings(const VehicleInitialGuess& guess,
                                      double terminal_heading) {
    std::vector<double> headings(guess.nodes.size(), 0.0);
    headings.front() = guess.nodes.front().rear_axle_pose.yaw;
    for (size_t node = 1; node < headings.size(); ++node) {
        headings[node] = equivalentAngle(
            guess.nodes[node].rear_axle_pose.yaw, headings[node - 1]);
    }
    headings.back() = equivalentAngle(terminal_heading, headings.back());
    return headings;
}

casadi::MX squaredDistance(const casadi::MX& first, const casadi::MX& second) {
    const casadi::MX dx = first(0) - second(0);
    const casadi::MX dy = first(1) - second(1);
    return dx * dx + dy * dy;
}

std::string yamlQuote(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 2);
    for (char character : value) {
        if (character == '\\' || character == '"') escaped.push_back('\\');
        if (character == '\n' || character == '\r') {
            escaped += " ";
        } else {
            escaped.push_back(character);
        }
    }
    return '"' + escaped + '"';
}

struct SymbolicVehicle {
    casadi::MX state;
    casadi::MX control;
    const NlpVehicleInput* input = nullptr;
    const VehicleInitialGuess* guess = nullptr;
};

struct ConstraintData {
    std::vector<casadi::MX> expressions;
    std::vector<double> lower;
    std::vector<double> upper;

    void add(const casadi::MX& expression, double minimum, double maximum) {
        expressions.push_back(expression);
        lower.push_back(minimum);
        upper.push_back(maximum);
    }
};

NlpSolutionValidation validateSolution(
    const NlpOptimizationInput& input, const NlpSolveResult& result,
    const NlpSolverConfig& config) {
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
        if (vehicle.id != source.id || vehicle.nodes.size() != result.node_count) {
            validation.reason = "vehicle ordering or node count mismatch";
            return validation;
        }
        if (!(source.parameters.length > 0.0) ||
            !(source.parameters.width > 0.0) ||
            !std::isfinite(source.parameters.length) ||
            !std::isfinite(source.parameters.width) ||
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
            std::abs(normalizeAngle(first.rear_axle_pose.yaw -
                                    source.initial_state.rear_axle_pose.yaw)));
        validation.max_start_error = std::max(validation.max_start_error,
                                              std::abs(first.signed_speed));
        validation.max_terminal_error = std::max(validation.max_terminal_error,
            std::hypot(last.rear_axle_pose.x - source.terminal.pose.x,
                       last.rear_axle_pose.y - source.terminal.pose.y));
        validation.max_terminal_error = std::max(validation.max_terminal_error,
            std::abs(normalizeAngle(last.rear_axle_pose.yaw - source.terminal.pose.yaw)));
        validation.max_terminal_error = std::max(validation.max_terminal_error,
                                                 std::abs(last.signed_speed));

        for (size_t node = 0; node < result.node_count; ++node) {
            const auto& value = vehicle.nodes[node];
            validation.max_speed_violation = std::max(validation.max_speed_violation,
                std::max(0.0, std::abs(value.signed_speed) - source.parameters.max_speed));
            validation.max_steer_violation = std::max(validation.max_steer_violation,
                std::max(0.0, std::abs(value.front_steer) - source.parameters.max_steer_angle));
            if (node + 1 < result.node_count) {
                validation.max_acceleration_violation = std::max(
                    validation.max_acceleration_violation,
                    std::max({0.0, value.acceleration - source.parameters.max_accel,
                              -source.parameters.max_decel - value.acceleration}));
                validation.max_steer_rate_violation = std::max(
                    validation.max_steer_rate_violation,
                    std::max(0.0, std::abs(value.steer_rate) -
                                      source.parameters.max_steer_rate));
            }
            if (value.aabb_index < 0 ||
                static_cast<size_t>(value.aabb_index) >= source.corridor.size()) {
                ++validation.aabb_violation_samples;
                validation.max_aabb_violation = kInfinity;
                continue;
            }
            if (node == 0 && vehicle.start_aabb_exempt) continue;
            const double violation = pointAabbViolation(
                value.disc_center, source.corridor[value.aabb_index]);
            validation.max_aabb_violation = std::max(validation.max_aabb_violation,
                                                     violation);
            if (violation > config.validation_tolerance) {
                ++validation.aabb_violation_samples;
            }
        }

        for (size_t interval = 0; interval + 1 < result.node_count; ++interval) {
            const auto& current = vehicle.nodes[interval];
            const auto& next = vehicle.nodes[interval + 1];
            if (current.aabb_index < 0 || next.aabb_index < 0 ||
                static_cast<size_t>(current.aabb_index) >= source.corridor.size() ||
                static_cast<size_t>(next.aabb_index) >= source.corridor.size()) continue;
            const Aabb& current_box = source.corridor[current.aabb_index];
            const Aabb& next_box = source.corridor[next.aabb_index];
            if (!boxesOverlap(current_box, next_box, config.validation_tolerance)) {
                ++validation.invalid_aabb_switches;
            }
            const NumericState initial{current.rear_axle_pose.x,
                                       current.rear_axle_pose.y,
                                       current.rear_axle_pose.yaw,
                                       current.signed_speed,
                                       current.front_steer};
            const NumericControl control{current.acceleration, current.steer_rate};
            const NumericState predicted = integrateRk4(
                initial, control, dt, source.parameters.wheel_base);
            validation.max_kinematic_position_residual = std::max(
                validation.max_kinematic_position_residual,
                std::hypot(predicted.x - next.rear_axle_pose.x,
                           predicted.y - next.rear_axle_pose.y));
            validation.max_kinematic_heading_residual = std::max(
                validation.max_kinematic_heading_residual,
                std::abs(normalizeAngle(predicted.yaw - next.rear_axle_pose.yaw)));
            for (size_t substep = 0; substep < substeps; ++substep) {
                const double fraction = static_cast<double>(substep) /
                                        static_cast<double>(substeps);
                const NumericState state = integrateRk4(
                    initial, control, dt * fraction, source.parameters.wheel_base);
                samples[index].push_back(state);
                const Pose2d center = bodyCenter(
                    {state.x, state.y, state.yaw}, source.parameters.rear_axle_to_center);
                if (!(interval == 0 && vehicle.start_aabb_exempt)) {
                    const double violation = pointAabbUnionViolation(
                        center, current_box, next_box);
                    validation.max_aabb_violation = std::max(
                        validation.max_aabb_violation, violation);
                    if (violation > config.validation_tolerance) {
                        ++validation.aabb_violation_samples;
                    }
                }
            }
        }
        const auto& last_node = vehicle.nodes.back();
        samples[index].push_back({last_node.rear_axle_pose.x,
                                  last_node.rear_axle_pose.y,
                                  last_node.rear_axle_pose.yaw,
                                  last_node.signed_speed,
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
                const Obb first_box = vehicleObb(
                    {first_state.x, first_state.y, first_state.yaw},
                    first_parameters.length, first_parameters.width,
                    first_parameters.rear_axle_to_center);
                const Obb second_box = vehicleObb(
                    {second_state.x, second_state.y, second_state.yaw},
                    second_parameters.length, second_parameters.width,
                    second_parameters.rear_axle_to_center);
                const double margin = obbSatSeparationMargin(first_box, second_box);
                validation.minimum_participant_obb_separation_margin = std::min(
                    validation.minimum_participant_obb_separation_margin, margin);
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
            const Obb static_box = vehicleObb(
                obstacle.rear_axle_pose, obstacle.length, obstacle.width,
                obstacle.rear_axle_to_center);
            for (const NumericState& state : samples[vehicle]) {
                const VehicleParameters& parameters = input.vehicles[vehicle].parameters;
                const Obb moving_box = vehicleObb(
                    {state.x, state.y, state.yaw}, parameters.length,
                    parameters.width, parameters.rear_axle_to_center);
                const double margin = obbSatSeparationMargin(moving_box, static_box);
                validation.minimum_static_obb_separation_margin = std::min(
                    validation.minimum_static_obb_separation_margin, margin);
                if (margin < -config.validation_tolerance) {
                    ++validation.static_obb_collision_samples;
                }
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
        validation.max_speed_violation <= tolerance &&
        validation.max_steer_violation <= tolerance &&
        validation.max_acceleration_violation <= tolerance &&
        validation.max_steer_rate_violation <= tolerance &&
        validation.max_aabb_violation <= tolerance &&
        validation.participant_obb_collision_samples == 0 &&
        validation.static_obb_collision_samples == 0 &&
        validation.aabb_violation_samples == 0 &&
        validation.invalid_aabb_switches == 0;
    validation.reason = validation.passed ?
        "all discrete and continuous checks passed (collision geometry: OBB SAT)" :
        "one or more independent safety checks failed (collision geometry: OBB SAT)";
    return validation;
}

double maximumViolation(const std::vector<double>& values,
                        const std::vector<double>& lower,
                        const std::vector<double>& upper) {
    double maximum = 0.0;
    for (size_t index = 0; index < values.size(); ++index) {
        if (std::isfinite(lower[index])) maximum = std::max(maximum, lower[index] - values[index]);
        if (std::isfinite(upper[index])) maximum = std::max(maximum, values[index] - upper[index]);
    }
    return std::max(0.0, maximum);
}

}  // namespace

NlpSolver::NlpSolver(const NlpSolverConfig& config) : config_(config) {}

NlpSolveResult NlpSolver::solve(const NlpOptimizationInput& input,
                                const NlpInitialGuess& initial_guess) const {
    NlpSolveResult result;
    result.statistics.casadi_version = casadi::CasadiMeta::version();
    if (!input.valid || !initial_guess.generated) {
        result.reason = "valid optimization input and generated initial guess are required";
        return result;
    }
    if (input.vehicles.empty() || initial_guess.node_count < 2 ||
        initial_guess.total_time <= 0.0) {
        result.reason = "empty vehicle set or invalid common time grid";
        return result;
    }
    if (config_.min_time_scale <= 0.0 ||
        config_.max_time_scale < config_.min_time_scale ||
        config_.validation_substeps == 0) {
        result.reason = "invalid solver configuration";
        return result;
    }

    const size_t nodes = initial_guess.node_count;
    std::vector<casadi::MX> variables;
    std::vector<double> initial_values;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    ConstraintData constraints;
    std::vector<SymbolicVehicle> symbolic_vehicles;
    const casadi::MX total_time = casadi::MX::sym("T");
    variables.push_back(total_time);
    initial_values.push_back(initial_guess.total_time);
    lower_bounds.push_back(std::max(1e-3, initial_guess.total_time * config_.min_time_scale));
    upper_bounds.push_back(std::max(lower_bounds.back() + 1e-3,
                                    initial_guess.total_time * config_.max_time_scale));
    const casadi::MX dt = total_time / static_cast<double>(nodes - 1);
    casadi::MX objective = config_.time_weight * total_time;

    for (size_t index = 0; index < input.vehicles.size(); ++index) {
        const NlpVehicleInput& vehicle = input.vehicles[index];
        const VehicleInitialGuess* guess = findGuess(initial_guess, vehicle.id);
        if (guess == nullptr || guess->nodes.size() != nodes ||
            vehicle.parameters.wheel_base <= 0.0 ||
            vehicle.parameters.disc_radius <= 0.0 ||
            vehicle.parameters.max_speed <= 0.0 ||
            vehicle.parameters.max_accel <= 0.0 ||
            vehicle.parameters.max_decel <= 0.0 ||
            vehicle.parameters.max_steer_angle <= 0.0 ||
            vehicle.parameters.max_steer_rate <= 0.0) {
            result.reason = "vehicle input or initial guess is incomplete for V" +
                            std::to_string(vehicle.id);
            return result;
        }
        if (!vehicle.corridor_valid || vehicle.corridor.empty() ||
            !guess->aabb_assignment_valid) {
            result.reason = "fixed AABB assignment is unavailable for V" +
                            std::to_string(vehicle.id);
            return result;
        }
        const std::vector<double> headings = unwrappedHeadings(*guess,
                                                               vehicle.terminal.pose.yaw);
        casadi::MX state = casadi::MX::sym("X_" + std::to_string(vehicle.id), 5,
                                           static_cast<casadi_int>(nodes));
        casadi::MX control = casadi::MX::sym("U_" + std::to_string(vehicle.id), 2,
                                             static_cast<casadi_int>(nodes - 1));
        variables.push_back(casadi::MX::reshape(state, 5 * nodes, 1));
        variables.push_back(casadi::MX::reshape(control, 2 * (nodes - 1), 1));

        for (size_t node = 0; node < nodes; ++node) {
            const InitialGuessNode& value = guess->nodes[node];
            const double state_initial[5] = {value.rear_axle_pose.x,
                                             value.rear_axle_pose.y,
                                             headings[node], value.signed_speed,
                                             value.front_steer};
            for (size_t component = 0; component < 5; ++component) {
                initial_values.push_back(state_initial[component]);
                lower_bounds.push_back(-kInfinity);
                upper_bounds.push_back(kInfinity);
            }
            const size_t base = lower_bounds.size() - 5;
            lower_bounds[base + 3] = -vehicle.parameters.max_speed;
            upper_bounds[base + 3] = vehicle.parameters.max_speed;
            lower_bounds[base + 4] = -vehicle.parameters.max_steer_angle;
            upper_bounds[base + 4] = vehicle.parameters.max_steer_angle;
            if (node == 0) {
                const Pose2d& start = vehicle.initial_state.rear_axle_pose;
                lower_bounds[base] = upper_bounds[base] = start.x;
                lower_bounds[base + 1] = upper_bounds[base + 1] = start.y;
                lower_bounds[base + 2] = upper_bounds[base + 2] = start.yaw;
                lower_bounds[base + 3] = upper_bounds[base + 3] = 0.0;
            } else if (node + 1 == nodes) {
                lower_bounds[base] = upper_bounds[base] = vehicle.terminal.pose.x;
                lower_bounds[base + 1] = upper_bounds[base + 1] = vehicle.terminal.pose.y;
                lower_bounds[base + 2] = upper_bounds[base + 2] = headings.back();
                lower_bounds[base + 3] = upper_bounds[base + 3] = 0.0;
            }
        }
        for (size_t interval = 0; interval + 1 < nodes; ++interval) {
            const InitialGuessNode& value = guess->nodes[interval];
            initial_values.push_back(value.acceleration);
            initial_values.push_back(value.steer_rate);
            lower_bounds.push_back(-vehicle.parameters.max_decel);
            upper_bounds.push_back(vehicle.parameters.max_accel);
            lower_bounds.push_back(-vehicle.parameters.max_steer_rate);
            upper_bounds.push_back(vehicle.parameters.max_steer_rate);
        }

        for (size_t node = 0; node < nodes; ++node) {
            const int box_index = guess->nodes[node].aabb_index;
            if (box_index < 0 || static_cast<size_t>(box_index) >= vehicle.corridor.size()) {
                result.reason = "invalid AABB index at V" + std::to_string(vehicle.id) +
                                " node " + std::to_string(node);
                return result;
            }
            const Aabb& box = vehicle.corridor[box_index];
            const casadi::MX center = symbolicCenter(state(casadi::Slice(), node),
                vehicle.parameters.rear_axle_to_center);
            if (!(node == 0 && guess->start_aabb_exempt)) {
                constraints.add(center(0), box.min_x, box.max_x);
                constraints.add(center(1), box.min_y, box.max_y);
            }
            if (node + 1 == nodes) continue;
            const casadi::MX current_state = state(casadi::Slice(), node);
            const casadi::MX current_control = control(casadi::Slice(), node);
            const casadi::MX predicted = integrateRk4(current_state, current_control,
                                                       dt, vehicle.parameters.wheel_base);
            for (size_t component = 0; component < 5; ++component) {
                constraints.add(state(component, node + 1) - predicted(component),
                                0.0, 0.0);
            }
            const int next_index = guess->nodes[node + 1].aabb_index;
            const Aabb& next_box = vehicle.corridor[next_index];
            if (!boxesOverlap(box, next_box, config_.validation_tolerance)) {
                result.reason = "disconnected fixed AABB switch at V" +
                    std::to_string(vehicle.id) + " node " + std::to_string(node);
                return result;
            }
            const casadi::MX midpoint = integrateRk4(current_state, current_control,
                                                      dt * 0.5,
                                                      vehicle.parameters.wheel_base);
            const casadi::MX midpoint_center = symbolicCenter(
                midpoint, vehicle.parameters.rear_axle_to_center);
            const Aabb middle_box{std::max(box.min_x, next_box.min_x),
                                  std::min(box.max_x, next_box.max_x),
                                  std::max(box.min_y, next_box.min_y),
                                  std::min(box.max_y, next_box.max_y)};
            if (!(node == 0 && guess->start_aabb_exempt)) {
                constraints.add(midpoint_center(0), middle_box.min_x, middle_box.max_x);
                constraints.add(midpoint_center(1), middle_box.min_y, middle_box.max_y);
            }
            const casadi::MX acceleration = control(0, node);
            const casadi::MX steer_rate = control(1, node);
            objective += dt * (config_.acceleration_weight * acceleration * acceleration +
                               config_.steer_rate_weight * steer_rate * steer_rate);
            if (node + 2 < nodes) {
                const casadi::MX da = control(0, node + 1) - acceleration;
                const casadi::MX dw = control(1, node + 1) - steer_rate;
                objective += config_.control_smooth_weight * (da * da + dw * dw);
            }
        }
        symbolic_vehicles.push_back({state, control, &vehicle, guess});
    }

    for (size_t first = 0; first < symbolic_vehicles.size(); ++first) {
        for (size_t second = first + 1; second < symbolic_vehicles.size(); ++second) {
            const auto& first_vehicle = symbolic_vehicles[first];
            const auto& second_vehicle = symbolic_vehicles[second];
            const double required = first_vehicle.input->parameters.disc_radius +
                                    second_vehicle.input->parameters.disc_radius;
            for (size_t node = 0; node < nodes; ++node) {
                const casadi::MX first_center = symbolicCenter(
                    first_vehicle.state(casadi::Slice(), node),
                    first_vehicle.input->parameters.rear_axle_to_center);
                const casadi::MX second_center = symbolicCenter(
                    second_vehicle.state(casadi::Slice(), node),
                    second_vehicle.input->parameters.rear_axle_to_center);
                constraints.add(squaredDistance(first_center, second_center),
                                required * required, kInfinity);
                if (node + 1 == nodes) continue;
                const casadi::MX first_mid = integrateRk4(
                    first_vehicle.state(casadi::Slice(), node),
                    first_vehicle.control(casadi::Slice(), node), dt * 0.5,
                    first_vehicle.input->parameters.wheel_base);
                const casadi::MX second_mid = integrateRk4(
                    second_vehicle.state(casadi::Slice(), node),
                    second_vehicle.control(casadi::Slice(), node), dt * 0.5,
                    second_vehicle.input->parameters.wheel_base);
                constraints.add(squaredDistance(
                    symbolicCenter(first_mid, first_vehicle.input->parameters.rear_axle_to_center),
                    symbolicCenter(second_mid, second_vehicle.input->parameters.rear_axle_to_center)),
                    required * required, kInfinity);
            }
        }
    }
    for (const SymbolicVehicle& vehicle : symbolic_vehicles) {
        for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
            const double required = vehicle.input->parameters.disc_radius +
                                    obstacle.disc_radius;
            const casadi::DM fixed_center(std::vector<double>{obstacle.disc_center.x,
                                                               obstacle.disc_center.y});
            for (size_t node = 0; node < nodes; ++node) {
                const casadi::MX center = symbolicCenter(
                    vehicle.state(casadi::Slice(), node),
                    vehicle.input->parameters.rear_axle_to_center);
                constraints.add(squaredDistance(center, fixed_center),
                                required * required, kInfinity);
                if (node + 1 == nodes) continue;
                const casadi::MX midpoint = integrateRk4(
                    vehicle.state(casadi::Slice(), node),
                    vehicle.control(casadi::Slice(), node), dt * 0.5,
                    vehicle.input->parameters.wheel_base);
                constraints.add(squaredDistance(symbolicCenter(
                    midpoint, vehicle.input->parameters.rear_axle_to_center), fixed_center),
                    required * required, kInfinity);
            }
        }
    }

    const casadi::MX decision = casadi::MX::vertcat(variables);
    const casadi::MX constraint_vector = constraints.expressions.empty() ?
        casadi::MX::zeros(0, 1) : casadi::MX::vertcat(constraints.expressions);
    const casadi::MXDict problem{{"x", decision}, {"f", objective},
                                 {"g", constraint_vector}};
    casadi::Dict options;
    options["print_time"] = false;
    options["ipopt.max_iter"] = config_.max_iterations;
    options["ipopt.tol"] = config_.tolerance;
    options["ipopt.acceptable_tol"] = config_.acceptable_tolerance;
    options["ipopt.print_level"] = config_.print_level;
    options["ipopt.sb"] = "yes";
    result.problem_built = true;
    const auto started = std::chrono::steady_clock::now();
    const std::clock_t cpu_started = std::clock();
    try {
        casadi::Function solver = casadi::nlpsol("forklift_nlp_solver", "ipopt",
                                                 problem, options);
        casadi::DMDict arguments;
        arguments["x0"] = initial_values;
        arguments["lbx"] = lower_bounds;
        arguments["ubx"] = upper_bounds;
        arguments["lbg"] = constraints.lower;
        arguments["ubg"] = constraints.upper;
        const casadi::DMDict solution = solver(arguments);
        const casadi::Dict stats = solver.stats();
        if (stats.count("success")) result.statistics.success = stats.at("success").to_bool();
        if (stats.count("return_status")) result.statistics.return_status = stats.at("return_status").to_string();
        if (stats.count("iter_count")) result.statistics.iterations = stats.at("iter_count").to_int();
        if (stats.count("t_proc_total")) result.statistics.cpu_time = stats.at("t_proc_total").to_double();
        if (stats.count("t_wall_total")) result.statistics.wall_time = stats.at("t_wall_total").to_double();
        result.statistics.objective = static_cast<double>(solution.at("f"));
        const std::vector<double> constraint_values = solution.at("g").get_elements();
        const std::vector<double> decision_values = solution.at("x").get_elements();
        result.statistics.max_constraint_violation = std::max(
            maximumViolation(constraint_values, constraints.lower, constraints.upper),
            maximumViolation(decision_values, lower_bounds, upper_bounds));
        result.solver_success = result.statistics.success;
        size_t offset = 0;
        result.total_time = decision_values[offset++];
        result.node_count = nodes;
        result.average_dt = result.total_time / static_cast<double>(nodes - 1);
        for (const SymbolicVehicle& symbolic : symbolic_vehicles) {
            OptimizedVehicleTrajectory trajectory;
            trajectory.id = symbolic.input->id;
            trajectory.start_aabb_exempt = symbolic.guess->start_aabb_exempt;
            trajectory.nodes.resize(nodes);
            for (size_t node = 0; node < nodes; ++node) {
                OptimizedTrajectoryNode& value = trajectory.nodes[node];
                value.index = node;
                value.time = result.average_dt * static_cast<double>(node);
                value.rear_axle_pose.x = decision_values[offset++];
                value.rear_axle_pose.y = decision_values[offset++];
                value.rear_axle_pose.yaw = normalizeAngle(decision_values[offset++]);
                value.signed_speed = decision_values[offset++];
                value.front_steer = decision_values[offset++];
                value.aabb_index = symbolic.guess->nodes[node].aabb_index;
                value.disc_center = bodyCenter(value.rear_axle_pose,
                    symbolic.input->parameters.rear_axle_to_center);
            }
            for (size_t node = 0; node + 1 < nodes; ++node) {
                trajectory.nodes[node].acceleration = decision_values[offset++];
                trajectory.nodes[node].steer_rate = decision_values[offset++];
            }
            if (nodes > 1) {
                trajectory.nodes.back().acceleration = trajectory.nodes[nodes - 2].acceleration;
                trajectory.nodes.back().steer_rate = trajectory.nodes[nodes - 2].steer_rate;
            }
            result.vehicles.push_back(std::move(trajectory));
        }
        result.validation = validateSolution(input, result, config_);
        result.validation_passed = result.validation.passed;
        if (!result.solver_success) {
            result.reason = "IPOPT did not report success: " +
                            result.statistics.return_status;
        } else if (!result.validation_passed) {
            result.reason = "IPOPT returned success but independent validation failed";
        } else {
            result.reason = "solver and independent validation succeeded";
        }
    } catch (const std::exception& error) {
        result.reason = std::string("CasADi/IPOPT exception: ") + error.what();
        result.statistics.return_status = "exception";
    }
    const double measured_wall = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    const double measured_cpu = static_cast<double>(std::clock() - cpu_started) /
                                static_cast<double>(CLOCKS_PER_SEC);
    if (result.statistics.wall_time <= 0.0) result.statistics.wall_time = measured_wall;
    if (result.statistics.cpu_time <= 0.0) result.statistics.cpu_time = measured_cpu;
    return result;
}

bool saveNlpSolveResult(const NlpSolveResult& result,
                        const std::string& case_directory,
                        std::string& error) {
    if (case_directory.empty()) {
        error = "case directory is empty";
        return false;
    }
    const std::filesystem::path directory(case_directory);
    if (!std::filesystem::is_directory(directory)) {
        error = "case directory does not exist";
        return false;
    }
    std::ofstream report(directory / "solver_report.yaml", std::ios::trunc);
    if (!report) {
        error = "cannot open solver_report.yaml";
        return false;
    }
    report << std::boolalpha << std::fixed << std::setprecision(9)
           << "problem_built: " << result.problem_built << '\n'
           << "solver_success: " << result.solver_success << '\n'
           << "validation_passed: " << result.validation_passed << '\n'
           << "executable_trajectory: false\n"
           << "reason: " << yamlQuote(result.reason) << '\n'
           << "solver: " << yamlQuote(result.statistics.solver_name) << '\n'
           << "casadi_version: " << yamlQuote(result.statistics.casadi_version) << '\n'
           << "return_status: " << yamlQuote(result.statistics.return_status) << '\n'
           << "iterations: " << result.statistics.iterations << '\n'
           << "cpu_time: " << result.statistics.cpu_time << '\n'
           << "wall_time: " << result.statistics.wall_time << '\n'
           << "objective: " << result.statistics.objective << '\n'
           << "max_constraint_violation: " << result.statistics.max_constraint_violation << '\n'
           << "node_count: " << result.node_count << '\n'
           << "total_time: " << result.total_time << '\n'
           << "average_dt: " << result.average_dt << '\n'
           << "collision_models:\n"
           << "  optimization_constraints: body_center_discs\n"
           << "  independent_validation: obb_sat\n"
           << "validation:\n"
           << "  collision_geometry: obb_sat\n"
           << "  reason: " << yamlQuote(result.validation.reason) << '\n'
           << "  max_start_error: " << result.validation.max_start_error << '\n'
           << "  max_terminal_error: " << result.validation.max_terminal_error << '\n'
           << "  max_kinematic_position_residual: "
           << result.validation.max_kinematic_position_residual << '\n'
           << "  max_kinematic_heading_residual: "
           << result.validation.max_kinematic_heading_residual << '\n'
           << "  max_speed_violation: " << result.validation.max_speed_violation << '\n'
           << "  max_steer_violation: " << result.validation.max_steer_violation << '\n'
           << "  max_acceleration_violation: "
           << result.validation.max_acceleration_violation << '\n'
           << "  max_steer_rate_violation: "
           << result.validation.max_steer_rate_violation << '\n'
           << "  max_aabb_violation: " << result.validation.max_aabb_violation << '\n'
           << "  minimum_participant_obb_separation_margin: "
           << result.validation.minimum_participant_obb_separation_margin << '\n'
           << "  minimum_static_obb_separation_margin: "
           << result.validation.minimum_static_obb_separation_margin << '\n'
           << "  participant_obb_collision_samples: "
           << result.validation.participant_obb_collision_samples << '\n'
           << "  static_obb_collision_samples: "
           << result.validation.static_obb_collision_samples << '\n'
           << "  aabb_violation_samples: "
           << result.validation.aabb_violation_samples << '\n'
           << "  invalid_aabb_switches: "
           << result.validation.invalid_aabb_switches << '\n';
    if (!report) {
        error = "failed while writing solver_report.yaml";
        return false;
    }
    std::ofstream trajectory(directory / "optimized_trajectory.csv", std::ios::trunc);
    if (!trajectory) {
        error = "cannot open optimized_trajectory.csv";
        return false;
    }
    trajectory << "vehicle_id,node_index,time,rear_x,rear_y,heading,signed_speed,"
                  "front_steer,acceleration,steer_rate,aabb_index,disc_center_x,"
                  "disc_center_y,solver_success,validation_passed\n";
    trajectory << std::fixed << std::setprecision(9) << std::boolalpha;
    for (const OptimizedVehicleTrajectory& vehicle : result.vehicles) {
        for (const OptimizedTrajectoryNode& node : vehicle.nodes) {
            trajectory << vehicle.id << ',' << node.index << ',' << node.time << ','
                       << node.rear_axle_pose.x << ',' << node.rear_axle_pose.y << ','
                       << node.rear_axle_pose.yaw << ',' << node.signed_speed << ','
                       << node.front_steer << ',' << node.acceleration << ','
                       << node.steer_rate << ',' << node.aabb_index << ','
                       << node.disc_center.x << ',' << node.disc_center.y << ','
                       << result.solver_success << ',' << result.validation_passed << '\n';
        }
    }
    if (!trajectory) {
        error = "failed while writing optimized_trajectory.csv";
        return false;
    }
    error = "ok";
    return true;
}

}  // namespace forklift_nlp_recovery
