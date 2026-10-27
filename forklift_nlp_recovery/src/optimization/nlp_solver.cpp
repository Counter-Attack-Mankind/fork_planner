#include "forklift_nlp_recovery/nlp_solver.h"
#include "forklift_nlp_recovery/nlp_validator.h"

#include <casadi/casadi.hpp>

#include <algorithm>
#include <array>
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

size_t directionChanges(const std::vector<double>& speeds, double threshold) {
    int direction = 0;
    size_t changes = 0;
    for (double speed : speeds) {
        int next = 0;
        if (speed >= threshold) next = 1;
        else if (speed <= -threshold) next = -1;
        if (next == 0) continue;
        if (direction != 0 && next != direction) ++changes;
        direction = next;
    }
    return changes;
}

double oppositeDirectionDistance(const std::vector<double>& speeds,
                                 const VehicleInitialGuess& reference,
                                 double dt) {
    double distance = 0.0;
    for (size_t node = 0; node + 1 < speeds.size(); ++node) {
        const double reference_sign = reference.nodes[node].path_type == 1 ? -1.0 : 1.0;
        const double first = std::max(0.0, -reference_sign * speeds[node]);
        const double second = std::max(0.0, -reference_sign * speeds[node + 1]);
        distance += 0.5 * dt * (first + second);
    }
    return distance;
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

std::string yamlNumber(double value) {
    if (std::isinf(value)) return value > 0.0 ? ".inf" : "-.inf";
    if (std::isnan(value)) return ".nan";
    std::ostringstream out;
    out << std::fixed << std::setprecision(9) << value;
    return out.str();
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
    std::vector<NlpConstraintMetadata> metadata;

    void add(const casadi::MX& expression, double minimum, double maximum,
             const NlpConstraintMetadata& constraint_metadata) {
        expressions.push_back(expression);
        lower.push_back(minimum);
        upper.push_back(maximum);
        metadata.push_back(constraint_metadata);
    }
};

struct Vector2 {
    double x = 0.0;
    double y = 0.0;
};

struct ObcaDualGuess {
    std::array<double, 4> lambda{};
    std::array<double, 4> mu{};
};

double dot(const Vector2& first, const Vector2& second) {
    return first.x * second.x + first.y * second.y;
}

double projectionRadius(double half_length, double half_width,
                        const Vector2& longitudinal, const Vector2& lateral,
                        const Vector2& axis) {
    return half_length * std::abs(dot(longitudinal, axis)) +
           half_width * std::abs(dot(lateral, axis));
}

std::array<double, 4> axisCoefficients(const Vector2& axis,
                                       const Vector2& first,
                                       const Vector2& second) {
    const double first_component = dot(axis, first);
    const double second_component = dot(axis, second);
    return {std::max(0.0, first_component), std::max(0.0, -first_component),
            std::max(0.0, second_component), std::max(0.0, -second_component)};
}

ObcaDualGuess initialObcaDual(const Pose2d& rear, const VehicleParameters& moving,
                              const StaticVehicleObstacle& obstacle) {
    const Pose2d moving_center = bodyCenter(rear, moving.rear_axle_to_center);
    const Pose2d obstacle_center = bodyCenter(obstacle.rear_axle_pose,
                                              obstacle.rear_axle_to_center);
    const Vector2 moving_longitudinal{std::cos(rear.yaw), std::sin(rear.yaw)};
    const Vector2 moving_lateral{-moving_longitudinal.y, moving_longitudinal.x};
    const Vector2 obstacle_longitudinal{std::cos(obstacle.rear_axle_pose.yaw),
                                        std::sin(obstacle.rear_axle_pose.yaw)};
    const Vector2 obstacle_lateral{-obstacle_longitudinal.y,
                                   obstacle_longitudinal.x};
    const Vector2 delta{moving_center.x - obstacle_center.x,
                        moving_center.y - obstacle_center.y};
    const Vector2 candidates[] = {
        moving_longitudinal, {-moving_longitudinal.x, -moving_longitudinal.y},
        moving_lateral, {-moving_lateral.x, -moving_lateral.y},
        obstacle_longitudinal, {-obstacle_longitudinal.x, -obstacle_longitudinal.y},
        obstacle_lateral, {-obstacle_lateral.x, -obstacle_lateral.y}};
    Vector2 normal = candidates[0];
    double best_separation = -kInfinity;
    for (const Vector2& candidate : candidates) {
        const double separation = dot(delta, candidate) - projectionRadius(
            0.5 * moving.length, 0.5 * moving.width, moving_longitudinal,
            moving_lateral, candidate) - projectionRadius(
            0.5 * obstacle.length, 0.5 * obstacle.width, obstacle_longitudinal,
            obstacle_lateral, candidate);
        if (separation > best_separation) {
            best_separation = separation;
            normal = candidate;
        }
    }
    ObcaDualGuess guess;
    guess.lambda = axisCoefficients(normal, obstacle_longitudinal, obstacle_lateral);
    const Vector2 local_normal{dot(normal, moving_longitudinal),
                               dot(normal, moving_lateral)};
    guess.mu = axisCoefficients({-local_normal.x, -local_normal.y},
                                {1.0, 0.0}, {0.0, 1.0});
    return guess;
}

Pose2d midpointPose(const InitialGuessNode& first, const InitialGuessNode& second) {
    return {0.5 * (first.rear_axle_pose.x + second.rear_axle_pose.x),
            0.5 * (first.rear_axle_pose.y + second.rear_axle_pose.y),
            first.rear_axle_pose.yaw + 0.5 * normalizeAngle(
                second.rear_axle_pose.yaw - first.rear_axle_pose.yaw)};
}

void addStaticObca(const casadi::MX& state, const Pose2d& initial_rear,
                   const NlpVehicleInput& moving,
                   const StaticVehicleObstacle& obstacle, size_t node,
                   const std::string& sample_name,
                   std::vector<casadi::MX>& variables,
                   std::vector<double>& initial_values,
                   std::vector<double>& lower_bounds,
                   std::vector<double>& upper_bounds,
                   std::vector<NlpConstraintMetadata>& variable_metadata,
                   ConstraintData& constraints) {
    const std::string suffix = "_V" + std::to_string(moving.id) + "_O" +
        std::to_string(obstacle.id) + "_N" + std::to_string(node) + "_" + sample_name;
    const casadi::MX lambda = casadi::MX::sym("obca_lambda" + suffix, 4);
    const casadi::MX mu = casadi::MX::sym("obca_mu" + suffix, 4);
    variables.push_back(lambda);
    variables.push_back(mu);
    const ObcaDualGuess dual_guess = initialObcaDual(initial_rear, moving.parameters,
                                                      obstacle);
    for (size_t index = 0; index < 4; ++index) {
        initial_values.push_back(dual_guess.lambda[index]);
        lower_bounds.push_back(0.0);
        upper_bounds.push_back(kInfinity);
        variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
            moving.id, obstacle.id, node, sample_name + "_lambda_" + std::to_string(index)});
    }
    for (size_t index = 0; index < 4; ++index) {
        initial_values.push_back(dual_guess.mu[index]);
        lower_bounds.push_back(0.0);
        upper_bounds.push_back(kInfinity);
        variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
            moving.id, obstacle.id, node, sample_name + "_mu_" + std::to_string(index)});
    }

    const double obstacle_cos = std::cos(obstacle.rear_axle_pose.yaw);
    const double obstacle_sin = std::sin(obstacle.rear_axle_pose.yaw);
    const double obstacle_center_x = obstacle.rear_axle_pose.x +
        obstacle.rear_axle_to_center * obstacle_cos;
    const double obstacle_center_y = obstacle.rear_axle_pose.y +
        obstacle.rear_axle_to_center * obstacle_sin;
    const double obstacle_longitudinal_b = obstacle_cos * obstacle_center_x +
        obstacle_sin * obstacle_center_y + 0.5 * obstacle.length;
    const double obstacle_longitudinal_negative_b = -obstacle_cos * obstacle_center_x -
        obstacle_sin * obstacle_center_y + 0.5 * obstacle.length;
    const double obstacle_lateral_b = -obstacle_sin * obstacle_center_x +
        obstacle_cos * obstacle_center_y + 0.5 * obstacle.width;
    const double obstacle_lateral_negative_b = obstacle_sin * obstacle_center_x -
        obstacle_cos * obstacle_center_y + 0.5 * obstacle.width;
    const casadi::MX normal_x = obstacle_cos * (lambda(0) - lambda(1)) -
        obstacle_sin * (lambda(2) - lambda(3));
    const casadi::MX normal_y = obstacle_sin * (lambda(0) - lambda(1)) +
        obstacle_cos * (lambda(2) - lambda(3));
    const casadi::MX heading = state(2);
    const casadi::MX local_normal_x = cos(heading) * normal_x + sin(heading) * normal_y;
    const casadi::MX local_normal_y = -sin(heading) * normal_x + cos(heading) * normal_y;
    constraints.add(mu(0) - mu(1) + local_normal_x, 0.0, 0.0,
        {NlpConstraintCategory::STATIC_OBCA, moving.id, obstacle.id, node,
         sample_name + "_stationarity_x"});
    constraints.add(mu(2) - mu(3) + local_normal_y, 0.0, 0.0,
        {NlpConstraintCategory::STATIC_OBCA, moving.id, obstacle.id, node,
         sample_name + "_stationarity_y"});
    constraints.add(normal_x * normal_x + normal_y * normal_y, 1.0, 1.0,
        {NlpConstraintCategory::STATIC_OBCA, moving.id, obstacle.id, node,
         sample_name + "_normalization"});

    const casadi::MX center = symbolicCenter(state, moving.parameters.rear_axle_to_center);
    const casadi::MX separation =
        -0.5 * moving.parameters.length * (mu(0) + mu(1)) -
        0.5 * moving.parameters.width * (mu(2) + mu(3)) +
        lambda(0) * (obstacle_cos * center(0) + obstacle_sin * center(1) -
                     obstacle_longitudinal_b) +
        lambda(1) * (-obstacle_cos * center(0) - obstacle_sin * center(1) -
                     obstacle_longitudinal_negative_b) +
        lambda(2) * (-obstacle_sin * center(0) + obstacle_cos * center(1) -
                     obstacle_lateral_b) +
        lambda(3) * (obstacle_sin * center(0) - obstacle_cos * center(1) -
                     obstacle_lateral_negative_b);
    constraints.add(separation, 0.0, kInfinity,
        {NlpConstraintCategory::STATIC_OBCA, moving.id, obstacle.id, node,
         sample_name + "_separation"});
}

}  // namespace

NlpSolver::NlpSolver(const NlpSolverConfig& config,
                     const CorridorBuilder& corridor_builder)
    : config_(config), corridor_builder_(corridor_builder) {}

NlpSolveResult NlpSolver::solve(const NlpOptimizationInput& input,
                                const NlpInitialGuess& initial_guess) const {
    NlpSolveResult result;
    result.statistics.casadi_version = casadi::CasadiMeta::version();
    result.constraint_diagnostics.reason = "candidate solution unavailable";
    result.direction_switch_speed_threshold =
        config_.direction_switch_speed_threshold;
    if (!input.valid || !initial_guess.generated) {
        result.reason = "valid optimization input and generated initial guess are required";
        return result;
    }
    if (input.vehicles.size() != 1 || initial_guess.vehicles.size() != 1 ||
        initial_guess.node_count < 2 ||
        initial_guess.total_time <= 0.0) {
        result.reason = "single active vehicle and a valid time grid are required";
        return result;
    }
    if (config_.min_time_scale <= 0.0 ||
        config_.max_time_scale < config_.min_time_scale ||
        config_.reference_direction_weight < 0.0 ||
        config_.direction_smoothing_speed <= 0.0 ||
        config_.direction_switch_speed_threshold <= 0.0 ||
        config_.validation_substeps == 0) {
        result.reason = "invalid solver configuration";
        return result;
    }

    const size_t nodes = initial_guess.node_count;
    std::vector<casadi::MX> variables;
    std::vector<double> initial_values;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<NlpConstraintMetadata> variable_metadata;
    ConstraintData constraints;
    std::vector<SymbolicVehicle> symbolic_vehicles;
    const casadi::MX total_time = casadi::MX::sym("T");
    variables.push_back(total_time);
    initial_values.push_back(initial_guess.total_time);
    lower_bounds.push_back(std::max(1e-3, initial_guess.total_time * config_.min_time_scale));
    upper_bounds.push_back(std::max(lower_bounds.back() + 1e-3,
                                    initial_guess.total_time * config_.max_time_scale));
    variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
                                 -1, -1, 0, "total_time"});
    const casadi::MX dt = total_time / static_cast<double>(nodes - 1);
    casadi::MX objective = config_.time_weight * total_time;

    for (size_t index = 0; index < input.vehicles.size(); ++index) {
        const NlpVehicleInput& vehicle = input.vehicles[index];
        const VehicleInitialGuess* guess = findGuess(initial_guess, vehicle.id);
        if (guess == nullptr || guess->nodes.size() != nodes ||
            vehicle.parameters.wheel_base <= 0.0 ||
            vehicle.parameters.length <= 0.0 ||
            vehicle.parameters.width <= 0.0 ||
            vehicle.parameters.max_speed <= 0.0 ||
            vehicle.parameters.max_accel <= 0.0 ||
            vehicle.parameters.max_decel <= 0.0 ||
            vehicle.parameters.max_steer_angle <= 0.0 ||
            vehicle.parameters.max_steer_rate <= 0.0) {
            result.reason = "vehicle input or initial guess is incomplete for V" +
                            std::to_string(vehicle.id);
            return result;
        }
        if (!vehicle.corridor_valid || vehicle.corridor.size() != nodes ||
            !guess->one_to_one_aabbs_valid) {
            result.reason = "one-to-one node AABBs are unavailable for V" +
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

        static const char* state_components[] = {"rear_x", "rear_y", "heading",
                                                  "signed_speed", "front_steer"};

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
                variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
                    vehicle.id, -1, node, state_components[component]});
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
                if (vehicle.initial_state.steering_valid) {
                    lower_bounds[base + 4] = upper_bounds[base + 4] =
                        vehicle.initial_state.steering_angle;
                }
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
            variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
                vehicle.id, -1, interval, "acceleration"});
            variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
                vehicle.id, -1, interval, "steer_rate"});
        }

        for (size_t node = 0; node < nodes; ++node) {
            if (guess->nodes[node].aabb_index != static_cast<int>(node)) {
                result.reason = "invalid AABB index at V" + std::to_string(vehicle.id) +
                                " node " + std::to_string(node);
                return result;
            }
            const Aabb& box = vehicle.corridor[node];
            const casadi::MX center = symbolicCenter(state(casadi::Slice(), node),
                vehicle.parameters.rear_axle_to_center);
            constraints.add(center(0), box.min_x, box.max_x,
                {NlpConstraintCategory::AABB_NODE, vehicle.id, -1, node, "x"});
            constraints.add(center(1), box.min_y, box.max_y,
                {NlpConstraintCategory::AABB_NODE, vehicle.id, -1, node, "y"});
            if (node + 1 == nodes) continue;
            const casadi::MX current_state = state(casadi::Slice(), node);
            const casadi::MX current_control = control(casadi::Slice(), node);
            const casadi::MX predicted = integrateRk4(current_state, current_control,
                                                       dt, vehicle.parameters.wheel_base);
            static const char* rk4_components[] = {"rear_x", "rear_y", "heading",
                                                    "signed_speed", "front_steer"};
            for (size_t component = 0; component < 5; ++component) {
                constraints.add(state(component, node + 1) - predicted(component),
                    0.0, 0.0, {NlpConstraintCategory::RK4, vehicle.id, -1,
                               node, rk4_components[component]});
            }
            const casadi::MX acceleration = control(0, node);
            const casadi::MX steer_rate = control(1, node);
            objective += dt * (config_.acceleration_weight * acceleration * acceleration +
                               config_.steer_rate_weight * steer_rate * steer_rate);
            const double reference_sign = guess->nodes[node].path_type == 1 ? -1.0 : 1.0;
            const casadi::MX directed_speed = reference_sign * state(3, node);
            const double smoothing = config_.direction_smoothing_speed;
            const casadi::MX opposite_magnitude =
                sqrt(directed_speed * directed_speed + smoothing * smoothing) - smoothing;
            const casadi::MX opposite_gate = 1.0 / (1.0 + exp(directed_speed / smoothing));
            objective += dt * config_.reference_direction_weight *
                         opposite_magnitude * opposite_gate;
            if (node + 2 < nodes) {
                const casadi::MX da = control(0, node + 1) - acceleration;
                const casadi::MX dw = control(1, node + 1) - steer_rate;
                objective += config_.control_smooth_weight * (da * da + dw * dw);
            }
        }
        symbolic_vehicles.push_back({state, control, &vehicle, guess});
    }

    for (const SymbolicVehicle& vehicle : symbolic_vehicles) {
        for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
            if (obstacle.length <= 0.0 || obstacle.width <= 0.0) {
                result.reason = "static vehicle OBB geometry is invalid for V" +
                    std::to_string(obstacle.id);
                return result;
            }
            for (size_t node = 0; node < nodes; ++node) {
                addStaticObca(vehicle.state(casadi::Slice(), node),
                    vehicle.guess->nodes[node].rear_axle_pose, *vehicle.input,
                    obstacle, node, "node", variables, initial_values,
                    lower_bounds, upper_bounds, variable_metadata, constraints);
                if (node + 1 == nodes) continue;
                const casadi::MX midpoint = integrateRk4(
                    vehicle.state(casadi::Slice(), node),
                    vehicle.control(casadi::Slice(), node), dt * 0.5,
                    vehicle.input->parameters.wheel_base);
                addStaticObca(midpoint, midpointPose(vehicle.guess->nodes[node],
                    vehicle.guess->nodes[node + 1]), *vehicle.input, obstacle,
                    node, "midpoint", variables, initial_values, lower_bounds,
                    upper_bounds, variable_metadata, constraints);
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
        result.constraint_diagnostics = diagnoseNlpConstraints({
            constraint_values, constraints.lower, constraints.upper,
            constraints.metadata, decision_values, lower_bounds, upper_bounds,
            variable_metadata});
        result.statistics.max_constraint_violation =
            result.constraint_diagnostics.max_violation;
        result.solver_success = result.statistics.success;
        size_t offset = 0;
        result.total_time = decision_values[offset++];
        result.node_count = nodes;
        result.average_dt = result.total_time / static_cast<double>(nodes - 1);
        for (const SymbolicVehicle& symbolic : symbolic_vehicles) {
            OptimizedVehicleTrajectory trajectory;
            trajectory.id = symbolic.input->id;
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
            std::vector<double> initial_speeds;
            std::vector<double> optimized_speeds;
            initial_speeds.reserve(nodes);
            optimized_speeds.reserve(nodes);
            for (size_t node = 0; node < nodes; ++node) {
                initial_speeds.push_back(symbolic.guess->nodes[node].signed_speed);
                optimized_speeds.push_back(trajectory.nodes[node].signed_speed);
            }
            result.direction_diagnostics.push_back({
                trajectory.id,
                directionChanges(initial_speeds,
                                 config_.direction_switch_speed_threshold),
                directionChanges(optimized_speeds,
                                 config_.direction_switch_speed_threshold),
                oppositeDirectionDistance(initial_speeds, *symbolic.guess,
                                           initial_guess.average_dt),
                oppositeDirectionDistance(optimized_speeds, *symbolic.guess,
                                           result.average_dt)});
            result.vehicles.push_back(std::move(trajectory));
        }
        result.validation = forklift_nlp_recovery::validateSolution(
            input, result, config_, corridor_builder_);
        result.validation_passed = result.validation.passed;
        if (!result.solver_success) {
            result.reason = "IPOPT did not report success: " +
                            result.statistics.return_status;
        } else if (!result.validation_passed) {
            result.reason = "IPOPT succeeded; independent validation reported violations";
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
           << "execution_publish_eligible: " << result.execution_publish_eligible << '\n'
           << "execution_published: " << result.execution_published << '\n'
           << "executable_trajectory: " << result.execution_publish_eligible << '\n'
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
           << "  optimization_constraints: active_obb_static_obb_obca\n"
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
           << "  static_map_violation_samples: "
           << result.validation.static_map_violation_samples << '\n';
    if (result.validation.static_map_violations.empty()) {
        report << "  static_map_violations: []\n";
    } else {
        report << "  static_map_violations:\n";
    }
    for (const StaticMapViolation& violation : result.validation.static_map_violations) {
        report << "    - vehicle_id: " << violation.vehicle_id << '\n'
               << "      interval: " << violation.interval_index << '\n'
               << "      substep: " << violation.substep_index << '\n'
               << "      sample_time: " << violation.sample_time << '\n'
               << "      disc_center_x: " << violation.disc_center.x << '\n'
               << "      disc_center_y: " << violation.disc_center.y << '\n'
               << "      violation_type: " << yamlQuote(violation.type) << '\n'
               << "      shelf_index: " << violation.shelf_index << '\n'
               << "      violation_distance_m: " << violation.violation_distance << '\n';
    }
    report << "direction_diagnostics:\n"
           << "  speed_hysteresis_threshold: "
           << result.direction_switch_speed_threshold << '\n'
           << "  vehicles:\n";
    for (const VehicleDirectionDiagnostics& vehicle : result.direction_diagnostics) {
        report << "    - id: " << vehicle.id << '\n'
               << "      initial_direction_changes: "
               << vehicle.initial_direction_changes << '\n'
               << "      optimized_direction_changes: "
               << vehicle.optimized_direction_changes << '\n'
               << "      initial_opposite_direction_distance: "
               << vehicle.initial_opposite_direction_distance << '\n'
               << "      optimized_opposite_direction_distance: "
               << vehicle.optimized_opposite_direction_distance << '\n';
    }
    report << "constraint_diagnostics:\n"
           << "  available: " << result.constraint_diagnostics.available << '\n'
           << "  reason: " << yamlQuote(result.constraint_diagnostics.reason) << '\n'
           << "  max_violation: " << result.constraint_diagnostics.max_violation << '\n'
           << "  categories:\n";
    for (const NlpConstraintCategoryDiagnostics& category :
         result.constraint_diagnostics.categories) {
        report << "    - category: "
               << nlpConstraintCategoryName(category.category) << '\n'
               << "      sample_count: " << category.sample_count << '\n'
               << "      violation_count: " << category.violation_count << '\n'
               << "      max_violation: " << category.max_violation << '\n'
               << "      max_distance_shortfall_m: "
               << category.max_distance_shortfall << '\n';
    }
    report << "  worst_violations:\n";
    for (size_t rank = 0;
         rank < result.constraint_diagnostics.worst_violations.size(); ++rank) {
        const NlpConstraintViolation& violation =
            result.constraint_diagnostics.worst_violations[rank];
        report << "    - rank: " << rank + 1 << '\n'
               << "      category: "
               << nlpConstraintCategoryName(violation.metadata.category) << '\n'
               << "      vehicle_id: " << violation.metadata.vehicle_id << '\n'
               << "      other_vehicle_id: "
               << violation.metadata.other_vehicle_id << '\n'
               << "      node: " << violation.metadata.node_index << '\n'
               << "      component: " << yamlQuote(violation.metadata.component) << '\n'
               << "      value: " << yamlNumber(violation.value) << '\n'
               << "      lower_bound: " << yamlNumber(violation.lower_bound) << '\n'
               << "      upper_bound: " << yamlNumber(violation.upper_bound) << '\n'
               << "      violation: " << yamlNumber(violation.violation) << '\n';
        if (violation.metadata.category == NlpConstraintCategory::STATIC_OBCA &&
            violation.metadata.component.find("separation") != std::string::npos) {
            report << "      separation_shortfall_m: "
                   << yamlNumber(violation.distance_shortfall) << '\n';
        }
    }
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
