#include "forklift_nlp_recovery/nlp_solver.h"
#include "forklift_nlp_recovery/nlp_validator.h"

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

casadi::MX squaredDistance(const casadi::MX& first, const casadi::MX& second) {
    const casadi::MX dx = first(0) - second(0);
    const casadi::MX dy = first(1) - second(1);
    return dx * dx + dy * dy;
}

casadi::MX squaredDistanceToObb(const casadi::MX& center,
                                const StaticVehicleObstacle& obstacle) {
    const casadi::MX dx = center(0) - obstacle.disc_center.x;
    const casadi::MX dy = center(1) - obstacle.disc_center.y;
    const double c = std::cos(obstacle.rear_axle_pose.yaw);
    const double s = std::sin(obstacle.rear_axle_pose.yaw);
    const casadi::MX local_x = c * dx + s * dy;
    const casadi::MX local_y = -s * dx + c * dy;
    const casadi::MX outside_x = fmax(fabs(local_x) - obstacle.length * 0.5, 0.0);
    const casadi::MX outside_y = fmax(fabs(local_y) - obstacle.width * 0.5, 0.0);
    return outside_x * outside_x + outside_y * outside_y;
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
                    required * required, kInfinity,
                    {NlpConstraintCategory::PARTICIPANT_DISC,
                     first_vehicle.input->id, second_vehicle.input->id,
                     node, "node", required});
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
                    required * required, kInfinity,
                    {NlpConstraintCategory::PARTICIPANT_DISC,
                     first_vehicle.input->id, second_vehicle.input->id,
                     node, "midpoint", required});
            }
        }
    }
    for (const SymbolicVehicle& vehicle : symbolic_vehicles) {
        for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
            const double required = vehicle.input->parameters.disc_radius;
            for (size_t node = 0; node < nodes; ++node) {
                const casadi::MX center = symbolicCenter(
                    vehicle.state(casadi::Slice(), node),
                    vehicle.input->parameters.rear_axle_to_center);
                constraints.add(squaredDistanceToObb(center, obstacle),
                    required * required, kInfinity,
                    {NlpConstraintCategory::STATIC_DISC, vehicle.input->id,
                     obstacle.id, node, "node", required});
                if (node + 1 == nodes) continue;
                const casadi::MX midpoint = integrateRk4(
                    vehicle.state(casadi::Slice(), node),
                    vehicle.control(casadi::Slice(), node), dt * 0.5,
                    vehicle.input->parameters.wheel_base);
                constraints.add(squaredDistanceToObb(symbolicCenter(
                    midpoint, vehicle.input->parameters.rear_axle_to_center), obstacle),
                    required * required, kInfinity,
                    {NlpConstraintCategory::STATIC_DISC, vehicle.input->id,
                     obstacle.id, node, "midpoint", required});
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
           << "  optimization_constraints: active_disc_static_obb\n"
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
        if (violation.metadata.category == NlpConstraintCategory::PARTICIPANT_DISC ||
            violation.metadata.category == NlpConstraintCategory::STATIC_DISC) {
            report << "      actual_center_distance_m: "
                   << yamlNumber(violation.actual_distance) << '\n'
                   << "      required_center_distance_m: "
                   << yamlNumber(violation.required_distance) << '\n'
                   << "      distance_shortfall_m: "
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
