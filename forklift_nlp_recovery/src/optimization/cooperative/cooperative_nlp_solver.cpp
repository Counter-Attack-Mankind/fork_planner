#include "forklift_nlp_recovery/cooperative/cooperative_nlp_solver.h"

#include <casadi/casadi.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <limits>
#include <set>

#include "forklift_nlp_recovery/cooperative/dynamic_obca.h"
#include "forklift_nlp_recovery/common/kinematic_model.h"
#include "forklift_nlp_recovery/nlp_validator.h"

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

bool inside(const Pose2d& point, const Aabb& box, double tolerance = 1e-9) {
    return point.x >= box.min_x - tolerance && point.x <= box.max_x + tolerance &&
           point.y >= box.min_y - tolerance && point.y <= box.max_y + tolerance;
}

const VehicleInitialGuess* findGuess(const NlpInitialGuess& guess, int id) {
    const auto found = std::find_if(guess.vehicles.begin(), guess.vehicles.end(),
        [id](const VehicleInitialGuess& value) { return value.id == id; });
    return found == guess.vehicles.end() ? nullptr : &*found;
}

struct Constraints {
    std::vector<casadi::MX> expressions;
    std::vector<double> lower;
    std::vector<double> upper;
    std::vector<NlpConstraintMetadata> metadata;

    void add(const casadi::MX& expression, double low, double high,
             const NlpConstraintMetadata& info) {
        expressions.push_back(expression);
        lower.push_back(low);
        upper.push_back(high);
        metadata.push_back(info);
    }
};

struct SymbolicVehicle {
    casadi::MX state;
    casadi::MX control;
    const NlpVehicleInput* input = nullptr;
    const VehicleInitialGuess* guess = nullptr;
};

casadi::MX staticState(const StaticVehicleObstacle& obstacle) {
    return casadi::MX::vertcat({casadi::MX(obstacle.rear_axle_pose.x),
        casadi::MX(obstacle.rear_axle_pose.y), casadi::MX(obstacle.rear_axle_pose.yaw),
        casadi::MX(0.0), casadi::MX(0.0)});
}

VehicleParameters staticParameters(const StaticVehicleObstacle& obstacle) {
    VehicleParameters result;
    result.length = obstacle.length;
    result.width = obstacle.width;
    result.rear_axle_to_center = obstacle.rear_axle_to_center;
    return result;
}

}  // namespace

CooperativeNlpSolver::CooperativeNlpSolver(
    const NlpSolverConfig& config, const CorridorBuilder& corridor_builder)
    : config_(config), corridor_builder_(corridor_builder) {}

NlpSolveResult CooperativeNlpSolver::solve(
    const NlpOptimizationInput& input, const NlpInitialGuess& initial_guess) const {
    NlpSolveResult result;
    result.statistics.casadi_version = casadi::CasadiMeta::version();
    result.constraint_diagnostics.reason = "candidate solution unavailable";
    if (!input.valid || input.vehicles.size() != 3 ||
        initial_guess.vehicles.size() != 3 || !initial_guess.generated ||
        initial_guess.node_count < 2 || initial_guess.total_time <= 0.0) {
        result.reason = "cooperative NLP requires valid synchronized input for exactly three vehicles";
        return result;
    }
    if (config_.min_time_scale <= 0.0 ||
        config_.max_time_scale < config_.min_time_scale ||
        config_.validation_substeps == 0) {
        result.reason = "invalid cooperative solver configuration";
        return result;
    }

    std::set<int> participant_ids;
    for (const NlpVehicleInput& vehicle : input.vehicles) {
        const Pose2d start_center = bodyCenter(vehicle.frozen_initial_state.rear_axle_pose,
                                               vehicle.parameters.rear_axle_to_center);
        const Pose2d terminal_center = bodyCenter(vehicle.terminal.pose,
                                                  vehicle.parameters.rear_axle_to_center);
        if (!participant_ids.insert(vehicle.id).second || !vehicle.stc_valid ||
            !vehicle.terminal_valid || vehicle.initial_state_corrected ||
            !inside(start_center, vehicle.safe_stc) ||
            !inside(terminal_center, vehicle.safe_stc)) {
            result.reason = "fixed cooperative start/terminal is outside safe_stc or requires correction for V" +
                            std::to_string(vehicle.id);
            return result;
        }
    }
    for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
        if (participant_ids.count(obstacle.id) != 0) {
            result.reason = "participant/static vehicle ID sets overlap at V" +
                            std::to_string(obstacle.id);
            return result;
        }
    }

    const size_t nodes = initial_guess.node_count;
    std::vector<casadi::MX> variables;
    std::vector<double> initial_values, lower_bounds, upper_bounds;
    std::vector<NlpConstraintMetadata> variable_metadata;
    Constraints constraints;
    std::vector<SymbolicVehicle> symbolic;

    const casadi::MX total_time = casadi::MX::sym("coop_T");
    variables.push_back(total_time);
    initial_values.push_back(initial_guess.total_time);
    lower_bounds.push_back(std::max(0.1, initial_guess.total_time * config_.min_time_scale));
    upper_bounds.push_back(std::max(lower_bounds.back() + 0.1,
                                    initial_guess.total_time * config_.max_time_scale));
    variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND, -1, -1, 0,
                                 "total_time"});
    const casadi::MX dt = total_time / static_cast<double>(nodes - 1);
    casadi::MX objective = config_.time_weight * total_time;

    for (const NlpVehicleInput& vehicle : input.vehicles) {
        const VehicleInitialGuess* guess = findGuess(initial_guess, vehicle.id);
        if (guess == nullptr || guess->nodes.size() != nodes ||
            vehicle.parameters.wheel_base <= 0.0 || vehicle.parameters.length <= 0.0 ||
            vehicle.parameters.width <= 0.0 || vehicle.parameters.max_speed <= 0.0 ||
            vehicle.parameters.max_accel <= 0.0 || vehicle.parameters.max_decel <= 0.0 ||
            vehicle.parameters.max_steer_angle <= 0.0 ||
            vehicle.parameters.max_steer_rate <= 0.0) {
            result.reason = "incomplete cooperative vehicle input for V" +
                            std::to_string(vehicle.id);
            return result;
        }
        const double start_heading = vehicle.frozen_initial_state.rear_axle_pose.yaw;
        const double terminal_heading = equivalentAngle(vehicle.terminal.pose.yaw, start_heading);
        casadi::MX state = casadi::MX::sym("coop_X_" + std::to_string(vehicle.id), 5,
                                           static_cast<casadi_int>(nodes));
        casadi::MX control = casadi::MX::sym("coop_U_" + std::to_string(vehicle.id), 2,
                                             static_cast<casadi_int>(nodes - 1));
        variables.push_back(casadi::MX::reshape(state, 5 * nodes, 1));
        variables.push_back(casadi::MX::reshape(control, 2 * (nodes - 1), 1));
        static const char* components[] = {"rear_x", "rear_y", "heading",
                                            "signed_speed", "front_steer"};
        for (size_t node = 0; node < nodes; ++node) {
            const InitialGuessNode& value = guess->nodes[node];
            const double x0[] = {value.rear_axle_pose.x, value.rear_axle_pose.y,
                                 value.rear_axle_pose.yaw, 0.0, value.front_steer};
            for (size_t component = 0; component < 5; ++component) {
                initial_values.push_back(x0[component]);
                lower_bounds.push_back(-kInfinity);
                upper_bounds.push_back(kInfinity);
                variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
                    vehicle.id, -1, node, components[component]});
            }
            const size_t base = lower_bounds.size() - 5;
            lower_bounds[base + 3] = -vehicle.parameters.max_speed;
            upper_bounds[base + 3] = vehicle.parameters.max_speed;
            lower_bounds[base + 4] = -vehicle.parameters.max_steer_angle;
            upper_bounds[base + 4] = vehicle.parameters.max_steer_angle;
            if (node == 0) {
                const Pose2d& start = vehicle.frozen_initial_state.rear_axle_pose;
                lower_bounds[base] = upper_bounds[base] = start.x;
                lower_bounds[base + 1] = upper_bounds[base + 1] = start.y;
                lower_bounds[base + 2] = upper_bounds[base + 2] = start_heading;
                lower_bounds[base + 3] = upper_bounds[base + 3] = 0.0;
                if (vehicle.frozen_initial_state.steering_valid) {
                    lower_bounds[base + 4] = upper_bounds[base + 4] =
                        vehicle.frozen_initial_state.steering_angle;
                }
            } else if (node + 1 == nodes) {
                lower_bounds[base] = upper_bounds[base] = vehicle.terminal.pose.x;
                lower_bounds[base + 1] = upper_bounds[base + 1] = vehicle.terminal.pose.y;
                lower_bounds[base + 2] = upper_bounds[base + 2] = terminal_heading;
                lower_bounds[base + 3] = upper_bounds[base + 3] = 0.0;
            }
        }
        for (size_t interval = 0; interval + 1 < nodes; ++interval) {
            initial_values.push_back(0.0);
            initial_values.push_back(0.0);
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
            const casadi::MX current = state(casadi::Slice(), node);
            const casadi::MX center = symbolicBodyCenter(current,
                vehicle.parameters.rear_axle_to_center);
            constraints.add(center(0), vehicle.safe_stc.min_x, vehicle.safe_stc.max_x,
                {NlpConstraintCategory::AABB_NODE, vehicle.id, -1, node, "stc_x"});
            constraints.add(center(1), vehicle.safe_stc.min_y, vehicle.safe_stc.max_y,
                {NlpConstraintCategory::AABB_NODE, vehicle.id, -1, node, "stc_y"});
            if (node + 1 == nodes) continue;
            const casadi::MX u = control(casadi::Slice(), node);
            const casadi::MX predicted = integrateBicycleRk4(current, u, dt,
                                                       vehicle.parameters.wheel_base);
            static const char* rk4[] = {"rear_x", "rear_y", "heading",
                                        "signed_speed", "front_steer"};
            for (size_t component = 0; component < 5; ++component) {
                constraints.add(state(component, node + 1) - predicted(component), 0.0, 0.0,
                    {NlpConstraintCategory::RK4, vehicle.id, -1, node, rk4[component]});
            }
            const casadi::MX midpoint = integrateBicycleRk4(current, u, dt * 0.5,
                                                       vehicle.parameters.wheel_base);
            const casadi::MX midpoint_center = symbolicBodyCenter(midpoint,
                vehicle.parameters.rear_axle_to_center);
            constraints.add(midpoint_center(0), vehicle.safe_stc.min_x,
                vehicle.safe_stc.max_x, {NlpConstraintCategory::AABB_NODE,
                vehicle.id, -1, node, "midpoint_stc_x"});
            constraints.add(midpoint_center(1), vehicle.safe_stc.min_y,
                vehicle.safe_stc.max_y, {NlpConstraintCategory::AABB_NODE,
                vehicle.id, -1, node, "midpoint_stc_y"});
            objective += dt * (config_.acceleration_weight * u(0) * u(0) +
                               config_.steer_rate_weight * u(1) * u(1));
            if (node + 2 < nodes) {
                const casadi::MX next_u = control(casadi::Slice(), node + 1);
                const casadi::MX da = next_u(0) - u(0);
                const casadi::MX dw = next_u(1) - u(1);
                objective += config_.control_smooth_weight * (da * da + dw * dw);
            }
        }
        symbolic.push_back({state, control, &vehicle, guess});
    }

    const auto addObca = [&](const casadi::MX& first_state,
                             const casadi::MX& second_state,
                             const Pose2d& first_pose, const Pose2d& second_pose,
                             const VehicleParameters& first,
                             const VehicleParameters& second, int first_id,
                             int second_id, size_t node, const std::string& sample,
                             NlpConstraintCategory category) {
        addDynamicObca(first_state, second_state, first_pose, second_pose, first, second,
            first_id, second_id, node, sample, category, variables, initial_values,
            lower_bounds, upper_bounds, variable_metadata, constraints.expressions,
            constraints.lower, constraints.upper, constraints.metadata);
    };
    for (const SymbolicVehicle& moving : symbolic) {
        for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
            if (obstacle.length <= 0.0 || obstacle.width <= 0.0) {
                result.reason = "invalid static vehicle geometry for V" +
                                std::to_string(obstacle.id);
                return result;
            }
            const casadi::MX fixed = staticState(obstacle);
            const VehicleParameters fixed_parameters = staticParameters(obstacle);
            for (size_t node = 0; node < nodes; ++node) {
                addObca(moving.state(casadi::Slice(), node), fixed,
                    moving.guess->nodes[node].rear_axle_pose, obstacle.rear_axle_pose,
                    moving.input->parameters, fixed_parameters, moving.input->id,
                    obstacle.id, node, "node", NlpConstraintCategory::STATIC_OBCA);
                if (node + 1 == nodes) continue;
                const casadi::MX midpoint = integrateBicycleRk4(moving.state(casadi::Slice(), node),
                    moving.control(casadi::Slice(), node), dt * 0.5,
                    moving.input->parameters.wheel_base);
                addObca(midpoint, fixed, moving.guess->nodes[node].rear_axle_pose,
                    obstacle.rear_axle_pose, moving.input->parameters, fixed_parameters,
                    moving.input->id, obstacle.id, node, "midpoint",
                    NlpConstraintCategory::STATIC_OBCA);
            }
        }
    }
    for (size_t first = 0; first < symbolic.size(); ++first) {
        for (size_t second = first + 1; second < symbolic.size(); ++second) {
            const SymbolicVehicle& a = symbolic[first];
            const SymbolicVehicle& b = symbolic[second];
            for (size_t node = 0; node < nodes; ++node) {
                addObca(a.state(casadi::Slice(), node), b.state(casadi::Slice(), node),
                    a.guess->nodes[node].rear_axle_pose,
                    b.guess->nodes[node].rear_axle_pose, a.input->parameters,
                    b.input->parameters, a.input->id, b.input->id, node, "node",
                    NlpConstraintCategory::DYNAMIC_OBCA);
                if (node + 1 == nodes) continue;
                const casadi::MX midpoint_a = integrateBicycleRk4(a.state(casadi::Slice(), node),
                    a.control(casadi::Slice(), node), dt * 0.5,
                    a.input->parameters.wheel_base);
                const casadi::MX midpoint_b = integrateBicycleRk4(b.state(casadi::Slice(), node),
                    b.control(casadi::Slice(), node), dt * 0.5,
                    b.input->parameters.wheel_base);
                addObca(midpoint_a, midpoint_b, a.guess->nodes[node].rear_axle_pose,
                    b.guess->nodes[node].rear_axle_pose, a.input->parameters,
                    b.input->parameters, a.input->id, b.input->id, node, "midpoint",
                    NlpConstraintCategory::DYNAMIC_OBCA);
            }
        }
    }

    result.statistics.variable_count = initial_values.size();
    result.statistics.constraint_count = constraints.lower.size();
    const casadi::MX decision = casadi::MX::vertcat(variables);
    const casadi::MX constraint_vector = casadi::MX::vertcat(constraints.expressions);
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
        casadi::Function solver = casadi::nlpsol("cooperative_nlp_solver", "ipopt",
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
        const std::vector<double> decision_values = solution.at("x").get_elements();
        const std::vector<double> constraint_values = solution.at("g").get_elements();
        result.constraint_diagnostics = diagnoseNlpConstraints({constraint_values,
            constraints.lower, constraints.upper, constraints.metadata, decision_values,
            lower_bounds, upper_bounds, variable_metadata});
        result.statistics.max_constraint_violation = result.constraint_diagnostics.max_violation;
        result.solver_success = result.statistics.success;
        size_t offset = 0;
        result.total_time = decision_values[offset++];
        result.node_count = nodes;
        result.average_dt = result.total_time / static_cast<double>(nodes - 1);
        for (const SymbolicVehicle& value : symbolic) {
            OptimizedVehicleTrajectory trajectory;
            trajectory.id = value.input->id;
            trajectory.nodes.resize(nodes);
            for (size_t node = 0; node < nodes; ++node) {
                OptimizedTrajectoryNode& point = trajectory.nodes[node];
                point.index = node;
                point.time = result.average_dt * static_cast<double>(node);
                point.rear_axle_pose = {decision_values[offset], decision_values[offset + 1],
                    normalizeAngle(decision_values[offset + 2])};
                point.signed_speed = decision_values[offset + 3];
                point.front_steer = decision_values[offset + 4];
                point.aabb_index = -1;
                point.disc_center = bodyCenter(point.rear_axle_pose,
                    value.input->parameters.rear_axle_to_center);
                offset += 5;
            }
            for (size_t node = 0; node + 1 < nodes; ++node) {
                trajectory.nodes[node].acceleration = decision_values[offset++];
                trajectory.nodes[node].steer_rate = decision_values[offset++];
            }
            trajectory.nodes.back().acceleration = trajectory.nodes[nodes - 2].acceleration;
            trajectory.nodes.back().steer_rate = trajectory.nodes[nodes - 2].steer_rate;
            result.vehicles.push_back(std::move(trajectory));
        }
        result.validation = validateSolution(input, result, config_, corridor_builder_);
        result.validation_passed = result.validation.passed;
        if (!result.solver_success) {
            result.reason = "IPOPT did not report success: " + result.statistics.return_status;
        } else if (!result.validation_passed) {
            result.reason = "IPOPT succeeded; cooperative independent validation failed";
        } else {
            result.reason = "cooperative solver and independent validation succeeded";
        }
    } catch (const std::exception& error) {
        result.reason = std::string("CasADi/IPOPT exception: ") + error.what();
        result.statistics.return_status = "exception";
    }
    const double wall = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    const double cpu = static_cast<double>(std::clock() - cpu_started) / CLOCKS_PER_SEC;
    if (result.statistics.wall_time <= 0.0) result.statistics.wall_time = wall;
    if (result.statistics.cpu_time <= 0.0) result.statistics.cpu_time = cpu;
    return result;
}

}  // namespace forklift_nlp_recovery
