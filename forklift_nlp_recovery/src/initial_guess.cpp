#include "forklift_nlp_recovery/initial_guess.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <system_error>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double angleDistance(double first, double second) {
    return std::abs(normalizeAngle(first - second));
}

double poseDistance(const Pose2d& first, const Pose2d& second) {
    return std::hypot(first.x - second.x, first.y - second.y);
}

Pose2d interpolatePose(const Pose2d& first, const Pose2d& second, double ratio) {
    Pose2d result;
    result.x = first.x + ratio * (second.x - first.x);
    result.y = first.y + ratio * (second.y - first.y);
    result.yaw = normalizeAngle(first.yaw + ratio * normalizeAngle(second.yaw - first.yaw));
    return result;
}

PathPoint interpolatePath(const std::vector<PathPoint>& path, double query_s) {
    if (query_s <= path.front().s) return path.front();
    if (query_s >= path.back().s) return path.back();
    const auto upper = std::upper_bound(
        path.begin(), path.end(), query_s,
        [](double value, const PathPoint& point) { return value < point.s; });
    const size_t hi = static_cast<size_t>(std::distance(path.begin(), upper));
    const size_t lo = hi - 1;
    const double span = path[hi].s - path[lo].s;
    const double ratio = span > kEpsilon ? (query_s - path[lo].s) / span : 1.0;
    PathPoint result;
    result.pose = interpolatePose(path[lo].pose, path[hi].pose, ratio);
    result.s = query_s;
    result.type = ratio < 0.5 ? path[lo].type : path[hi].type;
    return result;
}

Pose2d discCenter(const Pose2d& rear, const VehicleParameters& parameters) {
    return {rear.x + parameters.rear_axle_to_center * std::cos(rear.yaw),
            rear.y + parameters.rear_axle_to_center * std::sin(rear.yaw), rear.yaw};
}

bool contains(const Aabb& box, const Pose2d& point, double tolerance) {
    return point.x >= box.min_x - tolerance && point.x <= box.max_x + tolerance &&
           point.y >= box.min_y - tolerance && point.y <= box.max_y + tolerance;
}

bool assignAabbs(VehicleInitialGuess& guess, const NlpVehicleInput& input,
                 double tolerance, std::string& reason) {
    if (input.corridor.empty()) {
        reason = "corridor has no AABB";
        return false;
    }
    const size_t node_count = guess.nodes.size();
    const size_t box_count = input.corridor.size();
    const int unreachable = std::numeric_limits<int>::max() / 4;
    std::vector<std::vector<int>> cost(node_count, std::vector<int>(box_count, unreachable));
    std::vector<std::vector<int>> predecessor(node_count, std::vector<int>(box_count, -1));

    for (size_t box = 0; box < box_count; ++box) {
        if (contains(input.corridor[box], guess.nodes.front().disc_center, tolerance)) {
            cost[0][box] = static_cast<int>(box);
        }
    }
    for (size_t node = 1; node < node_count; ++node) {
        for (size_t box = 0; box < box_count; ++box) {
            if (!contains(input.corridor[box], guess.nodes[node].disc_center, tolerance)) continue;
            for (size_t previous = 0; previous < box_count; ++previous) {
                if (cost[node - 1][previous] == unreachable || box < previous || box > previous + 1) continue;
                if (box != previous &&
                    !contains(input.corridor[box], guess.nodes[node - 1].disc_center, tolerance) &&
                    !contains(input.corridor[previous], guess.nodes[node].disc_center, tolerance)) {
                    continue;
                }
                const int candidate = cost[node - 1][previous] + (box == previous ? 0 : 1);
                if (candidate < cost[node][box]) {
                    cost[node][box] = candidate;
                    predecessor[node][box] = static_cast<int>(previous);
                }
            }
        }
    }

    const auto best = std::min_element(cost.back().begin(), cost.back().end());
    if (best == cost.back().end() || *best == unreachable) {
        size_t first_uncovered = node_count;
        for (size_t node = 0; node < node_count; ++node) {
            const bool covered = std::any_of(input.corridor.begin(), input.corridor.end(),
                [&](const Aabb& box) { return contains(box, guess.nodes[node].disc_center, tolerance); });
            if (!covered) {
                first_uncovered = node;
                break;
            }
        }
        std::ostringstream text;
        if (first_uncovered < node_count) text << "disc center at node " << first_uncovered << " is outside every AABB";
        else text << "no ordered assignment through adjacent AABB overlap";
        reason = text.str();
        return false;
    }

    int box = static_cast<int>(std::distance(cost.back().begin(), best));
    for (size_t reverse = node_count; reverse-- > 0;) {
        guess.nodes[reverse].aabb_index = box;
        if (reverse > 0) box = predecessor[reverse][static_cast<size_t>(box)];
    }
    return true;
}

double positiveViolation(double value, double limit) {
    return std::max(0.0, value - std::max(0.0, limit));
}

std::string isoTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t raw = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &raw);
#else
    localtime_r(&raw, &tm);
#endif
    std::ostringstream value;
    value << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S%z");
    return value.str();
}

}  // namespace

const char* guessValueSourceName(GuessValueSource source) {
    switch (source) {
        case GuessValueSource::SNAPSHOT_MAGNITUDE_PATH_SIGN:
            return "snapshot_magnitude+path_type_sign_estimate";
        case GuessValueSource::PATH_TIME_DIFFERENCE:
            return "path_time_difference";
        case GuessValueSource::PATH_GEOMETRY_ESTIMATE:
            return "path_geometry_estimate";
        case GuessValueSource::FINITE_DIFFERENCE:
            return "finite_difference";
        case GuessValueSource::TERMINAL_ZERO_ASSUMPTION:
            return "terminal_zero_assumption";
    }
    return "unknown";
}

InitialGuessGenerator::InitialGuessGenerator(const InitialGuessConfig& config)
    : config_(config) {}

NlpInitialGuess InitialGuessGenerator::generate(const NlpOptimizationInput& input) const {
    NlpInitialGuess result;
    result.node_count = config_.node_count;
    result.total_time = config_.total_time;
    if (!input.valid) {
        result.reason = "optimization input is invalid: " + input.reason;
        return result;
    }
    if (config_.node_count < 2) {
        result.reason = "initial_guess_nodes must be at least 2";
        return result;
    }
    if (!(config_.total_time > 0.0) || !std::isfinite(config_.total_time)) {
        result.reason = "initial_guess_total_time must be finite and positive";
        return result;
    }
    if (input.vehicles.empty()) {
        result.reason = "optimization input has no participant vehicles";
        return result;
    }

    const double dt = config_.total_time / static_cast<double>(config_.node_count - 1);
    result.time_grid.reserve(config_.node_count);
    for (size_t node = 0; node < config_.node_count; ++node) {
        result.time_grid.push_back(dt * static_cast<double>(node));
    }

    for (const NlpVehicleInput& input_vehicle : input.vehicles) {
        VehicleInitialGuess vehicle;
        vehicle.id = input_vehicle.id;
        if (!input_vehicle.corridor_valid || input_vehicle.original_path.size() < 2) {
            vehicle.reason = !input_vehicle.corridor_valid ? input_vehicle.corridor_reason : "original path has fewer than two points";
            result.vehicles.push_back(std::move(vehicle));
            result.reason = "V" + std::to_string(input_vehicle.id) + ": " + result.vehicles.back().reason;
            return result;
        }
        if (!(input_vehicle.parameters.wheel_base > 0.0) ||
            !(input_vehicle.parameters.disc_radius > 0.0)) {
            vehicle.reason = "invalid wheel base or disc radius";
            result.vehicles.push_back(std::move(vehicle));
            result.reason = "V" + std::to_string(input_vehicle.id) + ": invalid vehicle parameters";
            return result;
        }

        const double path_begin = std::clamp(input_vehicle.path_s,
                                              input_vehicle.original_path.front().s,
                                              input_vehicle.original_path.back().s);
        const double path_end = std::clamp(input_vehicle.terminal.path_s, path_begin,
                                            input_vehicle.original_path.back().s);
        const PathPoint projected_start = interpolatePath(input_vehicle.original_path, path_begin);
        const double connector_length = poseDistance(input_vehicle.initial_state.rear_axle_pose,
                                                     projected_start.pose);
        const double path_length = std::max(0.0, path_end - path_begin);
        const double total_length = connector_length + path_length;
        vehicle.nodes.resize(config_.node_count);

        for (size_t node = 0; node < config_.node_count; ++node) {
            InitialGuessNode& state = vehicle.nodes[node];
            state.index = node;
            state.time = result.time_grid[node];
            const double ratio = static_cast<double>(node) /
                                 static_cast<double>(config_.node_count - 1);
            const double progress = ratio * total_length;
            if (node == 0) {
                state.rear_axle_pose = input_vehicle.initial_state.rear_axle_pose;
                state.path_s = path_begin;
                state.path_type = projected_start.type;
            } else if (node + 1 == config_.node_count) {
                state.rear_axle_pose = input_vehicle.terminal.pose;
                state.path_s = input_vehicle.terminal.path_s;
                state.path_type = input_vehicle.terminal.type;
            } else if (connector_length > kEpsilon && progress < connector_length) {
                state.rear_axle_pose = interpolatePose(
                    input_vehicle.initial_state.rear_axle_pose, projected_start.pose,
                    progress / connector_length);
                state.path_s = path_begin;
                state.path_type = projected_start.type;
            } else {
                const double path_progress = std::max(0.0, progress - connector_length);
                const PathPoint sample = interpolatePath(
                    input_vehicle.original_path, std::min(path_end, path_begin + path_progress));
                state.rear_axle_pose = sample.pose;
                state.path_s = sample.s;
                state.path_type = sample.type;
            }
            state.disc_center = discCenter(state.rear_axle_pose, input_vehicle.parameters);
        }

        for (size_t node = 0; node < config_.node_count; ++node) {
            InitialGuessNode& state = vehicle.nodes[node];
            const double direction = state.path_type == 1 ? -1.0 : 1.0;
            if (node == 0) {
                state.signed_speed = direction * std::abs(input_vehicle.initial_state.longitudinal_speed);
                state.speed_source = GuessValueSource::SNAPSHOT_MAGNITUDE_PATH_SIGN;
            } else if (node + 1 == config_.node_count) {
                state.signed_speed = 0.0;
                state.speed_source = GuessValueSource::TERMINAL_ZERO_ASSUMPTION;
            } else {
                const double distance = poseDistance(vehicle.nodes[node - 1].rear_axle_pose,
                                                     vehicle.nodes[node + 1].rear_axle_pose);
                state.signed_speed = direction * distance / (2.0 * dt);
                state.speed_source = GuessValueSource::PATH_TIME_DIFFERENCE;
            }
        }
        for (size_t node = 0; node + 1 < config_.node_count; ++node) {
            if (vehicle.nodes[node].path_type != vehicle.nodes[node + 1].path_type) {
                vehicle.nodes[node].signed_speed = 0.0;
                vehicle.nodes[node + 1].signed_speed = 0.0;
            }
        }

        for (size_t node = 0; node < config_.node_count; ++node) {
            InitialGuessNode& state = vehicle.nodes[node];
            const size_t lo = node == 0 ? 0 : node - 1;
            const size_t hi = node + 1 < config_.node_count ? node + 1 : node;
            const double distance = poseDistance(vehicle.nodes[lo].rear_axle_pose,
                                                 vehicle.nodes[hi].rear_axle_pose);
            const double direction = vehicle.nodes[node].path_type == 1 ? -1.0 : 1.0;
            const bool crosses_cusp = vehicle.nodes[lo].path_type != vehicle.nodes[hi].path_type;
            if (distance > kEpsilon && !crosses_cusp) {
                const double heading_change = normalizeAngle(
                    vehicle.nodes[hi].rear_axle_pose.yaw - vehicle.nodes[lo].rear_axle_pose.yaw);
                state.front_steer = std::atan(input_vehicle.parameters.wheel_base *
                                              heading_change / (direction * distance));
            } else {
                state.front_steer = 0.0;
            }
            state.steer_source = GuessValueSource::PATH_GEOMETRY_ESTIMATE;
        }

        for (size_t node = 0; node < config_.node_count; ++node) {
            const size_t lo = node == 0 ? 0 : node - 1;
            const size_t hi = node + 1 < config_.node_count ? node + 1 : node;
            const double span = static_cast<double>(hi - lo) * dt;
            vehicle.nodes[node].acceleration = span > 0.0 ?
                (vehicle.nodes[hi].signed_speed - vehicle.nodes[lo].signed_speed) / span : 0.0;
            vehicle.nodes[node].steer_rate = span > 0.0 ?
                normalizeAngle(vehicle.nodes[hi].front_steer - vehicle.nodes[lo].front_steer) / span : 0.0;
        }

        std::string assignment_reason;
        vehicle.aabb_assignment_valid = assignAabbs(
            vehicle, input_vehicle, config_.containment_tolerance, assignment_reason);
        if (!vehicle.aabb_assignment_valid) {
            vehicle.reason = assignment_reason;
            result.vehicles.push_back(std::move(vehicle));
            result.reason = "V" + std::to_string(input_vehicle.id) + " AABB assignment failed: " + assignment_reason;
            return result;
        }

        auto& diagnostics = vehicle.diagnostics;
        diagnostics.start_position_error = poseDistance(
            vehicle.nodes.front().rear_axle_pose, input_vehicle.initial_state.rear_axle_pose);
        diagnostics.start_heading_error = angleDistance(
            vehicle.nodes.front().rear_axle_pose.yaw, input_vehicle.initial_state.rear_axle_pose.yaw);
        diagnostics.start_speed_magnitude_error = std::abs(
            std::abs(vehicle.nodes.front().signed_speed) -
            std::abs(input_vehicle.initial_state.longitudinal_speed));
        diagnostics.terminal_position_error = poseDistance(
            vehicle.nodes.back().rear_axle_pose, input_vehicle.terminal.pose);
        diagnostics.terminal_heading_error = angleDistance(
            vehicle.nodes.back().rear_axle_pose.yaw, input_vehicle.terminal.pose.yaw);
        for (size_t node = 0; node < config_.node_count; ++node) {
            InitialGuessNode& state = vehicle.nodes[node];
            bool bounds_violated = false;
            const double speed_violation = positiveViolation(std::abs(state.signed_speed), input_vehicle.parameters.max_speed);
            const double steer_violation = positiveViolation(std::abs(state.front_steer), input_vehicle.parameters.max_steer_angle);
            const double accel_limit = state.acceleration >= 0.0 ? input_vehicle.parameters.max_accel : input_vehicle.parameters.max_decel;
            const double accel_violation = positiveViolation(std::abs(state.acceleration), accel_limit);
            const double rate_violation = positiveViolation(std::abs(state.steer_rate), input_vehicle.parameters.max_steer_rate);
            diagnostics.max_speed_violation = std::max(diagnostics.max_speed_violation, speed_violation);
            diagnostics.max_steer_violation = std::max(diagnostics.max_steer_violation, steer_violation);
            diagnostics.max_acceleration_violation = std::max(diagnostics.max_acceleration_violation, accel_violation);
            diagnostics.max_steer_rate_violation = std::max(diagnostics.max_steer_rate_violation, rate_violation);
            bounds_violated = speed_violation > kEpsilon || steer_violation > kEpsilon ||
                              accel_violation > kEpsilon || rate_violation > kEpsilon;
            if (bounds_violated) ++diagnostics.bound_violation_nodes;

            if (node + 1 < config_.node_count) {
                const InitialGuessNode& next = vehicle.nodes[node + 1];
                const double predicted_x = state.rear_axle_pose.x + dt * state.signed_speed * std::cos(state.rear_axle_pose.yaw);
                const double predicted_y = state.rear_axle_pose.y + dt * state.signed_speed * std::sin(state.rear_axle_pose.yaw);
                const double predicted_yaw = state.rear_axle_pose.yaw + dt * state.signed_speed /
                    input_vehicle.parameters.wheel_base * std::tan(state.front_steer);
                state.kinematic_position_residual = std::hypot(
                    next.rear_axle_pose.x - predicted_x, next.rear_axle_pose.y - predicted_y);
                state.kinematic_heading_residual = angleDistance(next.rear_axle_pose.yaw, predicted_yaw);
                diagnostics.max_kinematic_position_residual = std::max(
                    diagnostics.max_kinematic_position_residual, state.kinematic_position_residual);
                diagnostics.max_kinematic_heading_residual = std::max(
                    diagnostics.max_kinematic_heading_residual, state.kinematic_heading_residual);
            }
        }
        vehicle.valid = true;
        vehicle.reason = "ok";
        result.diagnostics.max_kinematic_position_residual = std::max(
            result.diagnostics.max_kinematic_position_residual,
            diagnostics.max_kinematic_position_residual);
        result.diagnostics.max_kinematic_heading_residual = std::max(
            result.diagnostics.max_kinematic_heading_residual,
            diagnostics.max_kinematic_heading_residual);
        result.diagnostics.bound_violation_nodes += diagnostics.bound_violation_nodes;
        result.vehicles.push_back(std::move(vehicle));
    }

    result.diagnostics.minimum_participant_clearance = std::numeric_limits<double>::infinity();
    result.diagnostics.minimum_static_clearance = std::numeric_limits<double>::infinity();
    for (size_t first = 0; first < result.vehicles.size(); ++first) {
        for (size_t second = first + 1; second < result.vehicles.size(); ++second) {
            const double required = input.vehicles[first].parameters.disc_radius +
                                    input.vehicles[second].parameters.disc_radius;
            for (size_t node = 0; node < config_.node_count; ++node) {
                const double distance = poseDistance(result.vehicles[first].nodes[node].disc_center,
                                                     result.vehicles[second].nodes[node].disc_center);
                const double clearance = distance - required;
                result.diagnostics.minimum_participant_clearance = std::min(
                    result.diagnostics.minimum_participant_clearance, clearance);
                if (clearance < 0.0) {
                    ++result.diagnostics.participant_collision_nodes;
                    result.diagnostics.collisions.push_back({node, result.time_grid[node],
                        result.vehicles[first].id, result.vehicles[second].id, false,
                        distance, required, clearance});
                }
            }
        }
    }
    for (size_t participant = 0; participant < result.vehicles.size(); ++participant) {
        for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
            const double required = input.vehicles[participant].parameters.disc_radius + obstacle.disc_radius;
            for (size_t node = 0; node < config_.node_count; ++node) {
                const double distance = poseDistance(result.vehicles[participant].nodes[node].disc_center,
                                                     obstacle.disc_center);
                const double clearance = distance - required;
                result.diagnostics.minimum_static_clearance = std::min(
                    result.diagnostics.minimum_static_clearance, clearance);
                if (clearance < 0.0) {
                    ++result.diagnostics.static_collision_nodes;
                    result.diagnostics.collisions.push_back({node, result.time_grid[node],
                        result.vehicles[participant].id, obstacle.id, true,
                        distance, required, clearance});
                }
            }
        }
    }

    result.generated = true;
    const bool dynamics_close = result.diagnostics.max_kinematic_position_residual <= 1e-3 &&
                                result.diagnostics.max_kinematic_heading_residual <= 1e-3;
    result.constraints_satisfied = result.diagnostics.bound_violation_nodes == 0 &&
        result.diagnostics.participant_collision_nodes == 0 &&
        result.diagnostics.static_collision_nodes == 0 && dynamics_close;
    result.reason = result.constraints_satisfied ? "generated and constraint checks passed" :
        "generated with constraint violations; retained for NLP initialization";
    return result;
}

CaseSaveResult saveInitialGuessCase(const OptimizationInputBuildResult& input,
                                    const NlpInitialGuess& guess,
                                    const std::string& results_directory) {
    CaseSaveResult result;
    if (!guess.generated) {
        result.reason = "initial guess was not generated";
        return result;
    }
    if (results_directory.empty()) {
        result.reason = "results directory is empty";
        return result;
    }
    std::error_code error;
    const std::filesystem::path root(results_directory);
    std::filesystem::create_directories(root, error);
    if (error) {
        result.reason = "cannot create results directory: " + error.message();
        return result;
    }

    std::filesystem::path case_path;
    for (unsigned int number = 1; number < 1000000; ++number) {
        std::ostringstream name;
        name << "case_" << std::setw(4) << std::setfill('0') << number;
        const std::filesystem::path candidate = root / name.str();
        error.clear();
        if (std::filesystem::create_directory(candidate, error)) {
            result.case_id = name.str();
            case_path = candidate;
            break;
        }
        if (error) {
            result.reason = "cannot create case directory: " + error.message();
            return result;
        }
    }
    if (case_path.empty()) {
        result.reason = "no available case number";
        return result;
    }

    std::string write_error;
    if (!writeOptimizationInputDebugFile(input,
            (case_path / "optimization_input.yaml").string(), write_error)) {
        result.reason = write_error;
        return result;
    }
    std::ofstream csv(case_path / "initial_guess.csv", std::ios::trunc);
    if (!csv) {
        result.reason = "cannot open initial_guess.csv";
        return result;
    }
    const std::string generated_at = isoTimestamp();
    csv << "case_id,generated_at,vehicle_id,node_index,time,path_s,path_type,"
           "rear_x,rear_y,heading,signed_speed,front_steer,acceleration,steer_rate,"
           "aabb_index,disc_center_x,disc_center_y,speed_source,steer_source,"
           "acceleration_source,steer_rate_source,kinematic_position_residual,"
           "kinematic_heading_residual\n";
    csv << std::fixed << std::setprecision(9);
    for (const VehicleInitialGuess& vehicle : guess.vehicles) {
        for (const InitialGuessNode& node : vehicle.nodes) {
            csv << result.case_id << ',' << generated_at << ',' << vehicle.id << ','
                << node.index << ',' << node.time << ',' << node.path_s << ','
                << static_cast<int>(node.path_type) << ',' << node.rear_axle_pose.x << ','
                << node.rear_axle_pose.y << ',' << node.rear_axle_pose.yaw << ','
                << node.signed_speed << ',' << node.front_steer << ','
                << node.acceleration << ',' << node.steer_rate << ','
                << node.aabb_index << ',' << node.disc_center.x << ','
                << node.disc_center.y << ',' << guessValueSourceName(node.speed_source) << ','
                << guessValueSourceName(node.steer_source) << ','
                << guessValueSourceName(node.acceleration_source) << ','
                << guessValueSourceName(node.steer_rate_source) << ','
                << node.kinematic_position_residual << ','
                << node.kinematic_heading_residual << '\n';
        }
    }
    if (!csv) {
        result.reason = "failed while writing initial_guess.csv";
        return result;
    }
    result.success = true;
    result.case_directory = case_path.string();
    result.reason = "ok";
    return result;
}

}  // namespace forklift_nlp_recovery
