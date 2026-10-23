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

double normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
double angleDistance(double first, double second) { return std::abs(normalizeAngle(first - second)); }
double poseDistance(const Pose2d& first, const Pose2d& second) {
    return std::hypot(first.x - second.x, first.y - second.y);
}

Pose2d interpolatePose(const Pose2d& first, const Pose2d& second, double ratio) {
    return {first.x + ratio * (second.x - first.x),
            first.y + ratio * (second.y - first.y),
            normalizeAngle(first.yaw + ratio * normalizeAngle(second.yaw - first.yaw))};
}

PathPoint interpolatePath(const std::vector<PathPoint>& path, double query_s) {
    if (query_s <= path.front().s) return path.front();
    if (query_s >= path.back().s) return path.back();
    const auto upper = std::upper_bound(path.begin(), path.end(), query_s,
        [](double value, const PathPoint& point) { return value < point.s; });
    const size_t hi = static_cast<size_t>(std::distance(path.begin(), upper));
    const size_t lo = hi - 1;
    const double span = path[hi].s - path[lo].s;
    const double ratio = span > kEpsilon ? (query_s - path[lo].s) / span : 1.0;
    return {interpolatePose(path[lo].pose, path[hi].pose, ratio), query_s,
            ratio < 0.5 ? path[lo].type : path[hi].type};
}

struct DirectionSegment {
    double begin_distance = 0.0;
    double length = 0.0;
    uint8_t type = 0;
};

struct ReferenceMotion {
    double path_begin = 0.0;
    double path_end = 0.0;
    PathPoint projected_start;
    double connector_length = 0.0;
    double path_length = 0.0;
    double total_length = 0.0;
    size_t direction_changes = 0;
    std::vector<DirectionSegment> segments;
};

void appendDirectionSegment(ReferenceMotion& reference, double length, uint8_t type) {
    if (length <= kEpsilon) return;
    if (!reference.segments.empty() && reference.segments.back().type == type) {
        reference.segments.back().length += length;
    } else {
        reference.segments.push_back({reference.total_length, length, type});
    }
    reference.total_length += length;
}

bool buildReferenceMotion(const NlpVehicleInput& vehicle, ReferenceMotion& reference,
                          std::string& error) {
    if (!vehicle.corridor_valid) {
        error = vehicle.corridor_reason;
        return false;
    }
    if (vehicle.original_path.size() < 2) {
        error = "original path has fewer than two points";
        return false;
    }
    for (size_t i = 0; i < vehicle.original_path.size(); ++i) {
        const PathPoint& point = vehicle.original_path[i];
        if (!std::isfinite(point.s) || !std::isfinite(point.pose.x) ||
            !std::isfinite(point.pose.y) || !std::isfinite(point.pose.yaw) ||
            (i > 0 && point.s + kEpsilon < vehicle.original_path[i - 1].s)) {
            error = "original path contains invalid or non-monotonic geometry";
            return false;
        }
    }
    const double first_s = vehicle.original_path.front().s;
    const double last_s = vehicle.original_path.back().s;
    if (vehicle.path_s < first_s - kEpsilon || vehicle.path_s > last_s + kEpsilon) {
        error = "current path_s is outside the original path";
        return false;
    }
    if (vehicle.terminal.path_s < vehicle.path_s - kEpsilon ||
        vehicle.terminal.path_s > last_s + kEpsilon ||
        vehicle.terminal.path_gen != vehicle.path_gen) {
        error = "terminal does not belong to the forward portion of the frozen path";
        return false;
    }
    reference.path_begin = vehicle.path_s;
    reference.path_end = vehicle.terminal.path_s;
    reference.projected_start = interpolatePath(vehicle.original_path, reference.path_begin);
    const PathPoint expected_terminal = interpolatePath(vehicle.original_path, reference.path_end);
    if (poseDistance(expected_terminal.pose, vehicle.terminal.pose) > 1e-4 ||
        angleDistance(expected_terminal.pose.yaw, vehicle.terminal.pose.yaw) > 1e-4) {
        error = "terminal pose does not match terminal path_s on the frozen path";
        return false;
    }
    reference.connector_length = poseDistance(vehicle.initial_state.rear_axle_pose,
                                               reference.projected_start.pose);
    reference.path_length = reference.path_end - reference.path_begin;
    appendDirectionSegment(reference, reference.connector_length, reference.projected_start.type);
    std::vector<double> boundaries{reference.path_begin, reference.path_end};
    for (size_t i = 1; i < vehicle.original_path.size(); ++i) {
        if (vehicle.original_path[i - 1].type == vehicle.original_path[i].type) continue;
        const double transition = 0.5 * (vehicle.original_path[i - 1].s + vehicle.original_path[i].s);
        if (transition > reference.path_begin + kEpsilon &&
            transition < reference.path_end - kEpsilon) boundaries.push_back(transition);
    }
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
        [](double first, double second) { return std::abs(first - second) <= kEpsilon; }), boundaries.end());
    for (size_t i = 1; i < boundaries.size(); ++i) {
        const double midpoint = 0.5 * (boundaries[i] + boundaries[i - 1]);
        appendDirectionSegment(reference, boundaries[i] - boundaries[i - 1],
                               interpolatePath(vehicle.original_path, midpoint).type);
    }
    reference.direction_changes = reference.segments.empty() ? 0 : reference.segments.size() - 1;
    return true;
}

struct SegmentProfile {
    double begin_distance = 0.0;
    double length = 0.0;
    uint8_t type = 0;
    double begin_time = 0.0;
    double duration = 0.0;
    double accel_time = 0.0;
    double cruise_time = 0.0;
    double peak_speed = 0.0;
};

bool estimateTravelTime(const NlpVehicleInput& vehicle, const ReferenceMotion& reference,
                        double& estimated_time, std::string& error) {
    const VehicleParameters& p = vehicle.parameters;
    if (!(p.max_speed > 0.0) || !(p.max_accel > 0.0) || !(p.max_decel > 0.0) ||
        !std::isfinite(p.max_speed) || !std::isfinite(p.max_accel) || !std::isfinite(p.max_decel)) {
        error = "max_speed, max_accel and max_decel must be finite and positive";
        return false;
    }
    estimated_time = 0.0;
    double start_speed = std::abs(vehicle.initial_state.longitudinal_speed);
    for (const DirectionSegment& segment : reference.segments) {
        const double stopping_distance = start_speed * start_speed / (2.0 * p.max_decel);
        if (segment.length + 1e-8 < stopping_distance) {
            estimated_time += start_speed / p.max_decel;
            start_speed = 0.0;
            continue;
        }
        const double speed_cap = std::max(p.max_speed, start_speed);
        const double peak_squared = (2.0 * p.max_accel * p.max_decel * segment.length +
                                     p.max_decel * start_speed * start_speed) /
                                    (p.max_accel + p.max_decel);
        const double peak_speed = std::sqrt(std::max(0.0, peak_squared));
        if (peak_speed <= speed_cap + kEpsilon) {
            estimated_time += std::max(0.0, peak_speed - start_speed) / p.max_accel;
            estimated_time += peak_speed / p.max_decel;
        } else {
            const double acceleration_distance = std::max(0.0,
                speed_cap * speed_cap - start_speed * start_speed) / (2.0 * p.max_accel);
            const double deceleration_distance = speed_cap * speed_cap / (2.0 * p.max_decel);
            const double cruise_distance = std::max(0.0,
                segment.length - acceleration_distance - deceleration_distance);
            estimated_time += std::max(0.0, speed_cap - start_speed) / p.max_accel;
            estimated_time += cruise_distance / speed_cap + speed_cap / p.max_decel;
        }
        start_speed = 0.0;
    }
    return estimated_time > kEpsilon;
}

bool buildTimeProfile(const NlpVehicleInput& vehicle, const ReferenceMotion& reference,
                      std::vector<SegmentProfile>& profiles, double& profile_time,
                      std::string& error) {
    const VehicleParameters& p = vehicle.parameters;
    if (!(p.max_speed > 0.0) || !(p.max_accel > 0.0) || !(p.max_decel > 0.0) ||
        !std::isfinite(p.max_speed) || !std::isfinite(p.max_accel) || !std::isfinite(p.max_decel)) {
        error = "max_speed, max_accel and max_decel must be finite and positive";
        return false;
    }
    profile_time = 0.0;
    for (const DirectionSegment& segment : reference.segments) {
        const double triangular_peak = std::sqrt(2.0 * segment.length * p.max_accel * p.max_decel /
                                                  (p.max_accel + p.max_decel));
        const double peak = std::min(p.max_speed, triangular_peak);
        const double accel_time = peak / p.max_accel;
        const double decel_time = peak / p.max_decel;
        const double accel_distance = 0.5 * p.max_accel * accel_time * accel_time;
        const double decel_distance = 0.5 * p.max_decel * decel_time * decel_time;
        const double cruise_distance = std::max(0.0, segment.length - accel_distance - decel_distance);
        const double cruise_time = cruise_distance / peak;
        const double duration = accel_time + cruise_time + decel_time;
        profiles.push_back({segment.begin_distance, segment.length, segment.type,
                            profile_time, duration, accel_time, cruise_time, peak});
        profile_time += duration;
    }
    if (profiles.empty() || profile_time <= kEpsilon) {
        error = "reference motion has zero travel time";
        return false;
    }
    return true;
}

double profileDistance(const SegmentProfile& profile, const VehicleParameters& p,
                       double local_time) {
    const double t = std::clamp(local_time - profile.begin_time, 0.0, profile.duration);
    if (t <= profile.accel_time) return profile.begin_distance + 0.5 * p.max_accel * t * t;
    const double accel_distance = 0.5 * p.max_accel * profile.accel_time * profile.accel_time;
    if (t <= profile.accel_time + profile.cruise_time) {
        return profile.begin_distance + accel_distance +
               profile.peak_speed * (t - profile.accel_time);
    }
    const double decel_t = t - profile.accel_time - profile.cruise_time;
    const double cruise_distance = profile.peak_speed * profile.cruise_time;
    return std::min(profile.begin_distance + profile.length,
        profile.begin_distance + accel_distance + cruise_distance +
        profile.peak_speed * decel_t - 0.5 * p.max_decel * decel_t * decel_t);
}

double sampleProfile(const std::vector<SegmentProfile>& profiles,
                     const VehicleParameters& parameters, double local_time) {
    const auto found = std::find_if(profiles.begin(), profiles.end(), [&](const SegmentProfile& profile) {
        return local_time <= profile.begin_time + profile.duration + kEpsilon;
    });
    if (found == profiles.end()) return profiles.back().begin_distance + profiles.back().length;
    return profileDistance(*found, parameters, local_time);
}

Pose2d discCenter(const Pose2d& rear, const VehicleParameters& parameters) {
    return {rear.x + parameters.rear_axle_to_center * std::cos(rear.yaw),
            rear.y + parameters.rear_axle_to_center * std::sin(rear.yaw), rear.yaw};
}

void sampleReferenceNode(const NlpVehicleInput& vehicle, const ReferenceMotion& reference,
                         double progress, InitialGuessNode& node) {
    if (node.index == 0) {
        node.rear_axle_pose = vehicle.initial_state.rear_axle_pose;
        node.path_s = reference.path_begin;
        node.path_type = reference.projected_start.type;
    } else if (progress >= reference.total_length - kEpsilon) {
        node.rear_axle_pose = vehicle.terminal.pose;
        node.path_s = vehicle.terminal.path_s;
        node.path_type = vehicle.terminal.type;
    } else if (reference.connector_length > kEpsilon && progress < reference.connector_length) {
        node.rear_axle_pose = interpolatePose(vehicle.initial_state.rear_axle_pose,
            reference.projected_start.pose, progress / reference.connector_length);
        node.path_s = reference.path_begin;
        node.path_type = reference.projected_start.type;
    } else {
        const double path_progress = std::max(0.0, progress - reference.connector_length);
        const PathPoint sample = interpolatePath(vehicle.original_path,
            std::min(reference.path_end, reference.path_begin + path_progress));
        node.rear_axle_pose = sample.pose;
        node.path_s = sample.s;
        node.path_type = sample.type;
    }
    node.disc_center = discCenter(node.rear_axle_pose, vehicle.parameters);
}

double positiveViolation(double value, double limit) {
    return std::max(0.0, value - std::max(0.0, limit));
}

void recomputeDerived(VehicleInitialGuess& guess, const NlpVehicleInput& input, double dt,
                      size_t waiting_begin) {
    const size_t count = guess.nodes.size();
    waiting_begin = std::min(waiting_begin, count);
    for (InitialGuessNode& node : guess.nodes) {
        node.disc_center = discCenter(node.rear_axle_pose, input.parameters);
    }
    for (size_t i = 0; i < count; ++i) {
        InitialGuessNode& node = guess.nodes[i];
        if (i == 0 || i + 1 == count) {
            node.signed_speed = 0.0;
            node.speed_source = GuessValueSource::TERMINAL_ZERO_ASSUMPTION;
        } else {
            const double distance = poseDistance(guess.nodes[i - 1].rear_axle_pose,
                                                 guess.nodes[i + 1].rear_axle_pose);
            node.signed_speed = (node.path_type == 1 ? -1.0 : 1.0) * distance / (2.0 * dt);
            node.speed_source = GuessValueSource::PATH_TIME_DIFFERENCE;
        }
        const size_t lo = i == 0 ? 0 : i - 1;
        const size_t hi = i + 1 < count ? i + 1 : i;
        const double distance = poseDistance(guess.nodes[lo].rear_axle_pose,
                                             guess.nodes[hi].rear_axle_pose);
        if (distance > kEpsilon) {
            const double direction = node.path_type == 1 ? -1.0 : 1.0;
            const double heading_change = normalizeAngle(guess.nodes[hi].rear_axle_pose.yaw -
                                                         guess.nodes[lo].rear_axle_pose.yaw);
            node.front_steer = std::atan(input.parameters.wheel_base * heading_change /
                                         (direction * distance));
        } else {
            node.front_steer = 0.0;
        }
    }
    if (waiting_begin < count) {
        const double waiting_steer = waiting_begin > 0 ?
            guess.nodes[waiting_begin - 1].front_steer : guess.nodes[waiting_begin].front_steer;
        for (size_t i = waiting_begin; i < count; ++i) {
            guess.nodes[i].signed_speed = 0.0;
            guess.nodes[i].speed_source = GuessValueSource::TERMINAL_ZERO_ASSUMPTION;
            guess.nodes[i].front_steer = waiting_steer;
        }
    }
    for (size_t i = 0; i < count; ++i) {
        const size_t lo = i == 0 ? 0 : i - 1;
        const size_t hi = i + 1 < count ? i + 1 : i;
        const double span = static_cast<double>(hi - lo) * dt;
        guess.nodes[i].acceleration = span > 0.0 ?
            (guess.nodes[hi].signed_speed - guess.nodes[lo].signed_speed) / span : 0.0;
        guess.nodes[i].steer_rate = span > 0.0 ?
            normalizeAngle(guess.nodes[hi].front_steer - guess.nodes[lo].front_steer) / span : 0.0;
        if (i >= waiting_begin) {
            guess.nodes[i].acceleration = 0.0;
            guess.nodes[i].steer_rate = 0.0;
        }
    }
}

void updateVehicleDiagnostics(VehicleInitialGuess& guess, const NlpVehicleInput& input,
                              double dt, double tolerance) {
    auto& d = guess.diagnostics;
    d.start_position_error = poseDistance(guess.nodes.front().rear_axle_pose,
                                          input.initial_state.rear_axle_pose);
    d.start_heading_error = angleDistance(guess.nodes.front().rear_axle_pose.yaw,
                                          input.initial_state.rear_axle_pose.yaw);
    d.start_speed_magnitude_error = std::abs(guess.nodes.front().signed_speed);
    d.terminal_position_error = poseDistance(guess.nodes.back().rear_axle_pose, input.terminal.pose);
    d.terminal_heading_error = angleDistance(guess.nodes.back().rear_axle_pose.yaw,
                                             input.terminal.pose.yaw);
    for (size_t i = 0; i < guess.nodes.size(); ++i) {
        InitialGuessNode& node = guess.nodes[i];
        const double speed_violation = positiveViolation(std::abs(node.signed_speed), input.parameters.max_speed);
        const double steer_violation = positiveViolation(std::abs(node.front_steer), input.parameters.max_steer_angle);
        const double accel_limit = node.acceleration >= 0.0 ? input.parameters.max_accel : input.parameters.max_decel;
        const double accel_violation = positiveViolation(std::abs(node.acceleration), accel_limit);
        const double rate_violation = positiveViolation(std::abs(node.steer_rate), input.parameters.max_steer_rate);
        d.max_speed_violation = std::max(d.max_speed_violation, speed_violation);
        d.max_steer_violation = std::max(d.max_steer_violation, steer_violation);
        d.max_acceleration_violation = std::max(d.max_acceleration_violation, accel_violation);
        d.max_steer_rate_violation = std::max(d.max_steer_rate_violation, rate_violation);
        if (speed_violation > kEpsilon || steer_violation > kEpsilon ||
            accel_violation > kEpsilon || rate_violation > kEpsilon) ++d.bound_violation_nodes;
        if (i < input.corridor.size()) {
            const Aabb& box = input.corridor[i];
            const double violation = std::max({box.min_x - node.disc_center.x,
                node.disc_center.x - box.max_x, box.min_y - node.disc_center.y,
                node.disc_center.y - box.max_y, 0.0});
            d.max_aabb_violation = std::max(d.max_aabb_violation, violation);
            if (violation > tolerance) ++d.aabb_violation_nodes;
        }
        if (i + 1 < guess.nodes.size()) {
            const InitialGuessNode& next = guess.nodes[i + 1];
            const double predicted_x = node.rear_axle_pose.x + dt * node.signed_speed * std::cos(node.rear_axle_pose.yaw);
            const double predicted_y = node.rear_axle_pose.y + dt * node.signed_speed * std::sin(node.rear_axle_pose.yaw);
            const double predicted_yaw = node.rear_axle_pose.yaw + dt * node.signed_speed /
                input.parameters.wheel_base * std::tan(node.front_steer);
            node.kinematic_position_residual = std::hypot(next.rear_axle_pose.x - predicted_x,
                                                         next.rear_axle_pose.y - predicted_y);
            node.kinematic_heading_residual = angleDistance(next.rear_axle_pose.yaw, predicted_yaw);
            d.max_kinematic_position_residual = std::max(d.max_kinematic_position_residual,
                                                         node.kinematic_position_residual);
            d.max_kinematic_heading_residual = std::max(d.max_kinematic_heading_residual,
                                                        node.kinematic_heading_residual);
        }
    }
}

void filterStaticVehicles(NlpOptimizationInput& input) {
    std::vector<StaticVehicleObstacle> selected;
    for (StaticVehicleObstacle obstacle : input.static_vehicles) {
        obstacle.influencing_vehicle_ids.clear();
        obstacle.min_distance_to_corridor = std::numeric_limits<double>::infinity();
        obstacle.influence_distance = 0.0;
        for (const NlpVehicleInput& participant : input.vehicles) {
            double distance = std::numeric_limits<double>::infinity();
            for (const Aabb& box : participant.corridor) {
                distance = std::min(distance,
                    OptimizationInputBuilder::pointToAabbDistance(obstacle.disc_center, box));
            }
            const double influence = obstacle.disc_radius + participant.parameters.disc_radius;
            if (distance <= influence + kEpsilon) {
                obstacle.influencing_vehicle_ids.push_back(participant.id);
                if (distance < obstacle.min_distance_to_corridor) {
                    obstacle.min_distance_to_corridor = distance;
                    obstacle.influence_distance = influence;
                }
            }
        }
        if (!obstacle.influencing_vehicle_ids.empty()) selected.push_back(std::move(obstacle));
    }
    input.static_vehicles.swap(selected);
}

void updateGlobalDiagnostics(NlpInitialGuess& guess, const NlpOptimizationInput& input) {
    guess.diagnostics.minimum_participant_clearance = std::numeric_limits<double>::infinity();
    guess.diagnostics.minimum_static_clearance = std::numeric_limits<double>::infinity();
    for (const VehicleInitialGuess& vehicle : guess.vehicles) {
        const auto& d = vehicle.diagnostics;
        guess.diagnostics.max_kinematic_position_residual = std::max(
            guess.diagnostics.max_kinematic_position_residual, d.max_kinematic_position_residual);
        guess.diagnostics.max_kinematic_heading_residual = std::max(
            guess.diagnostics.max_kinematic_heading_residual, d.max_kinematic_heading_residual);
        guess.diagnostics.bound_violation_nodes += d.bound_violation_nodes;
        guess.diagnostics.aabb_violation_nodes += d.aabb_violation_nodes;
        guess.diagnostics.max_aabb_violation = std::max(guess.diagnostics.max_aabb_violation,
                                                        d.max_aabb_violation);
    }
    for (size_t first = 0; first < guess.vehicles.size(); ++first) {
        for (size_t second = first + 1; second < guess.vehicles.size(); ++second) {
            const double required = input.vehicles[first].parameters.disc_radius +
                                    input.vehicles[second].parameters.disc_radius;
            for (size_t node = 0; node < guess.node_count; ++node) {
                const double distance = poseDistance(guess.vehicles[first].nodes[node].disc_center,
                                                     guess.vehicles[second].nodes[node].disc_center);
                const double clearance = distance - required;
                guess.diagnostics.minimum_participant_clearance = std::min(
                    guess.diagnostics.minimum_participant_clearance, clearance);
                if (clearance < 0.0) {
                    ++guess.diagnostics.participant_collision_nodes;
                    guess.diagnostics.collisions.push_back({node, guess.time_grid[node],
                        guess.vehicles[first].id, guess.vehicles[second].id, false,
                        distance, required, clearance});
                }
            }
        }
    }
    for (size_t participant = 0; participant < guess.vehicles.size(); ++participant) {
        for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
            const double required = input.vehicles[participant].parameters.disc_radius + obstacle.disc_radius;
            for (size_t node = 0; node < guess.node_count; ++node) {
                const double distance = poseDistance(guess.vehicles[participant].nodes[node].disc_center,
                                                     obstacle.disc_center);
                const double clearance = distance - required;
                guess.diagnostics.minimum_static_clearance = std::min(
                    guess.diagnostics.minimum_static_clearance, clearance);
                if (clearance < 0.0) {
                    ++guess.diagnostics.static_collision_nodes;
                    guess.diagnostics.collisions.push_back({node, guess.time_grid[node],
                        guess.vehicles[participant].id, obstacle.id, true,
                        distance, required, clearance});
                }
            }
        }
    }
    if (!std::isfinite(guess.diagnostics.minimum_participant_clearance))
        guess.diagnostics.minimum_participant_clearance = 0.0;
    if (!std::isfinite(guess.diagnostics.minimum_static_clearance))
        guess.diagnostics.minimum_static_clearance = 0.0;
    const bool dynamics_close = guess.diagnostics.max_kinematic_position_residual <= 1e-3 &&
                                guess.diagnostics.max_kinematic_heading_residual <= 1e-3;
    guess.constraints_satisfied = guess.diagnostics.bound_violation_nodes == 0 &&
        guess.diagnostics.aabb_violation_nodes == 0 &&
        guess.diagnostics.participant_collision_nodes == 0 &&
        guess.diagnostics.static_collision_nodes == 0 && dynamics_close;
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

bool writeInitialGuessCsv(const NlpInitialGuess& guess, const std::filesystem::path& path,
                          const std::string& case_id, const std::string& generated_at,
                          std::string& error) {
    std::ofstream csv(path, std::ios::trunc);
    if (!csv) {
        error = "cannot open " + path.filename().string();
        return false;
    }
    csv << "case_id,generated_at,vehicle_id,node_index,time,path_s,path_type,"
           "rear_x,rear_y,heading,signed_speed,front_steer,acceleration,steer_rate,"
           "aabb_index,disc_center_x,disc_center_y,seed_offset_x,seed_offset_y,"
           "speed_source,steer_source,acceleration_source,steer_rate_source,"
           "kinematic_position_residual,kinematic_heading_residual,grid_node_count,"
           "grid_total_time,grid_average_dt,reference_length,estimated_travel_time,"
           "direction_changes,shifted_seed_nodes,max_seed_offset\n";
    csv << std::fixed << std::setprecision(9);
    for (const VehicleInitialGuess& vehicle : guess.vehicles) {
        for (const InitialGuessNode& node : vehicle.nodes) {
            csv << case_id << ',' << generated_at << ',' << vehicle.id << ',' << node.index << ','
                << node.time << ',' << node.path_s << ',' << static_cast<int>(node.path_type) << ','
                << node.rear_axle_pose.x << ',' << node.rear_axle_pose.y << ',' << node.rear_axle_pose.yaw << ','
                << node.signed_speed << ',' << node.front_steer << ',' << node.acceleration << ','
                << node.steer_rate << ',' << node.aabb_index << ',' << node.disc_center.x << ','
                << node.disc_center.y << ',' << node.seed_offset_x << ',' << node.seed_offset_y << ','
                << guessValueSourceName(node.speed_source) << ',' << guessValueSourceName(node.steer_source) << ','
                << guessValueSourceName(node.acceleration_source) << ',' << guessValueSourceName(node.steer_rate_source) << ','
                << node.kinematic_position_residual << ',' << node.kinematic_heading_residual << ','
                << guess.node_count << ',' << guess.total_time << ',' << guess.average_dt << ','
                << vehicle.diagnostics.reference_length << ',' << vehicle.diagnostics.estimated_travel_time << ','
                << vehicle.diagnostics.direction_changes << ',' << vehicle.diagnostics.shifted_seed_nodes << ','
                << vehicle.diagnostics.max_seed_offset << '\n';
        }
    }
    if (!csv) {
        error = "failed while writing " + path.filename().string();
        return false;
    }
    return true;
}

}  // namespace

const char* guessValueSourceName(GuessValueSource source) {
    switch (source) {
        case GuessValueSource::PATH_TIME_DIFFERENCE: return "path_time_difference";
        case GuessValueSource::PATH_GEOMETRY_ESTIMATE: return "path_geometry_estimate";
        case GuessValueSource::FINITE_DIFFERENCE: return "finite_difference";
        case GuessValueSource::TERMINAL_ZERO_ASSUMPTION: return "terminal_zero_assumption";
    }
    return "unknown";
}

InitialGuessGenerator::InitialGuessGenerator(const InitialGuessConfig& config,
                                             const CorridorBuilder& corridor_builder)
    : config_(config), corridor_builder_(corridor_builder) {}

InitialGuessPreparationResult InitialGuessGenerator::prepare(NlpOptimizationInput& input) const {
    InitialGuessPreparationResult prepared;
    NlpInitialGuess& original = prepared.original_guess;
    if (!input.valid) {
        original.reason = "optimization input is invalid: " + input.reason;
        prepared.actual_guess.reason = original.reason;
        return prepared;
    }
    if (input.vehicles.empty() || config_.node_count < 2 ||
        !(config_.time_margin >= 1.0) || !std::isfinite(config_.time_margin)) {
        original.reason = "invalid vehicles, initial_guess_nodes, or initial_guess_time_margin";
        prepared.actual_guess.reason = original.reason;
        return prepared;
    }
    std::vector<ReferenceMotion> references(input.vehicles.size());
    std::vector<std::vector<SegmentProfile>> profiles(input.vehicles.size());
    std::vector<double> profile_times(input.vehicles.size(), 0.0);
    std::vector<double> estimated_times(input.vehicles.size(), 0.0);
    std::vector<size_t> waiting_begins(input.vehicles.size(), config_.node_count);
    double maximum_time = 0.0;
    for (size_t i = 0; i < input.vehicles.size(); ++i) {
        const NlpVehicleInput& vehicle = input.vehicles[i];
        std::string error;
        if (!(vehicle.parameters.wheel_base > 0.0) || !(vehicle.parameters.disc_radius > 0.0) ||
            !buildReferenceMotion(vehicle, references[i], error) ||
            !estimateTravelTime(vehicle, references[i], estimated_times[i], error) ||
            !buildTimeProfile(vehicle, references[i], profiles[i], profile_times[i], error)) {
            original.reason = "V" + std::to_string(vehicle.id) + ": " + error;
            prepared.actual_guess.reason = original.reason;
            return prepared;
        }
        maximum_time = std::max(maximum_time, estimated_times[i]);
    }
    original.node_count = config_.node_count;
    original.total_time = maximum_time * config_.time_margin;
    original.average_dt = original.total_time / static_cast<double>(original.node_count - 1);
    if (!(original.average_dt > 0.0) || !std::isfinite(original.average_dt)) {
        original.reason = "computed initial time grid is invalid";
        prepared.actual_guess.reason = original.reason;
        return prepared;
    }
    for (size_t node = 0; node < original.node_count; ++node)
        original.time_grid.push_back(original.average_dt * static_cast<double>(node));
    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size(); ++vehicle_index) {
        const NlpVehicleInput& source = input.vehicles[vehicle_index];
        VehicleInitialGuess vehicle;
        vehicle.id = source.id;
        vehicle.nodes.resize(original.node_count);
        vehicle.diagnostics.reference_length = references[vehicle_index].total_length;
        vehicle.diagnostics.estimated_travel_time = estimated_times[vehicle_index];
        vehicle.diagnostics.direction_changes = references[vehicle_index].direction_changes;
        for (size_t node = 0; node < original.node_count; ++node) {
            InitialGuessNode& value = vehicle.nodes[node];
            value.index = node;
            value.time = original.time_grid[node];
            value.aabb_index = static_cast<int>(node);
            const bool waiting = value.time + kEpsilon >= profile_times[vehicle_index];
            if (waiting && waiting_begins[vehicle_index] == original.node_count) {
                waiting_begins[vehicle_index] = node;
            }
            const double profile_time = std::min(value.time, profile_times[vehicle_index]);
            const double progress = waiting || node + 1 == original.node_count ?
                references[vehicle_index].total_length :
                sampleProfile(profiles[vehicle_index], source.parameters, profile_time);
            sampleReferenceNode(source, references[vehicle_index], progress, value);
        }
        recomputeDerived(vehicle, source, original.average_dt, waiting_begins[vehicle_index]);
        vehicle.valid = true;
        vehicle.reason = "time-profile resampling completed";
        original.vehicles.push_back(std::move(vehicle));
    }
    original.generated = true;
    original.reason = "pre-offset time-profile initial guess generated";
    prepared.actual_guess = original;
    NlpInitialGuess& actual = prepared.actual_guess;
    bool success = true;
    std::ostringstream failures;
    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size(); ++vehicle_index) {
        NlpVehicleInput& source = input.vehicles[vehicle_index];
        VehicleInitialGuess& vehicle = actual.vehicles[vehicle_index];
        source.corridor.clear();
        source.corridor_seeds.clear();
        for (size_t node = 0; node < vehicle.nodes.size(); ++node) {
            InitialGuessNode& value = vehicle.nodes[node];
            const Pose2d requested = value.disc_center;
            Pose2d legal = requested;
            std::string reason;
            if (!corridor_builder_.pointLegal(requested)) {
                if (node == 0 || node + 1 == vehicle.nodes.size()) {
                    reason = "fixed endpoint seed is illegal";
                } else if (corridor_builder_.findLegalSeed(requested, legal, reason)) {
                    value.seed_offset_x = legal.x - requested.x;
                    value.seed_offset_y = legal.y - requested.y;
                    value.rear_axle_pose.x += value.seed_offset_x;
                    value.rear_axle_pose.y += value.seed_offset_y;
                    ++vehicle.diagnostics.shifted_seed_nodes;
                    vehicle.diagnostics.max_seed_offset = std::max(vehicle.diagnostics.max_seed_offset,
                        std::hypot(value.seed_offset_x, value.seed_offset_y));
                }
            }
            Aabb box;
            if (!reason.empty() || !corridor_builder_.generateAabb(legal, box, reason)) {
                if (failures.tellp() > 0) failures << "; ";
                failures << "V" << source.id << " node " << node << ": " << reason;
                success = false;
                vehicle.valid = false;
                vehicle.reason = reason;
                break;
            }
            source.corridor_seeds.push_back(legal);
            source.corridor.push_back(box);
        }
        if (source.corridor.size() == vehicle.nodes.size()) {
            recomputeDerived(vehicle, source, actual.average_dt, waiting_begins[vehicle_index]);
            vehicle.one_to_one_aabbs_valid = true;
            vehicle.valid = true;
            vehicle.reason = "one AABB generated for every NLP node";
            source.corridor_valid = true;
            source.corridor_reason = "one-to-one node AABBs generated";
            updateVehicleDiagnostics(vehicle, source, actual.average_dt, config_.containment_tolerance);
        } else {
            source.corridor_valid = false;
            source.corridor_reason = vehicle.reason;
        }
    }
    filterStaticVehicles(input);
    actual.generated = success;
    if (success) {
        updateGlobalDiagnostics(actual, input);
        actual.reason = actual.constraints_satisfied ?
            "generated and initial checks passed" :
            "generated with initial constraint residuals; retained for NLP initialization";
        input.valid = true;
        input.reason = "ok";
    } else {
        actual.reason = failures.str();
        input.valid = false;
        input.reason = actual.reason;
    }
    return prepared;
}

CaseSaveResult saveInitialGuessCase(const OptimizationInputBuildResult& input,
                                    const NlpInitialGuess& original_guess,
                                    const NlpInitialGuess& actual_guess,
                                    const std::string& results_directory,
                                    const std::string& case_prefix) {
    CaseSaveResult result;
    if (original_guess.vehicles.empty() || actual_guess.vehicles.empty()) {
        result.reason = "original or actual initial guess has no vehicle data";
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
        if (case_prefix.empty()) name << "case_" << std::setw(4) << std::setfill('0') << number;
        else name << case_prefix << '_' << number;
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
    if (!writeOptimizationInputDebugFile(input, (case_path / "optimization_input.yaml").string(), write_error)) {
        result.reason = write_error;
        return result;
    }
    std::ofstream yaml(case_path / "optimization_input.yaml", std::ios::app);
    yaml << std::boolalpha << std::fixed << std::setprecision(9)
         << "initial_guess_grid: {node_count: " << actual_guess.node_count
         << ", total_time: " << actual_guess.total_time
         << ", average_dt: " << actual_guess.average_dt << "}\n";
    if (!yaml) {
        result.reason = "failed while appending initial guess timing";
        return result;
    }
    const std::string generated_at = isoTimestamp();
    if (!writeInitialGuessCsv(original_guess, case_path / "original_initial_guess.csv",
                              result.case_id, generated_at, result.reason) ||
        !writeInitialGuessCsv(actual_guess, case_path / "initial_guess.csv",
                              result.case_id, generated_at, result.reason)) return result;
    result.success = true;
    result.case_directory = case_path.string();
    result.reason = "ok";
    return result;
}

}  // namespace forklift_nlp_recovery
