#include "forklift_nlp_recovery/initial_guess_geometry_repair.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double pointDistance(const Pose2d& first, const Pose2d& second) {
    return std::hypot(first.x - second.x, first.y - second.y);
}

bool contains(const Aabb& box, const Pose2d& point, double tolerance) {
    return point.x >= box.min_x - tolerance && point.x <= box.max_x + tolerance &&
           point.y >= box.min_y - tolerance && point.y <= box.max_y + tolerance;
}

double aabbViolation(const Aabb& box, const Pose2d& point) {
    return std::max({box.min_x - point.x, point.x - box.max_x,
                     box.min_y - point.y, point.y - box.max_y, 0.0});
}

double corridorViolation(const std::vector<Aabb>& corridor, const Pose2d& point) {
    double result = kInfinity;
    for (const Aabb& box : corridor) result = std::min(result, aabbViolation(box, point));
    return result;
}

Pose2d projectInside(const Pose2d& point, const Aabb& box, double inset) {
    const double safe_x = std::min(inset, std::max(0.0, box.width() * 0.25));
    const double safe_y = std::min(inset, std::max(0.0, box.height() * 0.25));
    return {std::clamp(point.x, box.min_x + safe_x, box.max_x - safe_x),
            std::clamp(point.y, box.min_y + safe_y, box.max_y - safe_y), point.yaw};
}

Pose2d discCenter(const Pose2d& rear, const VehicleParameters& parameters) {
    return {rear.x + parameters.rear_axle_to_center * std::cos(rear.yaw),
            rear.y + parameters.rear_axle_to_center * std::sin(rear.yaw), rear.yaw};
}

bool isCusp(const VehicleInitialGuess& vehicle, size_t node) {
    return (node > 0 && vehicle.nodes[node - 1].path_type != vehicle.nodes[node].path_type) ||
           (node + 1 < vehicle.nodes.size() &&
            vehicle.nodes[node].path_type != vehicle.nodes[node + 1].path_type);
}

std::vector<size_t> containingBoxes(const std::vector<Aabb>& corridor,
                                    const Pose2d& point, double tolerance) {
    std::vector<size_t> result;
    for (size_t box = 0; box < corridor.size(); ++box) {
        if (contains(corridor[box], point, tolerance)) result.push_back(box);
    }
    return result;
}

bool orderedAssignment(const VehicleInitialGuess& vehicle,
                       const NlpVehicleInput& input,
                       const InitialGuessGeometryRepairConfig& config,
                       std::vector<int>& assignment, std::string& reason) {
    const size_t node_count = vehicle.nodes.size();
    const size_t box_count = input.corridor.size();
    if (node_count < 2 || box_count == 0) {
        reason = "empty node sequence or corridor";
        return false;
    }
    const double unreachable = kInfinity;
    std::vector<std::vector<double>> cost(node_count,
        std::vector<double>(box_count, unreachable));
    std::vector<std::vector<int>> predecessor(node_count,
        std::vector<int>(box_count, -1));

    const auto start_candidates = containingBoxes(
        input.corridor, vehicle.nodes.front().disc_center, config.containment_tolerance);
    if (!start_candidates.empty()) {
        for (size_t box : start_candidates) cost[0][box] = static_cast<double>(box) * 1e-9;
    } else {
        size_t first_covered = node_count;
        std::vector<size_t> candidates;
        for (size_t node = 1; node < node_count; ++node) {
            candidates = containingBoxes(input.corridor, vehicle.nodes[node].disc_center,
                                         config.containment_tolerance);
            if (!candidates.empty()) {
                first_covered = node;
                break;
            }
        }
        if (first_covered == node_count) {
            reason = "outside start has no covered forward sample";
            return false;
        }
        for (size_t box : candidates) cost[0][box] = static_cast<double>(box) * 1e-9;
    }

    for (size_t node = 1; node < node_count; ++node) {
        const Pose2d& center = vehicle.nodes[node].disc_center;
        const auto covered = containingBoxes(input.corridor, center,
                                              config.containment_tolerance);
        if (covered.empty() && (node + 1 == node_count || isCusp(vehicle, node))) {
            reason = node + 1 == node_count ? "terminal node is outside every AABB" :
                "an uncovered node coincides with a direction-change boundary";
            return false;
        }
        for (size_t box = 0; box < box_count; ++box) {
            if (!covered.empty() &&
                std::find(covered.begin(), covered.end(), box) == covered.end()) continue;
            const Pose2d projected = projectInside(center, input.corridor[box], config.aabb_inset);
            const double displacement = pointDistance(center, projected);
            if (covered.empty() && displacement > config.max_node_displacement + kEpsilon) continue;
            const double node_cost = covered.empty() ? displacement * displacement : 0.0;
            for (size_t previous = box > 0 ? box - 1 : 0; previous <= box; ++previous) {
                if (!std::isfinite(cost[node - 1][previous])) continue;
                const double candidate = cost[node - 1][previous] + node_cost +
                    (previous == box ? 0.0 : 1e-10);
                if (candidate < cost[node][box]) {
                    cost[node][box] = candidate;
                    predecessor[node][box] = static_cast<int>(previous);
                }
                if (previous == box) break;
            }
        }
    }

    const auto best = std::min_element(cost.back().begin(), cost.back().end());
    if (best == cost.back().end() || !std::isfinite(*best)) {
        reason = "no monotonic same-or-next AABB assignment exists within displacement limit";
        return false;
    }
    assignment.assign(node_count, -1);
    int box = static_cast<int>(std::distance(cost.back().begin(), best));
    for (size_t reverse = node_count; reverse-- > 0;) {
        assignment[reverse] = box;
        if (reverse > 0) box = predecessor[reverse][static_cast<size_t>(box)];
    }
    return true;
}

struct Vec2 { double x = 0.0; double y = 0.0; };

double dot(const Vec2& first, const Vec2& second) {
    return first.x * second.x + first.y * second.y;
}

struct Obb {
    Vec2 center;
    Vec2 longitudinal;
    Vec2 lateral;
    double half_length = 0.0;
    double half_width = 0.0;
};

Obb makeObb(const Pose2d& rear, double length, double width, double offset) {
    const double c = std::cos(rear.yaw);
    const double s = std::sin(rear.yaw);
    return {{rear.x + offset * c, rear.y + offset * s}, {c, s}, {-s, c},
            0.5 * length, 0.5 * width};
}

double projectionRadius(const Obb& box, const Vec2& axis) {
    return box.half_length * std::abs(dot(box.longitudinal, axis)) +
           box.half_width * std::abs(dot(box.lateral, axis));
}

bool obbOverlap(const Obb& first, const Obb& second, double tolerance) {
    const Vec2 delta{second.center.x - first.center.x, second.center.y - first.center.y};
    const Vec2 axes[] = {first.longitudinal, first.lateral,
                         second.longitudinal, second.lateral};
    for (const Vec2& axis : axes) {
        if (std::abs(dot(delta, axis)) + tolerance >=
            projectionRadius(first, axis) + projectionRadius(second, axis)) return false;
    }
    return true;
}

Pose2d interpolatePose(const Pose2d& first, const Pose2d& second, double ratio) {
    return {first.x + ratio * (second.x - first.x),
            first.y + ratio * (second.y - first.y),
            normalizeAngle(first.yaw + ratio * normalizeAngle(second.yaw - first.yaw))};
}

void recomputeDerivedState(VehicleInitialGuess& vehicle,
                           const NlpVehicleInput& input, double dt,
                           VehicleGeometryRepairDiagnostics& diagnostics) {
    const size_t count = vehicle.nodes.size();
    for (InitialGuessNode& node : vehicle.nodes) {
        node.disc_center = discCenter(node.rear_axle_pose, input.parameters);
    }
    for (size_t node = 0; node < count; ++node) {
        const double direction = vehicle.nodes[node].path_type == 1 ? -1.0 : 1.0;
        if (node == 0) {
            vehicle.nodes[node].signed_speed = direction *
                std::abs(input.initial_state.longitudinal_speed);
        } else if (node + 1 == count) {
            vehicle.nodes[node].signed_speed = 0.0;
        } else {
            vehicle.nodes[node].signed_speed = direction * pointDistance(
                vehicle.nodes[node - 1].rear_axle_pose,
                vehicle.nodes[node + 1].rear_axle_pose) / (2.0 * dt);
        }
    }
    for (size_t node = 0; node + 1 < count; ++node) {
        if (vehicle.nodes[node].path_type != vehicle.nodes[node + 1].path_type) {
            vehicle.nodes[node].signed_speed = 0.0;
            vehicle.nodes[node + 1].signed_speed = 0.0;
        }
    }
    for (size_t node = 0; node < count; ++node) {
        const size_t lo = node == 0 ? 0 : node - 1;
        const size_t hi = node + 1 < count ? node + 1 : node;
        const double distance = pointDistance(vehicle.nodes[lo].rear_axle_pose,
                                              vehicle.nodes[hi].rear_axle_pose);
        const bool cusp = vehicle.nodes[lo].path_type != vehicle.nodes[hi].path_type;
        const double direction = vehicle.nodes[node].path_type == 1 ? -1.0 : 1.0;
        double curvature = 0.0;
        if (distance > kEpsilon && !cusp) {
            curvature = normalizeAngle(vehicle.nodes[hi].rear_axle_pose.yaw -
                                       vehicle.nodes[lo].rear_axle_pose.yaw) /
                        (direction * distance);
        }
        diagnostics.max_curvature = std::max(diagnostics.max_curvature,
                                              std::abs(curvature));
        vehicle.nodes[node].front_steer =
            std::atan(input.parameters.wheel_base * curvature);
    }
    for (size_t node = 0; node < count; ++node) {
        const size_t lo = node == 0 ? 0 : node - 1;
        const size_t hi = node + 1 < count ? node + 1 : node;
        const double span = static_cast<double>(hi - lo) * dt;
        vehicle.nodes[node].acceleration = span > 0.0 ?
            (vehicle.nodes[hi].signed_speed - vehicle.nodes[lo].signed_speed) / span : 0.0;
        vehicle.nodes[node].steer_rate = span > 0.0 ?
            normalizeAngle(vehicle.nodes[hi].front_steer -
                           vehicle.nodes[lo].front_steer) / span : 0.0;
    }
    for (size_t node = 0; node + 1 < count; ++node) {
        InitialGuessNode& state = vehicle.nodes[node];
        const InitialGuessNode& next = vehicle.nodes[node + 1];
        const double predicted_x = state.rear_axle_pose.x +
            dt * state.signed_speed * std::cos(state.rear_axle_pose.yaw);
        const double predicted_y = state.rear_axle_pose.y +
            dt * state.signed_speed * std::sin(state.rear_axle_pose.yaw);
        const double predicted_yaw = state.rear_axle_pose.yaw +
            dt * state.signed_speed / input.parameters.wheel_base *
            std::tan(state.front_steer);
        state.kinematic_position_residual = std::hypot(
            next.rear_axle_pose.x - predicted_x, next.rear_axle_pose.y - predicted_y);
        state.kinematic_heading_residual = std::abs(normalizeAngle(
            next.rear_axle_pose.yaw - predicted_yaw));
        diagnostics.max_kinematic_position_residual = std::max(
            diagnostics.max_kinematic_position_residual,
            state.kinematic_position_residual);
        diagnostics.max_kinematic_heading_residual = std::max(
            diagnostics.max_kinematic_heading_residual,
            state.kinematic_heading_residual);
    }
}

}  // namespace

InitialGuessGeometryRepair::InitialGuessGeometryRepair(
    const InitialGuessGeometryRepairConfig& config,
    const CorridorBuilder& corridor_builder)
    : config_(config), corridor_builder_(corridor_builder) {}

InitialGuessGeometryRepairResult InitialGuessGeometryRepair::repair(
    const NlpOptimizationInput& input,
    const NlpInitialGuess& original_guess) const {
    InitialGuessGeometryRepairResult result;
    result.repaired_guess = original_guess;
    if (!config_.enabled) {
        result.success = original_guess.generated;
        result.reason = result.success ? "geometry repair disabled; original assignment valid" :
                                        "geometry repair disabled";
        return result;
    }
    if (!input.valid || original_guess.vehicles.size() != input.vehicles.size() ||
        original_guess.node_count < 2 || original_guess.average_dt <= 0.0) {
        result.reason = "incomplete input or original discrete guess";
        return result;
    }

    bool all_valid = true;
    result.vehicles.resize(input.vehicles.size());
    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size(); ++vehicle_index) {
        const NlpVehicleInput& source = input.vehicles[vehicle_index];
        const VehicleInitialGuess& original = original_guess.vehicles[vehicle_index];
        VehicleInitialGuess& repaired = result.repaired_guess.vehicles[vehicle_index];
        auto& diagnostics = result.vehicles[vehicle_index];
        diagnostics.id = source.id;
        if (source.id != original.id || original.nodes.size() != original_guess.node_count) {
            diagnostics.reason = "vehicle ordering or node count mismatch";
            all_valid = false;
            continue;
        }
        std::vector<bool> uncovered(original.nodes.size(), false);
        for (size_t node = 0; node < original.nodes.size(); ++node) {
            const double violation = corridorViolation(source.corridor,
                original.nodes[node].disc_center);
            diagnostics.max_aabb_violation_before = std::max(
                diagnostics.max_aabb_violation_before, violation);
            uncovered[node] = violation > config_.containment_tolerance &&
                !(node == 0 && original.start_aabb_exempt);
            if (uncovered[node]) ++diagnostics.original_uncovered_nodes;
        }
        if (original.aabb_assignment_valid && diagnostics.original_uncovered_nodes == 0) {
            diagnostics.success = true;
            diagnostics.reason = "already covered; original guess retained";
            continue;
        }

        diagnostics.attempted = true;
        result.attempted = true;
        std::vector<int> assignment;
        if (!orderedAssignment(original, source, config_, assignment,
                               diagnostics.reason)) {
            all_valid = false;
            continue;
        }

        std::vector<Pose2d> original_centers(original.nodes.size());
        std::vector<Pose2d> centers(original.nodes.size());
        std::vector<bool> movable(original.nodes.size(), false);
        for (size_t node = 0; node < original.nodes.size(); ++node) {
            original_centers[node] = original.nodes[node].disc_center;
            centers[node] = original_centers[node];
            if (!uncovered[node]) continue;
            centers[node] = projectInside(centers[node],
                source.corridor[static_cast<size_t>(assignment[node])], config_.aabb_inset);
            movable[node] = true;
            const size_t begin = node > config_.transition_nodes ?
                node - config_.transition_nodes : 1;
            const size_t end = std::min(node + config_.transition_nodes,
                                        original.nodes.size() - 2);
            for (size_t neighbor = begin; neighbor <= end; ++neighbor) {
                if (!isCusp(original, neighbor)) movable[neighbor] = true;
            }
        }
        for (size_t node = 1; node + 1 < centers.size(); ++node) {
            if (!movable[node] || uncovered[node]) continue;
            double weight_sum = 0.0;
            Vec2 offset;
            for (size_t gap = 1; gap + 1 < centers.size(); ++gap) {
                if (!uncovered[gap]) continue;
                const size_t distance = gap > node ? gap - node : node - gap;
                if (distance > config_.transition_nodes) continue;
                const double weight = static_cast<double>(config_.transition_nodes + 1 - distance) /
                                      static_cast<double>(config_.transition_nodes + 1);
                offset.x += weight * (centers[gap].x - original_centers[gap].x);
                offset.y += weight * (centers[gap].y - original_centers[gap].y);
                weight_sum += weight;
            }
            if (weight_sum > 0.0) {
                centers[node].x += offset.x / weight_sum;
                centers[node].y += offset.y / weight_sum;
            }
            centers[node] = projectInside(centers[node],
                source.corridor[static_cast<size_t>(assignment[node])], config_.aabb_inset);
        }
        const std::vector<Pose2d> repair_targets = centers;
        for (size_t iteration = 0; iteration < config_.smoothing_iterations; ++iteration) {
            std::vector<Pose2d> next = centers;
            for (size_t node = 1; node + 1 < centers.size(); ++node) {
                if (!movable[node]) continue;
                next[node].x = 0.60 * repair_targets[node].x +
                    0.20 * (centers[node - 1].x + centers[node + 1].x);
                next[node].y = 0.60 * repair_targets[node].y +
                    0.20 * (centers[node - 1].y + centers[node + 1].y);
                next[node] = projectInside(next[node],
                    source.corridor[static_cast<size_t>(assignment[node])], config_.aabb_inset);
            }
            centers.swap(next);
        }

        for (size_t node = 1; node + 1 < repaired.nodes.size(); ++node) {
            if (!movable[node]) continue;
            const double direction_yaw = std::atan2(centers[node + 1].y - centers[node - 1].y,
                                                     centers[node + 1].x - centers[node - 1].x) +
                (repaired.nodes[node].path_type == 1 ? M_PI : 0.0);
            const double yaw_delta = std::clamp(normalizeAngle(
                direction_yaw - original.nodes[node].rear_axle_pose.yaw),
                -config_.max_heading_adjustment, config_.max_heading_adjustment);
            const double yaw = normalizeAngle(
                original.nodes[node].rear_axle_pose.yaw + yaw_delta);
            repaired.nodes[node].rear_axle_pose = {
                centers[node].x - source.parameters.rear_axle_to_center * std::cos(yaw),
                centers[node].y - source.parameters.rear_axle_to_center * std::sin(yaw), yaw};
        }
        recomputeDerivedState(repaired, source, original_guess.average_dt, diagnostics);

        bool local_valid = true;
        const double body_radius = std::hypot(0.5 * source.parameters.length,
                                               0.5 * source.parameters.width);
        if (source.parameters.disc_radius + kEpsilon < body_radius) {
            ++diagnostics.static_obb_safety_violations;
            diagnostics.reason = "corridor disc does not circumscribe the vehicle OBB";
            local_valid = false;
        }
        for (size_t node = 0; node < repaired.nodes.size(); ++node) {
            const double displacement = pointDistance(original.nodes[node].disc_center,
                                                       repaired.nodes[node].disc_center);
            diagnostics.max_displacement = std::max(diagnostics.max_displacement,
                                                     displacement);
            if (displacement > 1e-8) {
                ++diagnostics.moved_nodes;
                diagnostics.nodes.push_back({node, assignment[node], uncovered[node],
                                             displacement});
            }
            diagnostics.max_aabb_violation_after = std::max(
                diagnostics.max_aabb_violation_after,
                corridorViolation(source.corridor, repaired.nodes[node].disc_center));
            if (node > 0 && !corridor_builder_.pointLegal(repaired.nodes[node].disc_center)) {
                ++diagnostics.disc_safety_violations;
                local_valid = false;
            }
        }
        if (diagnostics.max_displacement > config_.max_node_displacement + kEpsilon) {
            diagnostics.reason = "smoothed repair exceeds maximum node displacement";
            local_valid = false;
        }
        const double curvature_limit = std::tan(source.parameters.max_steer_angle) /
                                       source.parameters.wheel_base;
        if (diagnostics.max_curvature > curvature_limit + 1e-6) {
            diagnostics.reason = "repaired local curvature exceeds steering limit";
            local_valid = false;
        }
        for (size_t interval = 0; interval + 1 < repaired.nodes.size(); ++interval) {
            const Aabb& first = source.corridor[static_cast<size_t>(assignment[interval])];
            const Aabb& second = source.corridor[static_cast<size_t>(assignment[interval + 1])];
            for (size_t sample = 0; sample <= config_.interval_samples; ++sample) {
                const double ratio = static_cast<double>(sample) /
                                     static_cast<double>(config_.interval_samples);
                const Pose2d rear = interpolatePose(repaired.nodes[interval].rear_axle_pose,
                    repaired.nodes[interval + 1].rear_axle_pose, ratio);
                const Pose2d center = discCenter(rear, source.parameters);
                const double violation = std::min(aabbViolation(first, center),
                                                  aabbViolation(second, center));
                diagnostics.max_interval_aabb_violation = std::max(
                    diagnostics.max_interval_aabb_violation, violation);
                if (violation > config_.containment_tolerance) local_valid = false;
                if (!(interval == 0 && original.start_aabb_exempt) &&
                    !corridor_builder_.pointLegal(center)) {
                    ++diagnostics.disc_safety_violations;
                    local_valid = false;
                }
                const Obb moving = makeObb(rear, source.parameters.length,
                    source.parameters.width, source.parameters.rear_axle_to_center);
                for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
                    if (std::find(obstacle.influencing_vehicle_ids.begin(),
                                  obstacle.influencing_vehicle_ids.end(), source.id) ==
                        obstacle.influencing_vehicle_ids.end()) continue;
                    const Obb fixed = makeObb(obstacle.rear_axle_pose, obstacle.length,
                        obstacle.width, obstacle.rear_axle_to_center);
                    if (obbOverlap(moving, fixed, config_.containment_tolerance)) {
                        ++diagnostics.static_obb_safety_violations;
                        local_valid = false;
                    }
                }
            }
        }
        std::string assignment_reason;
        repaired.aabb_assignment_valid = assignInitialGuessAabbs(
            repaired, source, config_.containment_tolerance, assignment_reason);
        if (!repaired.aabb_assignment_valid) {
            diagnostics.reason = "strict post-repair AABB assignment failed: " + assignment_reason;
            local_valid = false;
        }
        repaired.valid = local_valid && repaired.aabb_assignment_valid;
        if (!repaired.valid && diagnostics.reason.empty()) {
            diagnostics.reason = "post-repair interval or safety check failed";
        }
        repaired.reason = repaired.valid ? "geometry repair passed" : diagnostics.reason;
        diagnostics.success = repaired.valid;
        if (diagnostics.success) diagnostics.reason = "local geometry repair passed";
        all_valid = all_valid && diagnostics.success;
    }

    for (size_t first = 0; first < result.repaired_guess.vehicles.size(); ++first) {
        for (size_t second = first + 1; second < result.repaired_guess.vehicles.size(); ++second) {
            const auto& first_vehicle = result.repaired_guess.vehicles[first];
            const auto& second_vehicle = result.repaired_guess.vehicles[second];
            const auto& first_parameters = input.vehicles[first].parameters;
            const auto& second_parameters = input.vehicles[second].parameters;
            for (size_t node = 0; node < result.repaired_guess.node_count; ++node) {
                const double center_distance = pointDistance(
                    first_vehicle.nodes[node].disc_center,
                    second_vehicle.nodes[node].disc_center);
                if (center_distance + config_.containment_tolerance <
                    first_parameters.disc_radius + second_parameters.disc_radius) {
                    ++result.vehicles[first].participant_disc_collisions;
                    ++result.vehicles[second].participant_disc_collisions;
                }
                if (obbOverlap(makeObb(first_vehicle.nodes[node].rear_axle_pose,
                                      first_parameters.length, first_parameters.width,
                                      first_parameters.rear_axle_to_center),
                               makeObb(second_vehicle.nodes[node].rear_axle_pose,
                                      second_parameters.length, second_parameters.width,
                                      second_parameters.rear_axle_to_center),
                               config_.containment_tolerance)) {
                    ++result.vehicles[first].participant_obb_collisions;
                    ++result.vehicles[second].participant_obb_collisions;
                }
            }
        }
    }
    result.success = all_valid;
    result.repaired_guess.generated = all_valid;
    result.repaired_guess.constraints_satisfied = result.attempted ? false :
        original_guess.constraints_satisfied;
    result.repaired_guess.reason = all_valid ?
        (result.attempted ? "local geometry repair succeeded; ready for IPOPT" :
                            "original AABB assignment valid; repair not required") :
        "local geometry repair failed; IPOPT initialization prohibited";
    result.reason = result.repaired_guess.reason;
    return result;
}

bool saveGeometryRepairReport(const InitialGuessGeometryRepairResult& result,
                              const std::string& case_directory,
                              std::string& error) {
    const std::filesystem::path directory(case_directory);
    if (!std::filesystem::is_directory(directory)) {
        error = "case directory does not exist";
        return false;
    }
    std::ofstream out(directory / "geometry_repair_report.yaml", std::ios::trunc);
    if (!out) {
        error = "cannot open geometry_repair_report.yaml";
        return false;
    }
    out << std::boolalpha << std::fixed << std::setprecision(9)
        << "attempted: " << result.attempted << '\n'
        << "success: " << result.success << '\n'
        << "reason: \"" << result.reason << "\"\n"
        << "vehicles:\n";
    for (const auto& vehicle : result.vehicles) {
        out << "  - id: " << vehicle.id << '\n'
            << "    attempted: " << vehicle.attempted << '\n'
            << "    success: " << vehicle.success << '\n'
            << "    reason: \"" << vehicle.reason << "\"\n"
            << "    original_uncovered_nodes: " << vehicle.original_uncovered_nodes << '\n'
            << "    moved_nodes: " << vehicle.moved_nodes << '\n'
            << "    max_displacement: " << vehicle.max_displacement << '\n'
            << "    max_aabb_violation_before: " << vehicle.max_aabb_violation_before << '\n'
            << "    max_aabb_violation_after: " << vehicle.max_aabb_violation_after << '\n'
            << "    max_interval_aabb_violation: " << vehicle.max_interval_aabb_violation << '\n'
            << "    max_curvature: " << vehicle.max_curvature << '\n'
            << "    max_kinematic_position_residual: "
            << vehicle.max_kinematic_position_residual << '\n'
            << "    max_kinematic_heading_residual: "
            << vehicle.max_kinematic_heading_residual << '\n'
            << "    disc_safety_violations: " << vehicle.disc_safety_violations << '\n'
            << "    static_obb_safety_violations: "
            << vehicle.static_obb_safety_violations << '\n'
            << "    participant_disc_collisions: "
            << vehicle.participant_disc_collisions << '\n'
            << "    participant_obb_collisions: "
            << vehicle.participant_obb_collisions << '\n'
            << "    repaired_nodes:\n";
        for (const auto& node : vehicle.nodes) {
            out << "      - {node: " << node.node_index
                << ", aabb: " << node.assigned_aabb
                << ", originally_uncovered: " << node.originally_uncovered
                << ", displacement: " << node.displacement << "}\n";
        }
    }
    if (!out) {
        error = "failed while writing geometry_repair_report.yaml";
        return false;
    }
    error = "ok";
    return true;
}

}  // namespace forklift_nlp_recovery
