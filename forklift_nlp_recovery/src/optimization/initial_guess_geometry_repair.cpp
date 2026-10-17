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
    if (!input.valid || input.vehicles.empty() ||
        original_guess.vehicles.size() != input.vehicles.size() ||
        original_guess.node_count < 2 ||
        original_guess.time_grid.size() != original_guess.node_count ||
        original_guess.average_dt <= 0.0) {
        result.reason = "incomplete input or original discrete guess";
        result.repaired_guess.generated = false;
        return result;
    }

    bool all_assignments_valid = true;
    result.vehicles.resize(input.vehicles.size());
    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size(); ++vehicle_index) {
        const NlpVehicleInput& source = input.vehicles[vehicle_index];
        const VehicleInitialGuess& original = original_guess.vehicles[vehicle_index];
        VehicleInitialGuess& repaired = result.repaired_guess.vehicles[vehicle_index];
        auto& diagnostics = result.vehicles[vehicle_index];
        diagnostics.id = source.id;
        if (source.id != original.id || original.nodes.size() != original_guess.node_count) {
            diagnostics.reason = "vehicle ordering or node count mismatch";
            repaired.valid = false;
            repaired.aabb_assignment_valid = false;
            all_assignments_valid = false;
            continue;
        }

        std::string assignment_reason;
        repaired.aabb_assignment_valid = assignInitialGuessAabbs(
            repaired, source, config_.containment_tolerance, assignment_reason);
        diagnostics.aabb_assignment_valid = repaired.aabb_assignment_valid;
        if (!repaired.aabb_assignment_valid) {
            diagnostics.reason = "logical AABB assignment failed: " + assignment_reason;
            repaired.valid = false;
            repaired.reason = diagnostics.reason;
            all_assignments_valid = false;
            continue;
        }

        std::vector<int> assignment(repaired.nodes.size(), -1);
        for (size_t node = 0; node < repaired.nodes.size(); ++node) {
            assignment[node] = repaired.nodes[node].aabb_index;
        }
        const std::vector<int> original_assignment = assignment;
        std::vector<bool> uncovered(original.nodes.size(), false);
        for (size_t node = 0; node < original.nodes.size(); ++node) {
            const double corridor_violation = corridorViolation(source.corridor,
                original.nodes[node].disc_center);
            if (!(node == 0 && original.start_aabb_exempt)) {
                diagnostics.max_aabb_violation_before = std::max(
                    diagnostics.max_aabb_violation_before,
                    aabbViolation(source.corridor[static_cast<size_t>(assignment[node])],
                                  original.nodes[node].disc_center));
            }
            uncovered[node] = corridor_violation > config_.containment_tolerance &&
                !(node == 0 && original.start_aabb_exempt);
            if (uncovered[node]) ++diagnostics.original_uncovered_nodes;
        }
        VehicleInitialGuess evaluated_original = original;
        VehicleGeometryRepairDiagnostics original_derived;
        recomputeDerivedState(evaluated_original, source, original_guess.average_dt,
                              original_derived);
        diagnostics.max_curvature_before = original_derived.max_curvature;
        diagnostics.max_kinematic_position_residual_before =
            original_derived.max_kinematic_position_residual;
        diagnostics.max_kinematic_heading_residual_before =
            original_derived.max_kinematic_heading_residual;

        std::vector<Pose2d> original_centers(original.nodes.size());
        std::vector<Pose2d> centers(original.nodes.size());
        std::vector<bool> movable(original.nodes.size(), false);
        for (size_t node = 0; node < original.nodes.size(); ++node) {
            original_centers[node] = original.nodes[node].disc_center;
            centers[node] = original_centers[node];
            if (!config_.enabled || !uncovered[node] || node == 0 ||
                node + 1 == original.nodes.size() || isCusp(original, node)) continue;
            diagnostics.repair_attempted = true;
            const Pose2d projected = projectInside(centers[node],
                source.corridor[static_cast<size_t>(assignment[node])], config_.aabb_inset);
            if (pointDistance(original_centers[node], projected) >
                config_.max_node_displacement + kEpsilon) continue;
            centers[node] = projected;
            movable[node] = true;
            const size_t begin = node > config_.transition_nodes ?
                node - config_.transition_nodes : 1;
            const size_t end = std::min(node + config_.transition_nodes,
                                        original.nodes.size() - 2);
            for (size_t neighbor = begin; neighbor <= end; ++neighbor) {
                if (!isCusp(original, neighbor)) movable[neighbor] = true;
            }
        }
        if (config_.enabled && diagnostics.original_uncovered_nodes > 0) {
            diagnostics.repair_attempted = true;
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

        for (size_t node = 1; node + 1 < centers.size(); ++node) {
            if (!movable[node]) continue;
            if (pointDistance(original_centers[node], centers[node]) >
                config_.max_node_displacement + kEpsilon) {
                centers[node] = original_centers[node];
                movable[node] = false;
            }
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

        repaired.aabb_assignment_valid = assignInitialGuessAabbs(
            repaired, source, config_.containment_tolerance, assignment_reason);
        diagnostics.aabb_assignment_valid = repaired.aabb_assignment_valid;
        if (!repaired.aabb_assignment_valid) {
            diagnostics.reason = "post-repair logical AABB assignment failed: " + assignment_reason;
            repaired.valid = false;
            repaired.reason = diagnostics.reason;
            all_assignments_valid = false;
            continue;
        }
        for (size_t node = 0; node < repaired.nodes.size(); ++node) {
            assignment[node] = repaired.nodes[node].aabb_index;
        }

        const double body_radius = std::hypot(0.5 * source.parameters.length,
                                               0.5 * source.parameters.width);
        if (source.parameters.disc_radius + kEpsilon < body_radius) {
            ++diagnostics.static_obb_safety_violations_before;
            ++diagnostics.static_obb_safety_violations;
        }
        for (size_t node = 0; node < evaluated_original.nodes.size(); ++node) {
            if (!(node == 0 && original.start_aabb_exempt) &&
                !corridor_builder_.pointLegal(evaluated_original.nodes[node].disc_center)) {
                ++diagnostics.disc_safety_violations_before;
            }
        }
        for (size_t interval = 0; interval + 1 < evaluated_original.nodes.size(); ++interval) {
            const Aabb& first = source.corridor[
                static_cast<size_t>(original_assignment[interval])];
            const Aabb& second = source.corridor[
                static_cast<size_t>(original_assignment[interval + 1])];
            for (size_t sample = 0; sample <= config_.interval_samples; ++sample) {
                const double ratio = static_cast<double>(sample) /
                                     static_cast<double>(config_.interval_samples);
                const Pose2d rear = interpolatePose(
                    evaluated_original.nodes[interval].rear_axle_pose,
                    evaluated_original.nodes[interval + 1].rear_axle_pose, ratio);
                const Pose2d center = discCenter(rear, source.parameters);
                diagnostics.max_interval_aabb_violation_before = std::max(
                    diagnostics.max_interval_aabb_violation_before,
                    std::min(aabbViolation(first, center), aabbViolation(second, center)));
                if (!(interval == 0 && original.start_aabb_exempt) &&
                    !corridor_builder_.pointLegal(center)) {
                    ++diagnostics.disc_safety_violations_before;
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
                        ++diagnostics.static_obb_safety_violations_before;
                    }
                }
            }
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
            if (!(node == 0 && repaired.start_aabb_exempt)) {
                diagnostics.max_aabb_violation_after = std::max(
                    diagnostics.max_aabb_violation_after,
                    aabbViolation(source.corridor[static_cast<size_t>(assignment[node])],
                                  repaired.nodes[node].disc_center));
            }
            if (!(node == 0 && repaired.start_aabb_exempt) &&
                !corridor_builder_.pointLegal(repaired.nodes[node].disc_center)) {
                ++diagnostics.disc_safety_violations;
            }
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
                if (!(interval == 0 && original.start_aabb_exempt) &&
                    !corridor_builder_.pointLegal(center)) {
                    ++diagnostics.disc_safety_violations;
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
                    }
                }
            }
        }
        repaired.valid = repaired.aabb_assignment_valid;
        repaired.reason = repaired.valid ? "complete logical AABB assignment" : diagnostics.reason;
        diagnostics.repaired = diagnostics.moved_nodes > 0;
        result.repaired = result.repaired || diagnostics.repaired;
        if (diagnostics.reason.empty()) {
            if (!config_.enabled) diagnostics.reason = "geometry repair disabled; diagnostics only";
            else if (diagnostics.original_uncovered_nodes == 0) {
                diagnostics.reason = "no AABB gap; original guess retained";
            } else if (diagnostics.repaired) {
                diagnostics.reason = diagnostics.max_aabb_violation_after <=
                    config_.containment_tolerance ? "local repair removed point AABB gaps" :
                    "local repair applied; remaining violations are diagnostic only";
            } else {
                diagnostics.reason = "repair produced no admissible displacement; original guess retained";
            }
        }
    }

    for (size_t first = 0;
         all_assignments_valid && first < result.repaired_guess.vehicles.size(); ++first) {
        for (size_t second = first + 1;
             second < result.repaired_guess.vehicles.size(); ++second) {
            const auto& first_vehicle = result.repaired_guess.vehicles[first];
            const auto& second_vehicle = result.repaired_guess.vehicles[second];
            const auto& original_first_vehicle = original_guess.vehicles[first];
            const auto& original_second_vehicle = original_guess.vehicles[second];
            const auto& first_parameters = input.vehicles[first].parameters;
            const auto& second_parameters = input.vehicles[second].parameters;
            for (size_t node = 0; node < result.repaired_guess.node_count; ++node) {
                const double original_center_distance = pointDistance(
                    original_first_vehicle.nodes[node].disc_center,
                    original_second_vehicle.nodes[node].disc_center);
                if (original_center_distance + config_.containment_tolerance <
                    first_parameters.disc_radius + second_parameters.disc_radius) {
                    ++result.vehicles[first].participant_disc_collisions_before;
                    ++result.vehicles[second].participant_disc_collisions_before;
                }
                if (obbOverlap(makeObb(original_first_vehicle.nodes[node].rear_axle_pose,
                                      first_parameters.length, first_parameters.width,
                                      first_parameters.rear_axle_to_center),
                               makeObb(original_second_vehicle.nodes[node].rear_axle_pose,
                                      second_parameters.length, second_parameters.width,
                                      second_parameters.rear_axle_to_center),
                               config_.containment_tolerance)) {
                    ++result.vehicles[first].participant_obb_collisions_before;
                    ++result.vehicles[second].participant_obb_collisions_before;
                }
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
    result.aabb_assignment_valid = all_assignments_valid;
    result.generated = all_assignments_valid;
    result.repaired_guess.generated = result.generated;
    result.repaired_guess.constraints_satisfied = result.repaired ? false :
        original_guess.constraints_satisfied;
    result.repaired_guess.reason = result.generated ?
        "initial guess generated with complete logical AABB assignments" :
        "initial guess lacks a complete logical AABB assignment";
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
        << "generated: " << result.generated << '\n'
        << "repaired: " << result.repaired << '\n'
        << "aabb_assignment_valid: " << result.aabb_assignment_valid << '\n'
        << "reason: \"" << result.reason << "\"\n"
        << "vehicles:\n";
    for (const auto& vehicle : result.vehicles) {
        out << "  - id: " << vehicle.id << '\n'
            << "    repair_attempted: " << vehicle.repair_attempted << '\n'
            << "    repaired: " << vehicle.repaired << '\n'
            << "    aabb_assignment_valid: " << vehicle.aabb_assignment_valid << '\n'
            << "    reason: \"" << vehicle.reason << "\"\n"
            << "    original_uncovered_nodes: " << vehicle.original_uncovered_nodes << '\n'
            << "    moved_nodes: " << vehicle.moved_nodes << '\n'
            << "    max_displacement: " << vehicle.max_displacement << '\n'
            << "    max_aabb_violation_before: " << vehicle.max_aabb_violation_before << '\n'
            << "    max_aabb_violation_after: " << vehicle.max_aabb_violation_after << '\n'
            << "    max_interval_aabb_violation_before: "
            << vehicle.max_interval_aabb_violation_before << '\n'
            << "    max_interval_aabb_violation_after: " << vehicle.max_interval_aabb_violation << '\n'
            << "    max_curvature_before: " << vehicle.max_curvature_before << '\n'
            << "    max_curvature_after: " << vehicle.max_curvature << '\n'
            << "    max_kinematic_position_residual_before: "
            << vehicle.max_kinematic_position_residual_before << '\n'
            << "    max_kinematic_heading_residual_before: "
            << vehicle.max_kinematic_heading_residual_before << '\n'
            << "    max_kinematic_position_residual_after: "
            << vehicle.max_kinematic_position_residual << '\n'
            << "    max_kinematic_heading_residual_after: "
            << vehicle.max_kinematic_heading_residual << '\n'
            << "    disc_safety_violations_before: "
            << vehicle.disc_safety_violations_before << '\n'
            << "    disc_safety_violations_after: " << vehicle.disc_safety_violations << '\n'
            << "    static_obb_safety_violations_before: "
            << vehicle.static_obb_safety_violations_before << '\n'
            << "    static_obb_safety_violations_after: "
            << vehicle.static_obb_safety_violations << '\n'
            << "    participant_disc_collisions_before: "
            << vehicle.participant_disc_collisions_before << '\n'
            << "    participant_disc_collisions_after: "
            << vehicle.participant_disc_collisions << '\n'
            << "    participant_obb_collisions_before: "
            << vehicle.participant_obb_collisions_before << '\n'
            << "    participant_obb_collisions_after: "
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
