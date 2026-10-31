#include "forklift_nlp_recovery/candidate_selector.h"

#include <sandbox_msgs/RecoverySnapshot.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct Obb {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double half_length = 0.0;
    double half_width = 0.0;
};

Obb makeObb(const Pose2d& rear, const MapParam& map) {
    return {rear.x + map.rear_axle_to_center * std::cos(rear.yaw),
            rear.y + map.rear_axle_to_center * std::sin(rear.yaw), rear.yaw,
            map.vehicle_length * 0.5, map.vehicle_width * 0.5};
}

bool overlaps(const Obb& first, const Obb& second) {
    const double first_axes[2][2] = {{std::cos(first.yaw), std::sin(first.yaw)},
                                     {-std::sin(first.yaw), std::cos(first.yaw)}};
    const double second_axes[2][2] = {{std::cos(second.yaw), std::sin(second.yaw)},
                                      {-std::sin(second.yaw), std::cos(second.yaw)}};
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double axes[4][2] = {{first_axes[0][0], first_axes[0][1]},
                               {first_axes[1][0], first_axes[1][1]},
                               {second_axes[0][0], second_axes[0][1]},
                               {second_axes[1][0], second_axes[1][1]}};
    for (const auto& axis : axes) {
        const auto radius = [&](const Obb& box, const double local_axes[2][2]) {
            return box.half_length * std::abs(axis[0] * local_axes[0][0] + axis[1] * local_axes[0][1]) +
                   box.half_width * std::abs(axis[0] * local_axes[1][0] + axis[1] * local_axes[1][1]);
        };
        if (std::abs(dx * axis[0] + dy * axis[1]) >
            radius(first, first_axes) + radius(second, second_axes) + kEpsilon) return false;
    }
    return true;
}

bool overlapsAabb(const Obb& box, const Aabb& rectangle) {
    const Obb axis_aligned{0.5 * (rectangle.min_x + rectangle.max_x),
                           0.5 * (rectangle.min_y + rectangle.max_y), 0.0,
                           0.5 * rectangle.width(), 0.5 * rectangle.height()};
    return overlaps(box, axis_aligned);
}

PathPoint interpolatePath(const std::vector<PathPoint>& path, double s) {
    if (s <= path.front().s) return path.front();
    if (s >= path.back().s) return path.back();
    const auto upper = std::upper_bound(path.begin(), path.end(), s,
        [](double value, const PathPoint& point) { return value < point.s; });
    const PathPoint& second = *upper;
    const PathPoint& first = *(upper - 1);
    const double ratio = (s - first.s) / std::max(kEpsilon, second.s - first.s);
    return {{first.pose.x + ratio * (second.pose.x - first.pose.x),
             first.pose.y + ratio * (second.pose.y - first.pose.y),
             normalizeAngle(first.pose.yaw + ratio * normalizeAngle(second.pose.yaw - first.pose.yaw))},
            s, ratio < 0.5 ? first.type : second.type};
}

bool obbInsideField(const Obb& box, const MapParam& map) {
    const double c = std::cos(box.yaw);
    const double s = std::sin(box.yaw);
    for (const double longitudinal : {-box.half_length, box.half_length}) {
        for (const double lateral : {-box.half_width, box.half_width}) {
            const double x = box.x + c * longitudinal - s * lateral;
            const double y = box.y + s * longitudinal + c * lateral;
            if (x < -kEpsilon || x > map.field_width + kEpsilon ||
                y < -kEpsilon || y > map.field_height + kEpsilon) return false;
        }
    }
    return true;
}

bool poseSafe(const Pose2d& pose, int vehicle_id,
              const sandbox_msgs::RecoverySnapshot& snapshot,
              const MapParam& map, const std::vector<ShelfBlock>& shelves,
              int moved_vehicle_id = -1, const Pose2d* moved_pose = nullptr,
              std::string* failure_reason = nullptr) {
    const Obb body = makeObb(pose, map);
    if (!obbInsideField(body, map)) {
        if (failure_reason != nullptr) *failure_reason = "vehicle OBB crosses map boundary";
        return false;
    }
    for (size_t i = 0; i < shelves.size(); ++i) {
        const ShelfBlock& shelf = shelves[i];
        if (overlapsAabb(body, {shelf.x, shelf.x_max(), shelf.y, shelf.y_max()})) {
            if (failure_reason != nullptr) {
                *failure_reason = "vehicle OBB overlaps shelf[" + std::to_string(i) +
                    "] " + shelf.name;
            }
            return false;
        }
    }
    for (const auto& obstacle_state : snapshot.vehicles) {
        if (obstacle_state.id == vehicle_id || !obstacle_state.pose_valid) continue;
        Pose2d obstacle_pose{obstacle_state.pose.x, obstacle_state.pose.y,
                             obstacle_state.pose.theta};
        if (obstacle_state.id == moved_vehicle_id && moved_pose != nullptr) {
            obstacle_pose = *moved_pose;
        }
        if (overlaps(body, makeObb(obstacle_pose, map))) {
            if (failure_reason != nullptr) {
                *failure_reason = "vehicle OBB overlaps frozen V" +
                    std::to_string(obstacle_state.id);
            }
            return false;
        }
    }
    return true;
}

}  // namespace

CandidateSelector::CandidateSelector(
    const MapParam& map_param, const std::vector<ShelfBlock>& shelves,
    const CandidateSelectionConfig& config,
    const OptimizationInputBuilder& input_builder)
    : map_param_(map_param), shelves_(shelves), config_(config),
      input_builder_(input_builder) {}

const sandbox_msgs::RecoveryVehicleState* CandidateSelector::findVehicle(
    const sandbox_msgs::RecoverySnapshot& snapshot, int id) const {
    const auto found = std::find_if(snapshot.vehicles.begin(), snapshot.vehicles.end(),
        [id](const auto& vehicle) { return vehicle.id == id; });
    return found == snapshot.vehicles.end() ? nullptr : &*found;
}

bool CandidateSelector::intersectsRow2Bridge(
    const sandbox_msgs::RecoveryVehicleState& vehicle) const {
    const Aabb bridge{map_param_.row2_left_width,
        map_param_.row2_left_width + map_param_.row2_gap,
        map_param_.y4(), map_param_.y5()};
    return overlapsAabb(makeObb({vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta},
                                map_param_), bridge);
}

double CandidateSelector::continuousSafeDistance(
    const sandbox_msgs::RecoveryVehicleState& vehicle,
    const sandbox_msgs::RecoverySnapshot& snapshot,
    int moved_vehicle_id, const Pose2d* moved_pose) const {
    std::vector<Pose2d> poses;
    poses.reserve(vehicle.path.size());
    for (const auto& point : vehicle.path) poses.push_back({point.x, point.y, point.theta});
    const std::vector<PathPoint> path = CorridorBuilder::parameterizePath(poses, vehicle.path_types);
    if (path.size() < 2 || vehicle.path_s < path.front().s - kEpsilon ||
        vehicle.path_s > path.back().s + kEpsilon) return 0.0;
    const double step = std::max(1e-4, std::min(config_.sweep_step,
                                                map_param_.path_resolution));
    const auto safe = [&](double s) {
        return poseSafe(interpolatePath(path, s).pose, vehicle.id, snapshot,
                        map_param_, shelves_, moved_vehicle_id, moved_pose);
    };
    if (!safe(vehicle.path_s)) return 0.0;
    double last_safe = vehicle.path_s;
    for (double s = std::min(path.back().s, vehicle.path_s + step);
         s <= path.back().s + kEpsilon; s = std::min(path.back().s, s + step)) {
        if (!safe(s)) break;
        last_safe = s;
        if (s >= path.back().s - kEpsilon) break;
    }
    return std::max(0.0, last_safe - vehicle.path_s);
}

CandidateEvaluation CandidateSelector::evaluateBuiltCandidate(
    const sandbox_msgs::RecoverySnapshot& snapshot,
    const std::vector<int>& deadlock_cycle_ids,
    const std::vector<double>& before, int candidate_id,
    OptimizationInputBuildResult build) const {
    CandidateEvaluation evaluation;
    evaluation.vehicle_id = candidate_id;
    evaluation.build = std::move(build);
    if (!evaluation.build.complete || !evaluation.build.input.valid ||
        evaluation.build.input.vehicles.size() != 1) {
        evaluation.reason = evaluation.build.reason;
        return evaluation;
    }

    const Pose2d moved_pose = evaluation.build.input.vehicles.front().terminal.pose;
    evaluation.worst_gain = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < deadlock_cycle_ids.size(); ++i) {
        if (deadlock_cycle_ids[i] == candidate_id) continue;
        const auto* beneficiary = findVehicle(snapshot, deadlock_cycle_ids[i]);
        const double after = continuousSafeDistance(
            *beneficiary, snapshot, candidate_id, &moved_pose);
        const double gain = after - before[i];
        evaluation.beneficiary_ids.push_back(beneficiary->id);
        evaluation.gains.push_back(gain);
        evaluation.worst_gain = std::min(evaluation.worst_gain, gain);
        if (gain + kEpsilon >= config_.minimum_effective_gain) {
            ++evaluation.positive_gain_count;
        }
        evaluation.total_positive_gain += std::max(0.0, gain);
    }
    if (!std::isfinite(evaluation.worst_gain)) evaluation.worst_gain = 0.0;
    if (evaluation.worst_gain < -config_.maximum_negative_gain - kEpsilon) {
        evaluation.reason = "target layout causes excessive negative spatial gain";
    } else if (evaluation.positive_gain_count == 0) {
        evaluation.reason = "target layout has no effective positive spatial gain";
    } else {
        evaluation.eligible = true;
        evaluation.reason = "ok";
    }
    return evaluation;
}

CandidateSelectionResult CandidateSelector::evaluate(
    const sandbox_msgs::RecoverySnapshot& snapshot,
    const std::vector<int>& deadlock_cycle_ids,
    int off_path_start_vehicle_id) const {
    CandidateSelectionResult result;
    if (deadlock_cycle_ids.size() < 2) {
        result.reason = "confirmed deadlock cycle has fewer than two vehicles";
        return result;
    }
    const std::vector<int>& frozen_ids = snapshot.frozen_vehicle_ids.empty() ?
        snapshot.nlp_vehicle_ids : snapshot.frozen_vehicle_ids;
    for (const int id : frozen_ids) {
        const auto* frozen = findVehicle(snapshot, id);
        if (frozen == nullptr || !frozen->pose_valid || !frozen->pose_fresh ||
            std::abs(frozen->current_speed) > config_.stopped_speed_tolerance) {
            result.reason = "frozen vehicle snapshot is incomplete, stale, or still moving";
            return result;
        }
    }
    std::vector<double> before(deadlock_cycle_ids.size(), 0.0);
    for (size_t i = 0; i < deadlock_cycle_ids.size(); ++i) {
        const auto* vehicle = findVehicle(snapshot, deadlock_cycle_ids[i]);
        if (vehicle == nullptr || !vehicle->pose_valid || !vehicle->pose_fresh) {
            result.reason = "deadlock cycle snapshot is incomplete or stale";
            return result;
        }
        before[i] = continuousSafeDistance(*vehicle, snapshot, -1, nullptr);
    }
    result.complete_snapshot = true;
    for (const int candidate_id : deadlock_cycle_ids) {
        CandidateEvaluation evaluation;
        evaluation.vehicle_id = candidate_id;
        const auto* candidate = findVehicle(snapshot, candidate_id);
        if (candidate == nullptr || !candidate->pose_valid || !candidate->pose_fresh) {
            evaluation.reason = "invalid or stale vehicle pose";
        } else if (std::abs(candidate->current_speed) > config_.stopped_speed_tolerance) {
            evaluation.reason = "vehicle has not stopped";
        } else if (!candidate->departed_source_slot) {
            evaluation.reason = "vehicle body has not cleared the source slot";
        } else if (intersectsRow2Bridge(*candidate)) {
            evaluation.reason = "vehicle OBB intersects the row-2 bridge region";
        } else {
            evaluation = evaluateBuiltCandidate(
                snapshot, deadlock_cycle_ids, before, candidate_id,
                input_builder_.build(snapshot, {candidate_id},
                    candidate_id == off_path_start_vehicle_id ? "bridge_escape" : "auto",
                    candidate_id == off_path_start_vehicle_id));
        }
        result.ranked.push_back(std::move(evaluation));
    }
    std::stable_sort(result.ranked.begin(), result.ranked.end(),
        [](const CandidateEvaluation& first, const CandidateEvaluation& second) {
            if (first.eligible != second.eligible) return first.eligible > second.eligible;
            if (first.positive_gain_count != second.positive_gain_count)
                return first.positive_gain_count > second.positive_gain_count;
            if (std::abs(first.total_positive_gain - second.total_positive_gain) > kEpsilon)
                return first.total_positive_gain > second.total_positive_gain;
            return first.vehicle_id < second.vehicle_id;
        });
    result.reason = std::any_of(result.ranked.begin(), result.ranked.end(),
        [](const CandidateEvaluation& candidate) { return candidate.eligible; }) ?
        "ok" : "no candidate has bounded positive spatial gain";
    return result;
}

BridgeEscapeSelectionResult CandidateSelector::evaluateBridgeEscapes(
    const sandbox_msgs::RecoverySnapshot& snapshot,
    const std::vector<int>& deadlock_cycle_ids) const {
    BridgeEscapeSelectionResult result;
    if (deadlock_cycle_ids.size() < 2) {
        result.reason = "confirmed deadlock cycle has fewer than two vehicles";
        return result;
    }
    const std::vector<int>& frozen_ids = snapshot.frozen_vehicle_ids.empty() ?
        snapshot.nlp_vehicle_ids : snapshot.frozen_vehicle_ids;
    for (const int id : frozen_ids) {
        const auto* frozen = findVehicle(snapshot, id);
        if (frozen == nullptr || !frozen->pose_valid || !frozen->pose_fresh ||
            std::abs(frozen->current_speed) > config_.stopped_speed_tolerance) {
            result.reason = "frozen vehicle snapshot is incomplete, stale, or still moving";
            return result;
        }
    }

    std::vector<double> before(deadlock_cycle_ids.size(), 0.0);
    for (size_t i = 0; i < deadlock_cycle_ids.size(); ++i) {
        const auto* vehicle = findVehicle(snapshot, deadlock_cycle_ids[i]);
        if (vehicle == nullptr || !vehicle->pose_valid || !vehicle->pose_fresh) {
            result.reason = "deadlock cycle snapshot is incomplete or stale";
            return result;
        }
        before[i] = continuousSafeDistance(*vehicle, snapshot, -1, nullptr);
    }
    result.complete_snapshot = true;

    const Aabb bridge{map_param_.row2_left_width,
        map_param_.row2_left_width + map_param_.row2_gap,
        map_param_.y4(), map_param_.y5()};
    const double step = std::max(1e-4, std::min(config_.sweep_step,
                                                map_param_.path_resolution));
    const double rear_clearance = config_.bridge_escape_rear_clearance >= 0.0 ?
        config_.bridge_escape_rear_clearance :
        map_param_.vehicle_length * 0.5 - map_param_.rear_axle_to_center;
    const double front_extent = map_param_.vehicle_length * 0.5 +
                                map_param_.rear_axle_to_center;

    for (const int candidate_id : deadlock_cycle_ids) {
        BridgeEscapeEvaluation escape;
        escape.vehicle_id = candidate_id;
        const auto* candidate = findVehicle(snapshot, candidate_id);
        if (candidate == nullptr || !candidate->pose_valid || !candidate->pose_fresh) {
            escape.reason = "invalid or stale vehicle pose";
        } else if (std::abs(candidate->current_speed) > config_.stopped_speed_tolerance) {
            escape.reason = "vehicle has not stopped";
        } else if (!intersectsRow2Bridge(*candidate)) {
            escape.reason = "vehicle OBB does not intersect the row-2 bridge region";
        } else {
            escape.intersects_bridge = true;
            escape.path_gen = candidate->path_gen;
            escape.frozen_path_s = candidate->path_s;
            escape.frozen_pose = {candidate->pose.x, candidate->pose.y,
                                  candidate->pose.theta};
            std::vector<Pose2d> poses;
            poses.reserve(candidate->path.size());
            for (const auto& point : candidate->path) poses.push_back({point.x, point.y, point.theta});
            const std::vector<PathPoint> path =
                CorridorBuilder::parameterizePath(poses, candidate->path_types);
            const bool path_valid = path.size() >= 2 &&
                candidate->path_s >= path.front().s - kEpsilon &&
                candidate->path_s <= path.back().s + kEpsilon;
            if (!path_valid) {
                escape.reason = "frozen path or path progress is invalid";
            } else {
                const uint8_t path_type = interpolatePath(path, candidate->path_s).type;
                const int preferred_direction =
                    path_type == sandbox_msgs::RecoveryVehicleState::REVERSE ? -1 : 1;
                const int directions[2] = {preferred_direction, -preferred_direction};
                std::string first_failure;
                for (const int direction : directions) {
                    const double ux = direction * std::cos(escape.frozen_pose.yaw);
                    const double uy = direction * std::sin(escape.frozen_pose.yaw);
                    if (std::abs(uy) <= kEpsilon) {
                        if (first_failure.empty()) first_failure = "fixed heading cannot reach a bridge y exit";
                        continue;
                    }
                    const double longitudinal_min = -rear_clearance * std::sin(escape.frozen_pose.yaw);
                    const double longitudinal_max = front_extent * std::sin(escape.frozen_pose.yaw);
                    const double lateral_y = map_param_.vehicle_width * 0.5 *
                                             std::abs(std::cos(escape.frozen_pose.yaw));
                    const double min_relative_y =
                        std::min(longitudinal_min, longitudinal_max) - lateral_y;
                    const double max_relative_y =
                        std::max(longitudinal_min, longitudinal_max) + lateral_y;
                    const double target_y = uy > 0.0 ? bridge.max_y - min_relative_y :
                                                       bridge.min_y - max_relative_y;
                    const double required_distance =
                        (target_y - escape.frozen_pose.y) / uy;
                    if (required_distance < -kEpsilon) {
                        if (first_failure.empty()) first_failure = "selected straight direction does not reach the bridge exit";
                        continue;
                    }
                    const int sample_count = std::max(1, static_cast<int>(
                        std::ceil(std::max(0.0, required_distance) / step)) + 2);
                    std::vector<PathPoint> segment;
                    bool direction_failed = false;
                    for (int sample = 0; sample <= sample_count; ++sample) {
                        const double distance = sample == sample_count ?
                            std::max(0.0, required_distance) + step :
                            std::min(std::max(0.0, required_distance) + step, sample * step);
                        const Pose2d pose{escape.frozen_pose.x + ux * distance,
                                          escape.frozen_pose.y + uy * distance,
                                          escape.frozen_pose.yaw};
                        std::string collision_reason;
                        if (!poseSafe(pose, candidate_id, snapshot, map_param_, shelves_,
                                      -1, nullptr, &collision_reason)) {
                            direction_failed = true;
                            if (first_failure.empty()) {
                                escape.sweep_failure_valid = true;
                                escape.sweep_failure_distance = distance;
                                escape.sweep_failure_pose = pose;
                                escape.sweep_failure_at_start = distance <= kEpsilon;
                                escape.sweep_failure_reason = collision_reason;
                                std::ostringstream reason;
                                reason << "bridge escape straight sweep collision at d=" << distance
                                       << " pose=(" << pose.x << "," << pose.y << ","
                                       << pose.yaw << "): " << collision_reason
                                       << " at_S0=" << escape.sweep_failure_at_start;
                                first_failure = reason.str();
                            }
                            break;
                        }
                        segment.push_back({pose, distance,
                            direction > 0 ? sandbox_msgs::RecoveryVehicleState::FORWARD :
                                            sandbox_msgs::RecoveryVehicleState::REVERSE});
                        if (distance + kEpsilon >= required_distance &&
                            !overlapsAabb(makeObb(pose, map_param_), bridge)) {
                            escape.escape_valid = true;
                            escape.sweep_failure_valid = false;
                            escape.sweep_failure_reason.clear();
                            escape.escape_distance = distance;
                            escape.motion_direction = direction;
                            escape.escape_pose = pose;
                            escape.escape_segment = std::move(segment);
                            break;
                        }
                    }
                    if (escape.escape_valid) break;
                    if (!direction_failed && first_failure.empty()) {
                        first_failure = "no complete fixed-heading bridge-exit pose found";
                    }
                }
                if (!escape.escape_valid) escape.reason = first_failure;
            }
            if (escape.escape_valid) {
                sandbox_msgs::RecoverySnapshot virtual_snapshot = snapshot;
                auto virtual_vehicle = std::find_if(
                    virtual_snapshot.vehicles.begin(), virtual_snapshot.vehicles.end(),
                    [candidate_id](const auto& vehicle) { return vehicle.id == candidate_id; });
                virtual_vehicle->current_speed = 0.0;
                virtual_vehicle->pose.x = escape.escape_pose.x;
                virtual_vehicle->pose.y = escape.escape_pose.y;
                virtual_vehicle->pose.theta = escape.escape_pose.yaw;
                escape.candidate = evaluateBuiltCandidate(
                    virtual_snapshot, deadlock_cycle_ids, before, candidate_id,
                    input_builder_.build(virtual_snapshot, {candidate_id}, "bridge_escape", true));
                escape.eligible = escape.candidate.eligible;
                escape.reason = escape.candidate.reason;
            }
        }
        result.ranked.push_back(std::move(escape));
    }
    std::stable_sort(result.ranked.begin(), result.ranked.end(),
        [](const BridgeEscapeEvaluation& first, const BridgeEscapeEvaluation& second) {
            if (first.eligible != second.eligible) return first.eligible > second.eligible;
            if (first.candidate.positive_gain_count != second.candidate.positive_gain_count) {
                return first.candidate.positive_gain_count > second.candidate.positive_gain_count;
            }
            if (std::abs(first.candidate.total_positive_gain -
                         second.candidate.total_positive_gain) > kEpsilon) {
                return first.candidate.total_positive_gain >
                       second.candidate.total_positive_gain;
            }
            return first.vehicle_id < second.vehicle_id;
        });
    result.reason = std::any_of(result.ranked.begin(), result.ranked.end(),
        [](const BridgeEscapeEvaluation& escape) { return escape.eligible; }) ?
        "ok" : "no bridge vehicle has a safe beneficial escape";
    return result;
}

}  // namespace forklift_nlp_recovery
