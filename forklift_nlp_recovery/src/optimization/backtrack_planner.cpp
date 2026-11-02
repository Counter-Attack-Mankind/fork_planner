#include "forklift_nlp_recovery/backtrack_planner.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

bool contains(const Aabb& box, const Pose2d& point) {
    return point.x >= box.min_x - kEpsilon && point.x <= box.max_x + kEpsilon &&
           point.y >= box.min_y - kEpsilon && point.y <= box.max_y + kEpsilon;
}

Pose2d poseAtS(const std::vector<PathPoint>& path, double s) {
    if (s <= path.front().s) return path.front().pose;
    if (s >= path.back().s) return path.back().pose;
    const auto upper = std::upper_bound(path.begin(), path.end(), s,
        [](double value, const PathPoint& point) { return value < point.s; });
    const PathPoint& second = *upper;
    const PathPoint& first = *(upper - 1);
    const double ratio = (s - first.s) / std::max(kEpsilon, second.s - first.s);
    const double yaw_delta = std::atan2(std::sin(second.pose.yaw - first.pose.yaw),
                                        std::cos(second.pose.yaw - first.pose.yaw));
    return {first.pose.x + ratio * (second.pose.x - first.pose.x),
            first.pose.y + ratio * (second.pose.y - first.pose.y),
            first.pose.yaw + ratio * yaw_delta};
}

}  // namespace

BacktrackPlanner::BacktrackPlanner(const MapParam& map_param,
                                   const std::vector<Slot>& slots,
                                   const BacktrackConfig& config)
    : map_param_(map_param), slots_(slots), config_(config) {}

const sandbox_msgs::RecoveryVehicleState* BacktrackPlanner::findVehicle(
    const sandbox_msgs::RecoverySnapshot& snapshot, int id) const {
    const auto found = std::find_if(snapshot.vehicles.begin(), snapshot.vehicles.end(),
        [id](const auto& vehicle) { return vehicle.id == id; });
    return found == snapshot.vehicles.end() ? nullptr : &*found;
}

const OptimizationInputBuildResult* BacktrackPlanner::findPreserved(
    const std::vector<OptimizationInputBuildResult>& builds, int id) const {
    const auto found = std::find_if(builds.begin(), builds.end(), [id](const auto& build) {
        return build.complete && build.input.vehicles.size() == 1 &&
               build.input.vehicles.front().id == id;
    });
    return found == builds.end() ? nullptr : &*found;
}

bool BacktrackPlanner::startsAtSourceB(
    const sandbox_msgs::RecoveryVehicleState& vehicle) const {
    if (vehicle.mission_phase != sandbox_msgs::RecoveryVehicleState::MISSION_TO_A1 ||
        vehicle.source_slot_id < 0 ||
        vehicle.source_slot_id >= static_cast<int>(slots_.size()) ||
        vehicle.path.empty()) return false;
    const Slot& slot = slots_[static_cast<size_t>(vehicle.source_slot_id)];
    const auto& start = vehicle.path.front();
    const double center_x = start.x + map_param_.rear_axle_to_center * std::cos(start.theta);
    const double center_y = start.y + map_param_.rear_axle_to_center * std::sin(start.theta);
    const double yaw_error = std::atan2(std::sin(start.theta - slot.dock_theta),
                                        std::cos(start.theta - slot.dock_theta));
    const double tolerance = std::max(0.02, config_.path_validation_step);
    return std::hypot(center_x - slot.dock_x(), center_y - slot.dock_y()) <= tolerance &&
           std::abs(yaw_error) <= 1e-3;
}

bool BacktrackPlanner::oldCorridorContains(const NlpVehicleInput& input,
                                           const Pose2d& rear) const {
    if (!input.stc_valid || input.corridor.empty()) return false;
    const Pose2d center{rear.x + map_param_.rear_axle_to_center * std::cos(rear.yaw),
                        rear.y + map_param_.rear_axle_to_center * std::sin(rear.yaw),
                        rear.yaw};
    return contains(input.safe_stc, center) && contains(input.corridor.front(), center);
}

BacktrackPlan BacktrackPlanner::plan(
    const sandbox_msgs::RecoverySnapshot& snapshot,
    const std::vector<int>& deadlock_cycle_ids,
    const std::vector<OptimizationInputBuildResult>& preserved_builds) const {
    BacktrackPlan result;
    result.speed = config_.speed;
    if (!config_.enabled) {
        result.reason = "NLP backtrack is disabled";
        return result;
    }
    const double step = std::max(0.005, config_.path_validation_step);
    for (const int id : deadlock_cycle_ids) {
        const auto* vehicle = findVehicle(snapshot, id);
        if (vehicle == nullptr || !vehicle->pose_valid || !vehicle->pose_fresh ||
            std::abs(vehicle->current_speed) > 1e-6 || vehicle->path.size() < 2 ||
            vehicle->path_types.size() != vehicle->path.size() ||
            vehicle->path_s <= kEpsilon) continue;

        BacktrackCandidate candidate;
        candidate.vehicle_id = id;
        candidate.path_gen = vehicle->path_gen;
        candidate.current_s = vehicle->path_s;
        const bool source_b = vehicle->path_s < config_.distance - kEpsilon &&
                              startsAtSourceB(*vehicle);
        if (source_b) {
            candidate.target_s = 0.0;
            candidate.hold_at_source_b = true;
            candidate.reason = "TO_A1 path start matches source B slot";
            result.candidates.push_back(std::move(candidate));
            continue;
        }

        const OptimizationInputBuildResult* preserved = findPreserved(preserved_builds, id);
        if (preserved == nullptr || !preserved->input.valid) continue;
        candidate.preserved_build = *preserved;
        const NlpVehicleInput& old_input = candidate.preserved_build.input.vehicles.front();
        const double requested = std::max(0.0, vehicle->path_s - config_.distance);
        double last_valid = vehicle->path_s;
        for (double sample = std::max(requested, vehicle->path_s - step);
             sample >= requested - kEpsilon;
             sample = std::max(requested, sample - step)) {
            if (!oldCorridorContains(old_input, poseAtS(old_input.original_path, sample))) break;
            last_valid = sample;
            if (sample <= requested + kEpsilon) break;
        }
        if (vehicle->path_s - last_valid <= kEpsilon) continue;
        candidate.target_s = last_valid;
        candidate.reason = last_valid <= requested + kEpsilon ?
            "requested distance is compatible with preserved STC" :
            "target clipped at preserved STC boundary";
        result.candidates.push_back(std::move(candidate));
    }
    result.executable = !result.candidates.empty();
    if (result.executable) {
        result.reason = "ok";
    } else {
        result.reason = "no deadlock-cycle vehicle has a safe positive backtrack interval";
    }
    return result;
}

}  // namespace forklift_nlp_recovery
