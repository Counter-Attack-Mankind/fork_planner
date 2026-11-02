#include "forklift_planner/multi_vehicle/recovery/nlp_backtrack_executor.h"

#include <algorithm>
#include <cmath>
#include <set>

#include <ros/ros.h>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

NlpBacktrackExecutor::NlpBacktrackExecutor(
    const MapParam& map_param, const std::vector<ShelfBlock>& shelves,
    const NlpBacktrackExecutorConfig& config)
    : map_param_(map_param), config_(config) {
    (void)shelves;
}

VehicleAgent* NlpBacktrackExecutor::findVehicle(std::vector<VehicleAgent>& vehicles,
                                                int id) const {
    const auto found = std::find_if(vehicles.begin(), vehicles.end(),
        [id](const VehicleAgent& vehicle) { return vehicle.id == id; });
    return found == vehicles.end() ? nullptr : &*found;
}

const VehicleAgent* NlpBacktrackExecutor::findVehicle(
    const std::vector<VehicleAgent>& vehicles, int id) const {
    const auto found = std::find_if(vehicles.begin(), vehicles.end(),
        [id](const VehicleAgent& vehicle) { return vehicle.id == id; });
    return found == vehicles.end() ? nullptr : &*found;
}

bool NlpBacktrackExecutor::poseSafe(
    const VehicleAgent& moving, const RoughWp& pose,
    const std::vector<VehicleAgent>& vehicles, int* blocker_id) const {
    if (blocker_id != nullptr) *blocker_id = -1;
    const OBB body = makeBody(pose, map_param_, 0.0);
    for (const VehicleAgent& other : vehicles) {
        if (other.id == moving.id) continue;
        RoughWp obstacle;
        bool valid = false;
        if (other.real_pose_valid) {
            obstacle = {other.real_x, other.real_y, other.real_yaw, WpType::FORWARD};
            valid = true;
        } else if (!other.track.empty()) {
            obstacle = other.track.poseAtS(
                std::clamp(other.path_s, 0.0, other.track.length()));
            valid = true;
        } else if (other.stationary_pose_valid) {
            obstacle = {other.stationary_x, other.stationary_y,
                        other.stationary_yaw, WpType::FORWARD};
            valid = true;
        }
        if (valid && overlaps(body, makeBody(obstacle, map_param_, 0.0))) {
            if (blocker_id != nullptr) *blocker_id = other.id;
            return false;
        }
    }
    return true;
}

bool NlpBacktrackExecutor::sweepClear(
    const VehicleAgent& moving, double target_s,
    const std::vector<VehicleAgent>& vehicles, CollisionInfo& collision) const {
    const double step = std::max(0.005, config_.path_validation_step);
    const double distance = moving.path_s - target_s;
    const int count = std::max(1, static_cast<int>(std::ceil(distance / step)));
    for (int i = 0; i <= count; ++i) {
        const double s = moving.path_s - distance * static_cast<double>(i) / count;
        const RoughWp pose = moving.track.poseAtS(s);
        int blocker_id = -1;
        if (!poseSafe(moving, pose, vehicles, &blocker_id)) {
            collision.blocker_id = blocker_id;
            collision.path_s = s;
            collision.pose = pose;
            return false;
        }
    }
    return true;
}

void NlpBacktrackExecutor::fail(const std::string& reason) {
    active_ = false;
    status_ = SAFETY_FAILED;
    reason_ = reason;
}

void NlpBacktrackExecutor::finishBatch() {
    active_ = false;
    status_ = moved_count_ > 0 ? SUCCEEDED : NO_EXECUTABLE;
    reason_ = moved_count_ > 0 ? "backtrack batch changed the frozen layout" :
                                 "all backtrack vehicles were skipped without movement";
    ROS_WARN("[NLP_BACKTRACK_FINISH] completed=%zu skipped=%zu moved=%zu total=%zu",
             completed_count_, skipped_count_, moved_count_, requests_.size());
    vehicle_id_ = -1;
    path_gen_ = -1;
    target_s_ = 0.0;
}

void NlpBacktrackExecutor::finishCurrentBlocked(
    VehicleAgent& vehicle, const CollisionInfo& collision,
    std::vector<VehicleAgent>& vehicles) {
    const Request& request = requests_[request_index_];
    vehicle.current_speed = 0.0;
    const double tolerance = std::max(0.005, config_.path_validation_step);
    const bool moved = request.start_s - vehicle.path_s > tolerance;
    vehicle.reason = moved ? "nlp_backtrack_collision_hold" : "nlp_backtrack_skipped";
    if (moved) ++moved_count_;
    ++skipped_count_;
    ROS_WARN("[NLP_BACKTRACK_BLOCKED] V%d blocker=V%d collision_s=%.6f "
             "pose=(%.6f,%.6f,%.6f) moved=%d",
             vehicle.id, collision.blocker_id, collision.path_s,
             collision.pose.x, collision.pose.y, collision.pose.theta, moved);
    ++request_index_;
    startCurrent(vehicles);
}

bool NlpBacktrackExecutor::startCurrent(std::vector<VehicleAgent>& vehicles) {
    if (request_index_ >= requests_.size()) {
        finishBatch();
        return true;
    }
    const Request& request = requests_[request_index_];
    VehicleAgent* vehicle = findVehicle(vehicles, request.vehicle_id);
    vehicle_id_ = request.vehicle_id;
    path_gen_ = request.path_gen;
    target_s_ = request.target_s;
    if (vehicle == nullptr || vehicle->path_gen != request.path_gen ||
        vehicle->track.empty() || request.target_s < -1e-9 ||
        request.target_s >= vehicle->path_s - 1e-9 ||
        vehicle->path_s - request.target_s > config_.max_distance + 1e-9) {
        fail("vehicle, path identity, or target interval changed before execution");
        return false;
    }
    requests_[request_index_].start_s = vehicle->path_s;
    ROS_INFO("[NLP_BACKTRACK_CHECK] V%d batch=%zu/%zu current_s=%.6f "
             "target_s=%.6f expected=%.6f",
             vehicle->id, request_index_ + 1, requests_.size(), vehicle->path_s,
             request.target_s, vehicle->path_s - request.target_s);
    CollisionInfo collision;
    if (!sweepClear(*vehicle, request.target_s, vehicles, collision)) {
        finishCurrentBlocked(*vehicle, collision, vehicles);
        return true;
    }
    vehicle->current_speed = 0.0;
    vehicle->reason = "nlp_backtrack_start";
    status_ = MOVING;
    reason_ = "executing backtrack vehicle " + std::to_string(request_index_ + 1) +
              "/" + std::to_string(requests_.size());
    return true;
}

bool NlpBacktrackExecutor::begin(
    const sandbox_msgs::NlpBacktrackCommand& command, uint64_t recovery_id,
    uint32_t solve_round, const std::vector<int>& deadlock_cycle_ids,
    std::vector<VehicleAgent>& vehicles, std::string& rejection_reason) {
    if (active_) {
        rejection_reason = "a backtrack batch is already active";
        return false;
    }
    if (command.recovery_id != recovery_id || command.solve_round != solve_round ||
        !std::isfinite(command.speed) || command.speed <= 0.0 ||
        command.vehicle_ids.size() != command.path_gens.size() ||
        command.vehicle_ids.size() != command.target_s.size() ||
        command.vehicle_ids.size() != command.hold_at_source_b.size()) {
        rejection_reason = "backtrack transaction or command dimensions are invalid";
        return false;
    }
    recovery_id_ = command.recovery_id;
    solve_round_ = command.solve_round;
    speed_ = command.speed;
    requests_.clear();
    request_index_ = 0;
    completed_count_ = 0;
    skipped_count_ = 0;
    moved_count_ = 0;
    std::set<int> received;
    for (size_t i = 0; i < command.vehicle_ids.size(); ++i) {
        const int id = command.vehicle_ids[i];
        const VehicleAgent* vehicle = findVehicle(vehicles, id);
        if (vehicle == nullptr || vehicle->path_gen != command.path_gens[i] ||
            std::find(deadlock_cycle_ids.begin(), deadlock_cycle_ids.end(), id) ==
                deadlock_cycle_ids.end() || !received.insert(id).second ||
            !std::isfinite(command.target_s[i])) {
            rejection_reason = "backtrack participant or frozen path identity mismatch";
            requests_.clear();
            return false;
        }
        requests_.push_back({id, command.path_gens[i], command.target_s[i],
                             command.hold_at_source_b[i] != 0, 0.0});
    }
    if (requests_.empty()) {
        active_ = false;
        status_ = NO_EXECUTABLE;
        vehicle_id_ = -1;
        path_gen_ = -1;
        target_s_ = 0.0;
        reason_ = "planner reported no executable backtrack";
        return true;
    }
    active_ = true;
    status_ = MOVING;
    return startCurrent(vehicles);
}

void NlpBacktrackExecutor::step(double dt, uint64_t recovery_id,
                                std::vector<VehicleAgent>& vehicles) {
    if (!active_) return;
    VehicleAgent* vehicle = findVehicle(vehicles, vehicle_id_);
    if (recovery_id != recovery_id_ || vehicle == nullptr ||
        vehicle->path_gen != path_gen_) {
        if (vehicle != nullptr) vehicle->current_speed = 0.0;
        fail("recovery or path identity changed during backtrack");
        return;
    }
    const double remaining = vehicle->path_s - target_s_;
    const double tolerance = std::max(0.005, config_.path_validation_step);
    if (remaining <= tolerance && vehicle->current_speed <= config_.max_decel * dt + 1e-9) {
        const RoughWp target = vehicle->track.poseAtS(target_s_);
        int blocker_id = -1;
        if (!poseSafe(*vehicle, target, vehicles, &blocker_id)) {
            finishCurrentBlocked(*vehicle, {blocker_id, target_s_, target}, vehicles);
            return;
        }
        vehicle->path_s = target_s_;
        vehicle->real_pose_valid = true;
        vehicle->real_x = target.x;
        vehicle->real_y = target.y;
        vehicle->real_yaw = target.theta;
        vehicle->current_speed = 0.0;
        vehicle->reason = requests_[request_index_].hold_at_source_b ?
            "nlp_backtrack_source_b_hold" : "nlp_backtrack_arrived";
        const double actual_distance = requests_[request_index_].start_s - vehicle->path_s;
        if (actual_distance > tolerance) ++moved_count_;
        ++completed_count_;
        ROS_INFO("[NLP_BACKTRACK_DONE] V%d start_s=%.6f final_s=%.6f actual=%.6f",
                 vehicle->id, requests_[request_index_].start_s,
                 vehicle->path_s, actual_distance);
        ++request_index_;
        startCurrent(vehicles);
        return;
    }
    const double stopping_speed = std::sqrt(
        std::max(0.0, 2.0 * config_.max_decel * std::max(0.0, remaining)));
    const double desired = std::min(speed_, stopping_speed);
    double next_speed = vehicle->current_speed;
    const double delta = desired - next_speed;
    const double limit = (delta >= 0.0 ? config_.max_accel : config_.max_decel) * dt;
    next_speed += std::clamp(delta, -limit, limit);
    const double next_s = std::max(target_s_, vehicle->path_s - next_speed * dt);
    const RoughWp next_pose = vehicle->track.poseAtS(next_s);
    int blocker_id = -1;
    if (!poseSafe(*vehicle, next_pose, vehicles, &blocker_id)) {
        finishCurrentBlocked(*vehicle, {blocker_id, next_s, next_pose}, vehicles);
        return;
    }
    vehicle->path_s = next_s;
    vehicle->real_pose_valid = true;
    vehicle->real_x = next_pose.x;
    vehicle->real_y = next_pose.y;
    vehicle->real_yaw = next_pose.theta;
    vehicle->current_speed = next_speed;
    vehicle->action = VehicleAction::CREEP;
    vehicle->requested_action = VehicleAction::CREEP;
    vehicle->reason = next_pose.type == WpType::FORWARD ?
        "nlp_backtrack_reverse" : "nlp_backtrack_forward";
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
