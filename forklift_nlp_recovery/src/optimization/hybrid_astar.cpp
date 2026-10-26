#include "forklift_nlp_recovery/hybrid_astar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace forklift_nlp_recovery {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct SearchNode {
    Pose2d pose;
    int direction = 1;
    int parent = -1;
    double cost = 0.0;
};

struct QueueEntry {
    double score = 0.0;
    int node = -1;
    bool operator<(const QueueEntry& other) const { return score > other.score; }
};

struct StateKey {
    int x = 0;
    int y = 0;
    int yaw = 0;
    int direction = 0;
    bool operator==(const StateKey& other) const {
        return x == other.x && y == other.y && yaw == other.yaw &&
               direction == other.direction;
    }
};

struct StateKeyHash {
    size_t operator()(const StateKey& key) const {
        size_t value = static_cast<size_t>(key.x) * 73856093u;
        value ^= static_cast<size_t>(key.y) * 19349663u;
        value ^= static_cast<size_t>(key.yaw) * 83492791u;
        value ^= static_cast<size_t>(key.direction + 2) * 2654435761u;
        return value;
    }
};

Pose2d bodyCenter(const Pose2d& rear, double offset) {
    return {rear.x + offset * std::cos(rear.yaw),
            rear.y + offset * std::sin(rear.yaw), rear.yaw};
}

bool discIntersectsObb(const Pose2d& center, double radius,
                       const StaticVehicleObstacle& obstacle) {
    const double dx = center.x - obstacle.disc_center.x;
    const double dy = center.y - obstacle.disc_center.y;
    const double c = std::cos(obstacle.rear_axle_pose.yaw);
    const double s = std::sin(obstacle.rear_axle_pose.yaw);
    const double local_x = c * dx + s * dy;
    const double local_y = -s * dx + c * dy;
    const double outside_x = std::max(std::abs(local_x) - obstacle.length * 0.5, 0.0);
    const double outside_y = std::max(std::abs(local_y) - obstacle.width * 0.5, 0.0);
    return outside_x * outside_x + outside_y * outside_y <= radius * radius + kEpsilon;
}

bool poseLegal(const Pose2d& rear, const NlpVehicleInput& vehicle,
               const std::vector<StaticVehicleObstacle>& obstacles) {
    if (vehicle.corridor.empty()) return false;
    const Pose2d center = bodyCenter(rear, vehicle.parameters.rear_axle_to_center);
    const Aabb& stc = vehicle.corridor.front();
    if (center.x < stc.min_x - kEpsilon || center.x > stc.max_x + kEpsilon ||
        center.y < stc.min_y - kEpsilon || center.y > stc.max_y + kEpsilon) return false;
    for (const auto& obstacle : obstacles) {
        if (discIntersectsObb(center, vehicle.parameters.disc_radius, obstacle)) return false;
    }
    return true;
}

Pose2d integrate(const Pose2d& pose, int direction, double steer,
                 double distance, double wheel_base) {
    Pose2d result = pose;
    const double signed_distance = static_cast<double>(direction) * distance;
    const double curvature = std::tan(steer) / wheel_base;
    if (std::abs(curvature) < 1e-9) {
        result.x += signed_distance * std::cos(result.yaw);
        result.y += signed_distance * std::sin(result.yaw);
    } else {
        const double next_yaw = result.yaw + signed_distance * curvature;
        result.x += (std::sin(next_yaw) - std::sin(result.yaw)) / curvature;
        result.y += (-std::cos(next_yaw) + std::cos(result.yaw)) / curvature;
        result.yaw = normalizeAngle(next_yaw);
    }
    return result;
}

double heuristic(const Pose2d& pose, const Pose2d& goal) {
    return std::hypot(goal.x - pose.x, goal.y - pose.y) +
           0.12 * std::abs(normalizeAngle(goal.yaw - pose.yaw));
}

StateKey stateKey(const Pose2d& pose, int direction,
                  const HybridAstarConfig& config) {
    const double wrapped = normalizeAngle(pose.yaw);
    int yaw = static_cast<int>(std::floor((wrapped + kPi) /
        (2.0 * kPi) * static_cast<double>(config.yaw_bins)));
    yaw = std::clamp(yaw, 0, config.yaw_bins - 1);
    return {static_cast<int>(std::floor(pose.x / config.xy_resolution)),
            static_cast<int>(std::floor(pose.y / config.xy_resolution)), yaw,
            direction};
}

bool connectGoal(const SearchNode& node, const NlpVehicleInput& vehicle,
                 const std::vector<StaticVehicleObstacle>& obstacles,
                 const HybridAstarConfig& config) {
    const Pose2d& goal = vehicle.terminal.pose;
    const double distance = std::hypot(goal.x - node.pose.x, goal.y - node.pose.y);
    const double yaw_error = std::abs(normalizeAngle(goal.yaw - node.pose.yaw));
    const int required_direction = vehicle.terminal.type == 1 ? -1 : 1;
    if (node.direction != required_direction ||
        distance > config.goal_position_tolerance ||
        yaw_error > config.goal_yaw_tolerance) return false;
    if (distance > kEpsilon) {
        const double curvature = yaw_error / distance;
        if (curvature > std::tan(vehicle.parameters.max_steer_angle) /
                            vehicle.parameters.wheel_base + 1e-6) return false;
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(
        distance / config.collision_check_step)));
    for (int i = 1; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / samples;
        Pose2d sample{node.pose.x + ratio * (goal.x - node.pose.x),
                      node.pose.y + ratio * (goal.y - node.pose.y),
                      normalizeAngle(node.pose.yaw + ratio *
                          normalizeAngle(goal.yaw - node.pose.yaw))};
        if (!poseLegal(sample, vehicle, obstacles)) return false;
    }
    return true;
}

}  // namespace

HybridAstarPlanner::HybridAstarPlanner(const HybridAstarConfig& config)
    : config_(config) {}

HybridAstarResult HybridAstarPlanner::plan(
    const NlpVehicleInput& vehicle,
    const std::vector<StaticVehicleObstacle>& obstacles) const {
    HybridAstarResult result;
    if (vehicle.corridor.size() != 1 || config_.integration_step <= 0.0 ||
        config_.collision_check_step <= 0.0 || config_.xy_resolution <= 0.0 ||
        config_.yaw_bins < 8 || config_.max_expansions <= 0 ||
        vehicle.parameters.wheel_base <= 0.0 ||
        vehicle.parameters.max_steer_angle <= 0.0) {
        result.reason = "invalid Hybrid A* input or configuration";
        return result;
    }
    if (!poseLegal(vehicle.initial_state.rear_axle_pose, vehicle, obstacles) ||
        !poseLegal(vehicle.terminal.pose, vehicle, obstacles)) {
        result.reason = "Hybrid A* start or fixed terminal is in collision";
        return result;
    }
    std::vector<SearchNode> nodes;
    nodes.push_back({vehicle.initial_state.rear_axle_pose, 0, -1, 0.0});
    std::priority_queue<QueueEntry> open;
    open.push({heuristic(nodes.front().pose, vehicle.terminal.pose), 0});
    std::unordered_map<StateKey, double, StateKeyHash> best;
    const double steer_values[3] = {-vehicle.parameters.max_steer_angle, 0.0,
                                     vehicle.parameters.max_steer_angle};
    int goal_node = -1;
    while (!open.empty() && result.expansions < config_.max_expansions) {
        const int current_index = open.top().node;
        open.pop();
        const SearchNode current = nodes[current_index];
        ++result.expansions;
        if (current.direction != 0 &&
            connectGoal(current, vehicle, obstacles, config_)) {
            goal_node = current_index;
            break;
        }
        for (const int direction : {-1, 1}) {
            for (const double steer : steer_values) {
                Pose2d pose = current.pose;
                bool legal = true;
                const int substeps = std::max(1, static_cast<int>(std::ceil(
                    config_.integration_step / config_.collision_check_step)));
                for (int substep = 0; substep < substeps; ++substep) {
                    pose = integrate(pose, direction, steer,
                        config_.integration_step / substeps,
                        vehicle.parameters.wheel_base);
                    if (!poseLegal(pose, vehicle, obstacles)) {
                        legal = false;
                        break;
                    }
                }
                if (!legal) continue;
                const double movement_cost = config_.integration_step *
                    (direction < 0 ? config_.reverse_cost : 1.0);
                const double switch_cost = current.direction != 0 &&
                    current.direction != direction ? config_.direction_switch_cost : 0.0;
                const double cost = current.cost + movement_cost + switch_cost +
                    0.01 * std::abs(steer / vehicle.parameters.max_steer_angle);
                const StateKey key = stateKey(pose, direction, config_);
                const auto known = best.find(key);
                if (known != best.end() && known->second <= cost + kEpsilon) continue;
                best[key] = cost;
                const int index = static_cast<int>(nodes.size());
                nodes.push_back({pose, direction, current_index, cost});
                open.push({cost + heuristic(pose, vehicle.terminal.pose), index});
            }
        }
    }
    if (goal_node < 0) {
        result.reason = result.expansions >= config_.max_expansions ?
            "Hybrid A* expansion limit reached" : "Hybrid A* open set exhausted";
        return result;
    }
    std::vector<SearchNode> reversed;
    for (int index = goal_node; index >= 0; index = nodes[index].parent) {
        reversed.push_back(nodes[index]);
    }
    std::reverse(reversed.begin(), reversed.end());
    result.path.reserve(reversed.size() + 1);
    double s = 0.0;
    for (size_t i = 0; i < reversed.size(); ++i) {
        if (i > 0) s += std::hypot(reversed[i].pose.x - reversed[i - 1].pose.x,
                                  reversed[i].pose.y - reversed[i - 1].pose.y);
        const int direction = reversed[i].direction == 0 ?
            (reversed.size() > 1 ? reversed[1].direction : 1) : reversed[i].direction;
        result.path.push_back({reversed[i].pose, s,
                               direction < 0 ? uint8_t{1} : uint8_t{0}});
    }
    s += std::hypot(vehicle.terminal.pose.x - result.path.back().pose.x,
                    vehicle.terminal.pose.y - result.path.back().pose.y);
    result.path.push_back({vehicle.terminal.pose, s, vehicle.terminal.type});
    result.success = true;
    result.reason = "ok";
    return result;
}

}  // namespace forklift_nlp_recovery
