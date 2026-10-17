#include "forklift_nlp_recovery/initial_guess.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <ros/ros.h>
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

struct DirectionSegment {
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

void appendDirectionSegment(std::vector<DirectionSegment>& segments,
                            double length, uint8_t type) {
    if (length <= kEpsilon) return;
    if (!segments.empty() && segments.back().type == type) {
        segments.back().length += length;
    } else {
        segments.push_back({length, type});
    }
}

bool buildReferenceMotion(const NlpVehicleInput& vehicle,
                          ReferenceMotion& reference, std::string& error) {
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
            !std::isfinite(point.pose.y) || !std::isfinite(point.pose.yaw)) {
            error = "original path contains non-finite geometry";
            return false;
        }
        if (i > 0 && point.s + kEpsilon < vehicle.original_path[i - 1].s) {
            error = "original path arc length is not monotonic";
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
        vehicle.terminal.path_s > last_s + kEpsilon) {
        error = "terminal path_s is behind the current state or outside the original path";
        return false;
    }
    if (vehicle.terminal.path_gen != vehicle.path_gen) {
        error = "terminal path_gen does not match the frozen path";
        return false;
    }

    reference.path_begin = std::clamp(vehicle.path_s, first_s, last_s);
    reference.path_end = std::clamp(vehicle.terminal.path_s, reference.path_begin, last_s);
    reference.projected_start = interpolatePath(vehicle.original_path, reference.path_begin);
    const PathPoint expected_terminal = interpolatePath(vehicle.original_path, reference.path_end);
    if (poseDistance(expected_terminal.pose, vehicle.terminal.pose) > 1e-4 ||
        angleDistance(expected_terminal.pose.yaw, vehicle.terminal.pose.yaw) > 1e-4) {
        error = "terminal pose does not match terminal path_s on the frozen path";
        return false;
    }

    reference.connector_length = poseDistance(
        vehicle.initial_state.rear_axle_pose, reference.projected_start.pose);
    reference.path_length = reference.path_end - reference.path_begin;
    reference.total_length = reference.connector_length + reference.path_length;
    appendDirectionSegment(reference.segments, reference.connector_length,
                           reference.projected_start.type);

    std::vector<double> boundaries{reference.path_begin, reference.path_end};
    for (size_t i = 1; i < vehicle.original_path.size(); ++i) {
        if (vehicle.original_path[i - 1].type == vehicle.original_path[i].type) continue;
        const double transition = 0.5 * (vehicle.original_path[i - 1].s +
                                         vehicle.original_path[i].s);
        if (transition > reference.path_begin + kEpsilon &&
            transition < reference.path_end - kEpsilon) {
            boundaries.push_back(transition);
        }
    }
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
        [](double first, double second) { return std::abs(first - second) <= kEpsilon; }),
        boundaries.end());
    for (size_t i = 1; i < boundaries.size(); ++i) {
        const double length = boundaries[i] - boundaries[i - 1];
        const uint8_t type = interpolatePath(
            vehicle.original_path, 0.5 * (boundaries[i] + boundaries[i - 1])).type;
        appendDirectionSegment(reference.segments, length, type);
    }
    reference.direction_changes = reference.segments.empty() ? 0 :
                                  reference.segments.size() - 1;

    if (reference.total_length <= 1e-6) {
        const double heading_change = angleDistance(
            vehicle.initial_state.rear_axle_pose.yaw, vehicle.terminal.pose.yaw);
        if (heading_change > M_PI / 180.0) {
            error = "zero-length reference requires a heading change that the kinematic bicycle cannot perform";
            return false;
        }
        if (std::abs(vehicle.initial_state.longitudinal_speed) > 1e-6) {
            error = "zero-length reference cannot stop the nonzero initial speed within the endpoint";
            return false;
        }
    }
    return true;
}

bool estimateTravelTime(const NlpVehicleInput& vehicle,
                        const ReferenceMotion& reference,
                        double& estimated_time, std::string& error) {
    const VehicleParameters& parameters = vehicle.parameters;
    if (!(parameters.max_speed > 0.0) || !(parameters.max_accel > 0.0) ||
        !(parameters.max_decel > 0.0) || !std::isfinite(parameters.max_speed) ||
        !std::isfinite(parameters.max_accel) || !std::isfinite(parameters.max_decel)) {
        error = "max_speed, max_accel and max_decel must be finite and positive";
        return false;
    }
    estimated_time = 0.0;
    double start_speed = std::abs(vehicle.initial_state.longitudinal_speed);
    for (const DirectionSegment& segment : reference.segments) {
        const double stopping_distance = start_speed * start_speed /
                                         (2.0 * parameters.max_decel);
        if (segment.length + 1e-8 < stopping_distance) {
            std::ostringstream reason;
            reason << "reference segment length " << segment.length
                   << " is shorter than the required stopping distance "
                   << stopping_distance;
            error = reason.str();
            return false;
        }
        const double speed_cap = std::max(parameters.max_speed, start_speed);
        const double peak_squared =
            (2.0 * parameters.max_accel * parameters.max_decel * segment.length +
             parameters.max_decel * start_speed * start_speed) /
            (parameters.max_accel + parameters.max_decel);
        const double peak_speed = std::sqrt(std::max(0.0, peak_squared));
        if (peak_speed <= speed_cap + kEpsilon) {
            estimated_time += std::max(0.0, peak_speed - start_speed) /
                              parameters.max_accel;
            estimated_time += peak_speed / parameters.max_decel;
        } else {
            const double acceleration_distance =
                std::max(0.0, speed_cap * speed_cap - start_speed * start_speed) /
                (2.0 * parameters.max_accel);
            const double deceleration_distance = speed_cap * speed_cap /
                                                 (2.0 * parameters.max_decel);
            const double cruise_distance = std::max(
                0.0, segment.length - acceleration_distance - deceleration_distance);
            estimated_time += std::max(0.0, speed_cap - start_speed) /
                              parameters.max_accel;
            estimated_time += cruise_distance / speed_cap;
            estimated_time += speed_cap / parameters.max_decel;
        }
        start_speed = 0.0;
    }
    return true;
}

Pose2d discCenter(const Pose2d& rear, const VehicleParameters& parameters) {
    return {rear.x + parameters.rear_axle_to_center * std::cos(rear.yaw),
            rear.y + parameters.rear_axle_to_center * std::sin(rear.yaw), rear.yaw};
}

bool contains(const Aabb& box, const Pose2d& point, double tolerance) {
    return point.x >= box.min_x - tolerance && point.x <= box.max_x + tolerance &&
           point.y >= box.min_y - tolerance && point.y <= box.max_y + tolerance;
}

int containingAabb(const std::vector<Aabb>& corridor, const Pose2d& point,
                   double tolerance) {
    for (size_t box = 0; box < corridor.size(); ++box) {
        if (contains(corridor[box], point, tolerance)) return static_cast<int>(box);
    }
    return -1;
}

int firstForwardAabb(const VehicleInitialGuess& guess, const NlpVehicleInput& input,
                     double tolerance) {
    const Pose2d& start = guess.nodes.front().disc_center;
    for (const Pose2d& seed : input.corridor_seeds) {
        if (poseDistance(seed, start) <= tolerance) continue;
        const int box = containingAabb(input.corridor, seed, tolerance);
        if (box >= 0) return box;
    }
    for (size_t node = 1; node < guess.nodes.size(); ++node) {
        const int box = containingAabb(
            input.corridor, guess.nodes[node].disc_center, tolerance);
        if (box >= 0) return box;
    }
    return -1;
}

std::vector<size_t> containingAabbs(const std::vector<Aabb>& corridor,
                                    const Pose2d& point, double tolerance) {
    std::vector<size_t> result;
    for (size_t box = 0; box < corridor.size(); ++box) {
        if (contains(corridor[box], point, tolerance)) result.push_back(box);
    }
    return result;
}

std::string indexList(const std::vector<size_t>& indices) {
    std::ostringstream text;
    text << '[';
    for (size_t i = 0; i < indices.size(); ++i) {
        if (i > 0) text << ',';
        text << indices[i];
    }
    text << ']';
    return text.str();
}

std::vector<size_t> reachableAabbs(const std::vector<int>& costs, int unreachable) {
    std::vector<size_t> result;
    for (size_t box = 0; box < costs.size(); ++box) {
        if (costs[box] != unreachable) result.push_back(box);
    }
    return result;
}

bool segmentIntersectsAabb(const Pose2d& first, const Pose2d& second,
                           const Aabb& box, double tolerance,
                           double& entry_ratio, double& exit_ratio) {
    entry_ratio = 0.0;
    exit_ratio = 1.0;
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const auto clip = [&](double origin, double delta, double minimum, double maximum) {
        minimum -= tolerance;
        maximum += tolerance;
        if (std::abs(delta) <= kEpsilon) return origin >= minimum && origin <= maximum;
        double near = (minimum - origin) / delta;
        double far = (maximum - origin) / delta;
        if (near > far) std::swap(near, far);
        entry_ratio = std::max(entry_ratio, near);
        exit_ratio = std::min(exit_ratio, far);
        return entry_ratio <= exit_ratio;
    };
    return clip(first.x, dx, box.min_x, box.max_x) &&
           clip(first.y, dy, box.min_y, box.max_y) &&
           entry_ratio <= 1.0 && exit_ratio >= 0.0;
}

void logAabbAssignmentFailure(const VehicleInitialGuess& guess,
                              const NlpVehicleInput& input, double tolerance,
                              const std::vector<std::vector<int>>& cost,
                              int unreachable, int logical_start_box,
                              const std::string& reason) {
    std::ostringstream log;
    log << std::fixed << std::setprecision(6);
    log << "[nlp_recovery][AABB_ASSIGNMENT_FAILED] V" << input.id
        << " reason=\"" << reason << "\""
        << " start_exempt=" << (guess.start_aabb_exempt ? "true" : "false")
        << " logical_start_aabb=";
    if (logical_start_box >= 0) log << logical_start_box;
    else log << "unresolved";
    log << " tolerance=" << tolerance << '\n';

    log << "AABBs:\n";
    for (size_t box = 0; box < input.corridor.size(); ++box) {
        const Aabb& bounds = input.corridor[box];
        log << "  box=" << box << " bounds=[" << bounds.min_x << ',' << bounds.max_x
            << "]x[" << bounds.min_y << ',' << bounds.max_y << "]\n";
    }
    log << "Adjacent intersections:\n";
    for (size_t box = 0; box + 1 < input.corridor.size(); ++box) {
        const Aabb& first = input.corridor[box];
        const Aabb& second = input.corridor[box + 1];
        const double min_x = std::max(first.min_x, second.min_x);
        const double max_x = std::min(first.max_x, second.max_x);
        const double min_y = std::max(first.min_y, second.min_y);
        const double max_y = std::min(first.max_y, second.max_y);
        const bool overlaps = min_x <= max_x + tolerance && min_y <= max_y + tolerance;
        log << "  " << box << "->" << box + 1 << " overlap="
            << (overlaps ? "true" : "false") << " intersection=[" << min_x << ','
            << max_x << "]x[" << min_y << ',' << max_y << "]\n";
    }

    log << "Nodes (disc center, candidates, DP reachable):\n";
    size_t first_empty = guess.nodes.size();
    for (size_t node = 0; node < guess.nodes.size(); ++node) {
        const Pose2d& center = guess.nodes[node].disc_center;
        const std::vector<size_t> candidates = containingAabbs(input.corridor, center, tolerance);
        const std::vector<size_t> reachable = node < cost.size() ?
            reachableAabbs(cost[node], unreachable) : std::vector<size_t>{};
        if (first_empty == guess.nodes.size() && reachable.empty()) first_empty = node;
        log << "  k=" << node << " disc=(" << center.x << ',' << center.y << ')'
            << " candidates=" << indexList(candidates)
            << " reachable=" << indexList(reachable) << '\n';
    }

    log << "firstForwardAabb audit:\n";
    log << "  implementation=corridor_seeds_array_order_then_discrete_nodes"
        << " (does not geometrically prove first path entry)\n";
    for (size_t seed = 0; seed < input.corridor_seeds.size(); ++seed) {
        const Pose2d& point = input.corridor_seeds[seed];
        log << "  seed_index=" << seed << " point=(" << point.x << ',' << point.y
            << ") candidates="
            << indexList(containingAabbs(input.corridor, point, tolerance))
            << " distance_from_start=" << poseDistance(point, guess.nodes.front().disc_center)
            << '\n';
    }
    const int implementation_choice = guess.start_aabb_exempt ?
        firstForwardAabb(guess, input, tolerance) : -1;
    int first_discrete_box = -1;
    size_t first_discrete_node = guess.nodes.size();
    for (size_t node = 1; node < guess.nodes.size(); ++node) {
        const std::vector<size_t> candidates = containingAabbs(
            input.corridor, guess.nodes[node].disc_center, tolerance);
        if (!candidates.empty()) {
            first_discrete_node = node;
            first_discrete_box = static_cast<int>(candidates.front());
            break;
        }
    }
    int dense_first_box = -1;
    size_t dense_interval = guess.nodes.size();
    double dense_ratio = 0.0;
    constexpr size_t kAuditSamplesPerInterval = 100;
    for (size_t node = 1; node < guess.nodes.size() && dense_first_box < 0; ++node) {
        for (size_t sample = 1; sample <= kAuditSamplesPerInterval; ++sample) {
            const double ratio = static_cast<double>(sample) / kAuditSamplesPerInterval;
            const Pose2d rear = interpolatePose(guess.nodes[node - 1].rear_axle_pose,
                                                guess.nodes[node].rear_axle_pose, ratio);
            const std::vector<size_t> candidates = containingAabbs(
                input.corridor, discCenter(rear, input.parameters), tolerance);
            if (!candidates.empty()) {
                dense_first_box = static_cast<int>(candidates.front());
                dense_interval = node - 1;
                dense_ratio = ratio;
                break;
            }
        }
    }
    log << "  implementation_choice=";
    if (guess.start_aabb_exempt) log << implementation_choice;
    else log << "not_invoked(start already covered)";
    log
        << " first_covered_discrete_node=";
    if (first_discrete_box >= 0) log << first_discrete_node << " box=" << first_discrete_box;
    else log << "none";
    log << " dense_guess_segment_first_entry=";
    if (dense_first_box >= 0) {
        log << "interval=" << dense_interval << "->" << dense_interval + 1
            << " ratio=" << dense_ratio << " box=" << dense_first_box;
    } else {
        log << "none";
    }
    if (guess.start_aabb_exempt && implementation_choice >= 0 && dense_first_box >= 0 &&
        implementation_choice != dense_first_box) {
        log << " MISMATCH(seed-order choice is not the first forward entry in dense audit)";
    }
    log << '\n';

    if (first_empty < guess.nodes.size()) {
        log << "First unreachable node: k=" << first_empty << '\n';
        const size_t begin = first_empty > 0 ? first_empty - 1 : 0;
        const size_t end = std::min(first_empty + 1, guess.nodes.size() - 1);
        for (size_t node = begin; node <= end; ++node) {
            const Pose2d& center = guess.nodes[node].disc_center;
            log << "  context k=" << node << " disc=(" << center.x << ',' << center.y
                << ") candidates="
                << indexList(containingAabbs(input.corridor, center, tolerance))
                << " reachable=" << indexList(reachableAabbs(cost[node], unreachable)) << '\n';
        }
        if (first_empty == 0) {
            log << "  rejection=START_LOGICAL_OWNERSHIP_UNRESOLVED candidates="
                << indexList(containingAabbs(input.corridor,
                                             guess.nodes.front().disc_center, tolerance))
                << " selected=" << logical_start_box << '\n';
        } else {
            const std::vector<size_t> previous_reachable =
                reachableAabbs(cost[first_empty - 1], unreachable);
            const std::vector<size_t> current_candidates = containingAabbs(
                input.corridor, guess.nodes[first_empty].disc_center, tolerance);
            if (current_candidates.empty()) {
                log << "  rejection=NODE_OUTSIDE_ALL_AABBS\n";
            }
            for (size_t previous : previous_reachable) {
                for (size_t box : current_candidates) {
                    log << "  transition " << previous << "->" << box << " rejected=";
                    if (box < previous) {
                        log << "AABB_NUMBER_BACKTRACK (current < previous)";
                    } else if (box > previous + 1) {
                        log << "AABB_NUMBER_SKIP (current > previous + 1)";
                    } else if (box != previous) {
                        const bool previous_node_in_current = contains(
                            input.corridor[box], guess.nodes[first_empty - 1].disc_center,
                            tolerance);
                        const bool current_node_in_previous = contains(
                            input.corridor[previous], guess.nodes[first_empty].disc_center,
                            tolerance);
                        if (!previous_node_in_current && !current_node_in_previous) {
                            const Aabb intersection{
                                std::max(input.corridor[previous].min_x,
                                         input.corridor[box].min_x),
                                std::min(input.corridor[previous].max_x,
                                         input.corridor[box].max_x),
                                std::max(input.corridor[previous].min_y,
                                         input.corridor[box].min_y),
                                std::min(input.corridor[previous].max_y,
                                         input.corridor[box].max_y)};
                            const bool boxes_overlap =
                                intersection.min_x <= intersection.max_x + tolerance &&
                                intersection.min_y <= intersection.max_y + tolerance;
                            double entry_ratio = 0.0;
                            double exit_ratio = 0.0;
                            const bool segment_crosses_overlap = boxes_overlap &&
                                segmentIntersectsAabb(
                                    guess.nodes[first_empty - 1].disc_center,
                                    guess.nodes[first_empty].disc_center,
                                    intersection, tolerance, entry_ratio, exit_ratio);
                            log << "ADJACENT_OVERLAP_TRANSITION_CONDITION"
                                << " previous_node_in_current=false"
                                << " current_node_in_previous=false"
                                << " boxes_overlap=" << (boxes_overlap ? "true" : "false")
                                << " node_segment_crosses_overlap="
                                << (segment_crosses_overlap ? "true" : "false");
                            if (segment_crosses_overlap) {
                                log << " segment_ratio=[" << std::max(0.0, entry_ratio)
                                    << ',' << std::min(1.0, exit_ratio) << ']';
                            }
                        } else {
                            log << "none (transition should be reachable; inspect DP cost)";
                        }
                    } else {
                        log << "none (same-box transition should be reachable; inspect DP cost)";
                    }
                    log << '\n';
                }
            }
        }
    }
    ROS_ERROR_STREAM(log.str());
}

bool assignAabbsImpl(VehicleInitialGuess& guess, const NlpVehicleInput& input,
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
    int logical_start_box = -1;

    for (size_t box = 0; box < box_count; ++box) {
        if (contains(input.corridor[box], guess.nodes.front().disc_center, tolerance)) {
            cost[0][box] = static_cast<int>(box);
            if (logical_start_box < 0) logical_start_box = static_cast<int>(box);
        }
    }
    const bool start_covered = std::any_of(cost.front().begin(), cost.front().end(),
        [&](int value) { return value != unreachable; });
    if (!start_covered) {
        const int first_forward_box = firstForwardAabb(guess, input, tolerance);
        if (first_forward_box < 0) {
            reason = "start disc center is outside every AABB and the forward reference path never enters one";
            logAabbAssignmentFailure(guess, input, tolerance, cost, unreachable,
                                     logical_start_box, reason);
            return false;
        }
        guess.start_aabb_exempt = true;
        logical_start_box = first_forward_box;
        cost[0][static_cast<size_t>(first_forward_box)] = first_forward_box;
    }
    for (size_t node = 1; node < node_count; ++node) {
        for (size_t box = 0; box < box_count; ++box) {
            if (!contains(input.corridor[box], guess.nodes[node].disc_center, tolerance)) continue;
            for (size_t previous = 0; previous < box_count; ++previous) {
                if (cost[node - 1][previous] == unreachable || box < previous || box > previous + 1) continue;
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
        for (size_t node = guess.start_aabb_exempt ? 1 : 0; node < node_count; ++node) {
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
        logAabbAssignmentFailure(guess, input, tolerance, cost, unreachable,
                                 logical_start_box, reason);
        return false;
    }

    int box = static_cast<int>(std::distance(cost.back().begin(), best));
    for (size_t reverse = node_count; reverse-- > 0;) {
        guess.nodes[reverse].aabb_index = box;
        if (reverse > 0) box = predecessor[reverse][static_cast<size_t>(box)];
    }
    if (guess.start_aabb_exempt) {
        ROS_WARN("[nlp_recovery][AABB_START_EXEMPT] V%d start outside all AABBs; assign first forward AABB=%d; skip start AABB constraint and required entry-interval check.",
                 input.id, guess.nodes.front().aabb_index);
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

bool writeInitialGuessCsv(const NlpInitialGuess& guess,
                          const std::filesystem::path& path,
                          const std::string& case_id,
                          const std::string& generated_at,
                          std::string& error) {
    std::ofstream csv(path, std::ios::trunc);
    if (!csv) {
        error = "cannot open " + path.filename().string();
        return false;
    }
    csv << "case_id,generated_at,vehicle_id,node_index,time,path_s,path_type,"
           "rear_x,rear_y,heading,signed_speed,front_steer,acceleration,steer_rate,"
           "aabb_index,disc_center_x,disc_center_y,speed_source,steer_source,"
           "acceleration_source,steer_rate_source,kinematic_position_residual,"
           "kinematic_heading_residual,grid_node_count,grid_total_time,"
           "grid_average_dt,reference_length,estimated_travel_time,direction_changes\n";
    csv << std::fixed << std::setprecision(9);
    for (const VehicleInitialGuess& vehicle : guess.vehicles) {
        for (const InitialGuessNode& node : vehicle.nodes) {
            csv << case_id << ',' << generated_at << ',' << vehicle.id << ','
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
                << node.kinematic_heading_residual << ',' << guess.node_count << ','
                << guess.total_time << ',' << guess.average_dt << ','
                << vehicle.diagnostics.reference_length << ','
                << vehicle.diagnostics.estimated_travel_time << ','
                << vehicle.diagnostics.direction_changes << '\n';
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

bool assignInitialGuessAabbs(VehicleInitialGuess& guess,
                             const NlpVehicleInput& input,
                             double tolerance, std::string& reason) {
    guess.start_aabb_exempt = false;
    for (InitialGuessNode& node : guess.nodes) node.aabb_index = -1;
    return assignAabbsImpl(guess, input, tolerance, reason);
}

NlpInitialGuess InitialGuessGenerator::generate(const NlpOptimizationInput& input) const {
    NlpInitialGuess result;
    bool all_assignments_valid = true;
    std::vector<std::string> assignment_failures;
    result.automatic_timing = config_.automatic;
    if (!input.valid) {
        result.reason = "optimization input is invalid: " + input.reason;
        return result;
    }
    if (input.vehicles.empty()) {
        result.reason = "optimization input has no participant vehicles";
        return result;
    }

    std::vector<ReferenceMotion> references(input.vehicles.size());
    std::vector<double> estimated_times(input.vehicles.size(), 0.0);
    size_t feature_nodes = 2;
    double maximum_estimated_time = 0.0;
    for (size_t i = 0; i < input.vehicles.size(); ++i) {
        const NlpVehicleInput& vehicle = input.vehicles[i];
        if (!(vehicle.parameters.wheel_base > 0.0) ||
            !(vehicle.parameters.disc_radius > 0.0)) {
            result.reason = "V" + std::to_string(vehicle.id) +
                            ": invalid wheel base or disc radius";
            return result;
        }
        std::string error;
        if (!buildReferenceMotion(vehicle, references[i], error)) {
            result.reason = "V" + std::to_string(vehicle.id) + ": " + error;
            return result;
        }
        if (!estimateTravelTime(vehicle, references[i], estimated_times[i], error)) {
            result.reason = "V" + std::to_string(vehicle.id) +
                            " timing failed: " + error;
            return result;
        }
        maximum_estimated_time = std::max(maximum_estimated_time, estimated_times[i]);
        feature_nodes = std::max(feature_nodes,
            3 + 2 * references[i].direction_changes);
        feature_nodes = std::max(feature_nodes, 1 + 2 * vehicle.corridor.size());
    }

    if (config_.automatic) {
        if (!(config_.target_dt > 0.0) || !std::isfinite(config_.target_dt)) {
            result.reason = "initial_guess_target_dt must be finite and positive";
            return result;
        }
        if (config_.min_nodes < 2 || config_.max_nodes < config_.min_nodes) {
            result.reason = "initial guess node limits are invalid";
            return result;
        }
        if (!(config_.time_margin >= 1.0) || !std::isfinite(config_.time_margin)) {
            result.reason = "initial_guess_time_margin must be finite and at least 1.0";
            return result;
        }
        if (feature_nodes > config_.max_nodes) {
            result.reason = "initial_guess_max_nodes is too small for corridor and direction changes";
            return result;
        }
        result.total_time = maximum_estimated_time * config_.time_margin;
        size_t time_nodes = result.total_time > kEpsilon ?
            static_cast<size_t>(std::ceil(result.total_time / config_.target_dt)) + 1 :
            config_.min_nodes;
        result.node_count = std::clamp(
            std::max({time_nodes, config_.min_nodes, feature_nodes}),
            config_.min_nodes, config_.max_nodes);
        if (result.total_time <= kEpsilon) {
            result.total_time = config_.target_dt *
                                static_cast<double>(result.node_count - 1);
        }
    } else {
        if (config_.node_count < 2) {
            result.reason = "initial_guess_nodes must be at least 2";
            return result;
        }
        if (!(config_.total_time > 0.0) || !std::isfinite(config_.total_time)) {
            result.reason = "initial_guess_total_time must be finite and positive";
            return result;
        }
        result.node_count = config_.node_count;
        result.total_time = config_.total_time;
    }

    result.average_dt = result.total_time /
                        static_cast<double>(result.node_count - 1);
    const double dt = result.average_dt;
    result.time_grid.reserve(result.node_count);
    for (size_t node = 0; node < result.node_count; ++node) {
        result.time_grid.push_back(dt * static_cast<double>(node));
    }

    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size();
         ++vehicle_index) {
        const NlpVehicleInput& input_vehicle = input.vehicles[vehicle_index];
        const ReferenceMotion& reference = references[vehicle_index];
        VehicleInitialGuess vehicle;
        vehicle.id = input_vehicle.id;
        vehicle.diagnostics.reference_length = reference.total_length;
        vehicle.diagnostics.estimated_travel_time = estimated_times[vehicle_index];
        vehicle.diagnostics.direction_changes = reference.direction_changes;
        vehicle.nodes.resize(result.node_count);

        for (size_t node = 0; node < result.node_count; ++node) {
            InitialGuessNode& state = vehicle.nodes[node];
            state.index = node;
            state.time = result.time_grid[node];
            const double ratio = static_cast<double>(node) /
                                 static_cast<double>(result.node_count - 1);
            const double progress = ratio * reference.total_length;
            if (node == 0) {
                state.rear_axle_pose = input_vehicle.initial_state.rear_axle_pose;
                state.path_s = reference.path_begin;
                state.path_type = reference.projected_start.type;
            } else if (node + 1 == result.node_count) {
                state.rear_axle_pose = input_vehicle.terminal.pose;
                state.path_s = input_vehicle.terminal.path_s;
                state.path_type = input_vehicle.terminal.type;
            } else if (reference.connector_length > kEpsilon &&
                       progress < reference.connector_length) {
                state.rear_axle_pose = interpolatePose(
                    input_vehicle.initial_state.rear_axle_pose,
                    reference.projected_start.pose,
                    progress / reference.connector_length);
                state.path_s = reference.path_begin;
                state.path_type = reference.projected_start.type;
            } else {
                const double path_progress = std::max(
                    0.0, progress - reference.connector_length);
                const PathPoint sample = interpolatePath(
                    input_vehicle.original_path,
                    std::min(reference.path_end,
                             reference.path_begin + path_progress));
                state.rear_axle_pose = sample.pose;
                state.path_s = sample.s;
                state.path_type = sample.type;
            }
            state.disc_center = discCenter(state.rear_axle_pose, input_vehicle.parameters);
        }

        for (size_t node = 0; node < result.node_count; ++node) {
            InitialGuessNode& state = vehicle.nodes[node];
            const double direction = state.path_type == 1 ? -1.0 : 1.0;
            if (node == 0) {
                state.signed_speed = direction * std::abs(input_vehicle.initial_state.longitudinal_speed);
                state.speed_source = GuessValueSource::SNAPSHOT_MAGNITUDE_PATH_SIGN;
            } else if (node + 1 == result.node_count) {
                state.signed_speed = 0.0;
                state.speed_source = GuessValueSource::TERMINAL_ZERO_ASSUMPTION;
            } else {
                const double distance = poseDistance(vehicle.nodes[node - 1].rear_axle_pose,
                                                     vehicle.nodes[node + 1].rear_axle_pose);
                state.signed_speed = direction * distance / (2.0 * dt);
                state.speed_source = GuessValueSource::PATH_TIME_DIFFERENCE;
            }
        }
        for (size_t node = 0; node + 1 < result.node_count; ++node) {
            if (vehicle.nodes[node].path_type != vehicle.nodes[node + 1].path_type) {
                vehicle.nodes[node].signed_speed = 0.0;
                vehicle.nodes[node + 1].signed_speed = 0.0;
            }
        }

        for (size_t node = 0; node < result.node_count; ++node) {
            InitialGuessNode& state = vehicle.nodes[node];
            const size_t lo = node == 0 ? 0 : node - 1;
            const size_t hi = node + 1 < result.node_count ? node + 1 : node;
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

        for (size_t node = 0; node < result.node_count; ++node) {
            const size_t lo = node == 0 ? 0 : node - 1;
            const size_t hi = node + 1 < result.node_count ? node + 1 : node;
            const double span = static_cast<double>(hi - lo) * dt;
            vehicle.nodes[node].acceleration = span > 0.0 ?
                (vehicle.nodes[hi].signed_speed - vehicle.nodes[lo].signed_speed) / span : 0.0;
            vehicle.nodes[node].steer_rate = span > 0.0 ?
                normalizeAngle(vehicle.nodes[hi].front_steer - vehicle.nodes[lo].front_steer) / span : 0.0;
        }

        std::string assignment_reason;
        vehicle.aabb_assignment_valid = assignInitialGuessAabbs(
            vehicle, input_vehicle, config_.containment_tolerance, assignment_reason);
        if (!vehicle.aabb_assignment_valid) {
            vehicle.reason = assignment_reason;
            vehicle.valid = false;
            all_assignments_valid = false;
            assignment_failures.push_back("V" + std::to_string(input_vehicle.id) +
                " AABB assignment failed: " + assignment_reason);
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
        for (size_t node = 0; node < result.node_count; ++node) {
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

            if (node + 1 < result.node_count) {
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
        vehicle.valid = vehicle.aabb_assignment_valid;
        if (vehicle.valid) vehicle.reason = "ok";
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
            for (size_t node = 0; node < result.node_count; ++node) {
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
            for (size_t node = 0; node < result.node_count; ++node) {
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

    result.generated = all_assignments_valid;
    const bool dynamics_close = result.diagnostics.max_kinematic_position_residual <= 1e-3 &&
                                result.diagnostics.max_kinematic_heading_residual <= 1e-3;
    result.constraints_satisfied = result.diagnostics.bound_violation_nodes == 0 &&
        result.diagnostics.participant_collision_nodes == 0 &&
        result.diagnostics.static_collision_nodes == 0 && dynamics_close;
    if (!all_assignments_valid) {
        std::ostringstream reason;
        for (size_t i = 0; i < assignment_failures.size(); ++i) {
            if (i != 0) reason << "; ";
            reason << assignment_failures[i];
        }
        result.reason = reason.str();
    } else {
        result.reason = result.constraints_satisfied ?
            "generated and constraint checks passed" :
            "generated with constraint violations; retained for NLP initialization";
    }
    return result;
}

CaseSaveResult saveInitialGuessCase(const OptimizationInputBuildResult& input,
                                    const NlpInitialGuess& original_guess,
                                    const NlpInitialGuess& repaired_guess,
                                    const std::string& results_directory,
                                    const std::string& case_prefix) {
    CaseSaveResult result;
    if (original_guess.vehicles.empty() || repaired_guess.vehicles.empty()) {
        result.reason = "original or repaired initial guess has no vehicle data";
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
        if (case_prefix.empty()) {
            name << "case_" << std::setw(4) << std::setfill('0') << number;
        } else {
            name << case_prefix << '_' << number;
        }
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
    std::ofstream yaml(case_path / "optimization_input.yaml", std::ios::app);
    if (!yaml) {
        result.reason = "cannot append initial guess timing to optimization_input.yaml";
        return result;
    }
    yaml << std::boolalpha << std::fixed << std::setprecision(9)
         << "initial_guess_grid: {automatic: " << repaired_guess.automatic_timing
         << ", node_count: " << repaired_guess.node_count
         << ", total_time: " << repaired_guess.total_time
         << ", average_dt: " << repaired_guess.average_dt << "}\n";
    yaml.close();
    if (!yaml) {
        result.reason = "failed while appending initial guess timing";
        return result;
    }
    const std::string generated_at = isoTimestamp();
    if (!writeInitialGuessCsv(original_guess, case_path / "original_initial_guess.csv",
                              result.case_id, generated_at, result.reason) ||
        !writeInitialGuessCsv(repaired_guess, case_path / "repaired_initial_guess.csv",
                              result.case_id, generated_at, result.reason) ||
        !writeInitialGuessCsv(repaired_guess, case_path / "initial_guess.csv",
                              result.case_id, generated_at, result.reason)) {
        return result;
    }
    result.success = true;
    result.case_directory = case_path.string();
    result.reason = "ok";
    return result;
}

CaseSaveResult saveInitialGuessCase(const OptimizationInputBuildResult& input,
                                    const NlpInitialGuess& guess,
                                    const std::string& results_directory) {
    return saveInitialGuessCase(input, guess, guess, results_directory, "");
}

}  // namespace forklift_nlp_recovery
