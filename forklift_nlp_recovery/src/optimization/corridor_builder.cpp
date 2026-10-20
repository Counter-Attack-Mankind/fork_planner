#include "forklift_nlp_recovery/corridor_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

bool overlapsWithArea(const Aabb& a, const Aabb& b) {
    const double overlap_x = std::min(a.max_x, b.max_x) - std::max(a.min_x, b.min_x);
    const double overlap_y = std::min(a.max_y, b.max_y) - std::max(a.min_y, b.min_y);
    return overlap_x > kEpsilon && overlap_y > kEpsilon;
}

bool contains(const Aabb& outer, const Aabb& inner) {
    return outer.min_x <= inner.min_x + kEpsilon &&
           outer.max_x + kEpsilon >= inner.max_x &&
           outer.min_y <= inner.min_y + kEpsilon &&
           outer.max_y + kEpsilon >= inner.max_y;
}

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

CorridorBuilder::CorridorBuilder(const MapParam& map_param,
                                 const std::vector<ShelfBlock>& shelves,
                                 const CorridorConfig& config)
    : map_param_(map_param), config_(config) {
    const double geometric_radius = std::hypot(map_param_.vehicle_length * 0.5,
                                               map_param_.vehicle_width * 0.5);
    vehicle_radius_ = config_.vehicle_radius > 0.0
        ? config_.vehicle_radius : geometric_radius;
    boundary_clearance_ = std::max(0.0, vehicle_radius_ + config_.safety_margin - config_.boundary_inset_reduction);
    shelf_inflation_ = std::max(
        0.0, vehicle_radius_ + config_.safety_margin -
                 config_.shelf_inflation_reduction);

    
    original_shelves_.reserve(shelves.size());
    for (const auto& shelf : shelves) {
        original_shelves_.push_back({
        shelf.x,
        shelf.x_max(),
        shelf.y,
        shelf.y_max()
        });
    }

}

Pose2d CorridorBuilder::bodyCenter(const Pose2d& rear_axle_pose) const {
    Pose2d center = rear_axle_pose;
    center.x += map_param_.rear_axle_to_center * std::cos(rear_axle_pose.yaw);
    center.y += map_param_.rear_axle_to_center * std::sin(rear_axle_pose.yaw);
    return center;
}

bool CorridorBuilder::pointLegal(const Pose2d& point) const {
    const Aabb seed{point.x, point.x, point.y, point.y};
    return aabbLegal(seed);
}

bool CorridorBuilder::aabbLegal(const Aabb& box) const {
    if (box.min_x < boundary_clearance_ - kEpsilon ||
        box.max_x > map_param_.field_width - boundary_clearance_ + kEpsilon ||
        box.min_y < boundary_clearance_ - kEpsilon ||
        box.max_y > map_param_.field_height - boundary_clearance_ + kEpsilon) {
        return false;
    }

    for (const auto& shelf : original_shelves_) {
        // 两个 AABB 在 X、Y 方向的间距
        const double dx = std::max({
            0.0,
            shelf.min_x - box.max_x,
            box.min_x - shelf.max_x
        });

        const double dy = std::max({
            0.0,
            shelf.min_y - box.max_y,
            box.min_y - shelf.max_y
        });

        // 两个矩形内部发生重叠
        const bool interior_overlap =
            box.max_x > shelf.min_x + kEpsilon &&
            box.min_x < shelf.max_x - kEpsilon &&
            box.max_y > shelf.min_y + kEpsilon &&
            box.min_y < shelf.max_y - kEpsilon;

        if (interior_overlap) {
            return false;
        }

        // 两个矩形之间的最小欧氏距离
        const double distance = std::hypot(dx, dy);

        if (distance < shelf_inflation_ - kEpsilon) {
            return false;
        }
    }

    return true;
}

bool CorridorBuilder::connected(const Aabb& first, const Aabb& second) const {
    return overlapsWithArea(first, second);
}

bool CorridorBuilder::generateAabb(const Pose2d& seed, Aabb& box,
                                   std::string& reason) const {
    if (!pointLegal(seed)) {
        std::ostringstream out;
        out << "seed (" << seed.x << ", " << seed.y
            << ") is inside an inflated shelf or outside the inset map boundary";
        reason = out.str();
        return false;
    }
    box = {seed.x, seed.x, seed.y, seed.y};
    bool open[4] = {true, true, true, true};  // up, down, left, right
    while (open[0] || open[1] || open[2] || open[3]) {
        for (int direction = 0; direction < 4; ++direction) {
            if (!open[direction]) continue;
            Aabb candidate = box;
            double room = 0.0;
            if (direction == 0) room = config_.max_height - box.height();
            if (direction == 1) room = config_.max_height - box.height();
            if (direction == 2) room = config_.max_width - box.width();
            if (direction == 3) room = config_.max_width - box.width();
            if (room <= kEpsilon) {
                open[direction] = false;
                continue;
            }
            const double delta = std::min(config_.expansion_step, room);
            if (direction == 0) candidate.max_y += delta;
            if (direction == 1) candidate.min_y -= delta;
            if (direction == 2) candidate.min_x -= delta;
            if (direction == 3) candidate.max_x += delta;
            if (aabbLegal(candidate)) box = candidate;
            else open[direction] = false;
        }
    }
    reason.clear();
    return true;
}

std::vector<PathPoint> CorridorBuilder::parameterizePath(
    const std::vector<Pose2d>& poses, const std::vector<uint8_t>& types) {
    std::vector<PathPoint> path;
    path.reserve(poses.size());
    double s = 0.0;
    for (size_t i = 0; i < poses.size(); ++i) {
        if (i > 0) s += std::hypot(poses[i].x - poses[i - 1].x,
                                  poses[i].y - poses[i - 1].y);
        path.push_back({poses[i], s, i < types.size() ? types[i] : uint8_t{0}});
    }
    return path;
}

PathPoint CorridorBuilder::interpolate(const std::vector<PathPoint>& path,
                                       double s) const {
    if (s <= path.front().s) return path.front();
    if (s >= path.back().s) return path.back();
    const auto upper = std::upper_bound(
        path.begin(), path.end(), s,
        [](double value, const PathPoint& point) { return value < point.s; });
    const PathPoint& second = *upper;
    const PathPoint& first = *(upper - 1);
    const double span = second.s - first.s;
    const double ratio = span > kEpsilon ? (s - first.s) / span : 0.0;
    PathPoint result;
    result.s = s;
    result.pose.x = first.pose.x + ratio * (second.pose.x - first.pose.x);
    result.pose.y = first.pose.y + ratio * (second.pose.y - first.pose.y);
    result.pose.yaw = normalizeAngle(
        first.pose.yaw + ratio * normalizeAngle(second.pose.yaw - first.pose.yaw));
    result.type = ratio < 0.5 ? first.type : second.type;
    return result;
}

bool CorridorBuilder::segmentLegal(const Pose2d& from, const Pose2d& to) const {
    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(
        distance / std::max(0.005, config_.expansion_step * 0.5))));
    for (int i = 0; i <= steps; ++i) {
        const double ratio = static_cast<double>(i) / steps;
        Pose2d point;
        point.x = from.x + ratio * (to.x - from.x);
        point.y = from.y + ratio * (to.y - from.y);
        if (!pointLegal(point)) return false;
    }
    return true;
}

bool CorridorBuilder::appendConnected(const Pose2d& seed, int depth,
                                      std::vector<Pose2d>& seeds,
                                      std::vector<Aabb>& boxes,
                                      std::string& reason) const {
    Aabb box;
    if (!generateAabb(seed, box, reason)) return false;
    if (boxes.empty() || connected(boxes.back(), box)) {
        seeds.push_back(seed);
        boxes.push_back(box);
        return true;
    }
    if (depth >= config_.max_bridge_depth || !segmentLegal(seeds.back(), seed)) {
        std::ostringstream out;
        out << "AABB connection failed between seeds (" << seeds.back().x << ", "
            << seeds.back().y << ") and (" << seed.x << ", " << seed.y << ")";
        reason = out.str();
        return false;
    }
    const Pose2d from = seeds.back();
    Pose2d middle;
    middle.x = (from.x + seed.x) * 0.5;
    middle.y = (from.y + seed.y) * 0.5;
    middle.yaw = normalizeAngle(from.yaw + 0.5 * normalizeAngle(seed.yaw - from.yaw));
    if (!appendConnected(middle, depth + 1, seeds, boxes, reason)) return false;
    return appendConnected(seed, depth + 1, seeds, boxes, reason);
}

void CorridorBuilder::mergeRedundant(std::vector<Pose2d>& seeds,
                                     std::vector<Aabb>& boxes) const {
    if (boxes.size() < 2) return;
    size_t i = 1;
    while (i < boxes.size()) {
        if (contains(boxes[i - 1], boxes[i])) {
            boxes.erase(boxes.begin() + i);
            seeds.erase(seeds.begin() + i);
            continue;
        }
        if (contains(boxes[i], boxes[i - 1])) {
            boxes[i - 1] = boxes[i];
            boxes.erase(boxes.begin() + i);
            seeds.erase(seeds.begin() + i);
            continue;
        }
        const Aabb joined{std::min(boxes[i - 1].min_x, boxes[i].min_x),
                          std::max(boxes[i - 1].max_x, boxes[i].max_x),
                          std::min(boxes[i - 1].min_y, boxes[i].min_y),
                          std::max(boxes[i - 1].max_y, boxes[i].max_y)};
        if (joined.width() <= config_.max_width + kEpsilon &&
            joined.height() <= config_.max_height + kEpsilon && aabbLegal(joined)) {
            boxes[i - 1] = joined;
            boxes.erase(boxes.begin() + i);
            seeds.erase(seeds.begin() + i);
            continue;
        }
        ++i;
    }
}

CorridorResult CorridorBuilder::build(const Pose2d& start,
                                      const std::vector<PathPoint>& path,
                                      double current_s, int path_gen) const {
    CorridorResult result;
    if (path.size() < 2 || path.back().s <= kEpsilon) {
        result.reason = "offline path has fewer than two distinct points";
        return result;
    }
    const Pose2d start_center = bodyCenter(start);
    if (!pointLegal(start_center)) {
        std::ostringstream out;
        out << "actual rear-axle start (" << start.x << ", " << start.y
            << ") places body center (" << start_center.x << ", "
            << start_center.y << ") outside the configured single-disc free space";
        result.reason = out.str();
        return result;
    }
    const double clamped_s = std::clamp(current_s, 0.0, path.back().s);
    const double target_s = std::min(path.back().s,
        clamped_s + std::max(0.0, config_.rejoin_distance));
    if (target_s <= clamped_s + kEpsilon) {
        result.reason = "no forward path remains for a safe rejoin point";
        return result;
    }

    std::string reason;
    if (!appendConnected(start_center, 0, result.seeds, result.boxes, reason)) {
        result.reason = reason;
        return result;
    }
    const auto appendPathSeed = [&](const Pose2d& original_seed) {
        if (pointLegal(original_seed)) {
            return appendConnected(original_seed, 0, result.seeds, result.boxes, reason);
        }
        const size_t seed_count = result.seeds.size();
        const size_t box_count = result.boxes.size();
        const int max_steps = config_.seed_search_step > 0.0 &&
                              std::isfinite(config_.seed_search_step) &&
                              std::isfinite(config_.seed_search_max_offset) ?
            static_cast<int>(std::clamp(config_.seed_search_max_offset /
                                       config_.seed_search_step, 0.0, 100.0)) : 0;
        for (int step = 1; step <= max_steps; ++step) {
            const double offset = step * config_.seed_search_step;
            for (int direction = 0; direction < 4; ++direction) {
                Pose2d candidate = original_seed;
                if (direction == 0) candidate.x += offset;
                if (direction == 1) candidate.x -= offset;
                if (direction == 2) candidate.y += offset;
                if (direction == 3) candidate.y -= offset;
                if (!pointLegal(candidate)) continue;
                Aabb box;
                if (!generateAabb(candidate, box, reason)) continue;
                if (appendConnected(candidate, 0, result.seeds, result.boxes, reason)) {
                    return true;
                }
                result.seeds.resize(seed_count);
                result.boxes.resize(box_count);
            }
        }
        reason = "no connected legal AABB seed within local search range";
        return false;
    };
    const PathPoint current = interpolate(path, clamped_s);
    const Pose2d current_center = bodyCenter(current.pose);
    if (!appendPathSeed(current_center)) {
        result.reason = "cannot connect actual start to current path: " + reason;
        return result;
    }

    double sample_s = clamped_s;
    PathPoint terminal = current;
    bool terminal_valid = pointLegal(current_center);
    size_t terminal_seed_count = result.seeds.size();
    size_t terminal_box_count = result.boxes.size();
    bool shortened = false;
    std::string stop_reason;
    while (sample_s + kEpsilon < target_s) {
        sample_s = std::min(target_s,
            sample_s + std::max(0.01, config_.path_sample_step));
        const PathPoint point = interpolate(path, sample_s);
        const Pose2d original_seed = bodyCenter(point.pose);
        const bool original_legal = pointLegal(original_seed);
        const size_t seed_count = result.seeds.size();
        const size_t box_count = result.boxes.size();
        if (!appendPathSeed(original_seed)) {
            result.seeds.resize(seed_count);
            result.boxes.resize(box_count);
            if (!terminal_valid || terminal.s - clamped_s + kEpsilon <
                                       config_.min_rejoin_distance) {
                result.reason = "path corridor stopped before minimum rejoin distance at s=" +
                                std::to_string(sample_s) + ": " + reason;
                return result;
            }
            shortened = true;
            stop_reason = reason;
            break;
        }
        if (original_legal) {
            terminal = point;
            terminal_valid = true;
            terminal_seed_count = result.seeds.size();
            terminal_box_count = result.boxes.size();
        }
    }
    if (!terminal_valid || (terminal.s + kEpsilon < target_s &&
        terminal.s - clamped_s + kEpsilon < config_.min_rejoin_distance)) {
        result.reason = "no legal original-path rejoin point within the allowed range";
        return result;
    }
    if (result.seeds.size() != terminal_seed_count) {
        result.seeds.resize(terminal_seed_count);
        result.boxes.resize(terminal_box_count);
        shortened = true;
        stop_reason = "trailing original-path samples are not legal rejoin points";
    }
    mergeRedundant(result.seeds, result.boxes);
    for (size_t i = 1; i < result.boxes.size(); ++i) {
        if (!connected(result.boxes[i - 1], result.boxes[i])) {
            result.reason = "internal error: merged AABB sequence is disconnected";
            result.boxes.clear();
            result.seeds.clear();
            return result;
        }
    }
    const Pose2d terminal_center = bodyCenter(terminal.pose);
    const Aabb& final_box = result.boxes.back();
    if (!pointLegal(terminal_center) || terminal_center.x < final_box.min_x - kEpsilon ||
        terminal_center.x > final_box.max_x + kEpsilon ||
        terminal_center.y < final_box.min_y - kEpsilon ||
        terminal_center.y > final_box.max_y + kEpsilon) {
        result.reason = "original-path rejoin point is outside the final legal AABB";
        return result;
    }
    result.terminal = {terminal.pose, terminal.s, path_gen, terminal.type};
    result.success = true;
    result.reason = shortened ? "ok; rejoin shortened before obstacle: " + stop_reason : "ok";
    return result;
}

}  // namespace forklift_nlp_recovery
