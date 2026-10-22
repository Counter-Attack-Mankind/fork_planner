#include "forklift_nlp_recovery/corridor_builder.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

double normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

}  // namespace

CorridorBuilder::CorridorBuilder(const MapParam& map_param,
                                 const std::vector<ShelfBlock>& shelves,
                                 const CorridorConfig& config)
    : map_param_(map_param), config_(config) {
    const double geometric_radius = std::hypot(map_param_.vehicle_length * 0.5,
                                               map_param_.vehicle_width * 0.5);
    vehicle_radius_ = config_.vehicle_radius > 0.0 ? config_.vehicle_radius : geometric_radius;
    boundary_clearance_ = std::max(0.0, vehicle_radius_ + config_.safety_margin - config_.boundary_inset_reduction);
    shelf_inflation_ = std::max(0.0, vehicle_radius_ + config_.safety_margin - config_.shelf_inflation_reduction);
    original_shelves_.reserve(shelves.size());
    for (const ShelfBlock& shelf : shelves) {
        original_shelves_.push_back({shelf.x, shelf.x_max(), shelf.y, shelf.y_max()});
    }
}

Pose2d CorridorBuilder::bodyCenter(const Pose2d& rear_axle_pose) const {
    Pose2d center = rear_axle_pose;
    center.x += map_param_.rear_axle_to_center * std::cos(rear_axle_pose.yaw);
    center.y += map_param_.rear_axle_to_center * std::sin(rear_axle_pose.yaw);
    return center;
}

bool CorridorBuilder::pointLegal(const Pose2d& point) const {
    return aabbLegal({point.x, point.x, point.y, point.y});
}

bool CorridorBuilder::aabbLegal(const Aabb& box) const {
    if (box.min_x < boundary_clearance_ - kEpsilon ||
        box.max_x > map_param_.field_width - boundary_clearance_ + kEpsilon ||
        box.min_y < boundary_clearance_ - kEpsilon ||
        box.max_y > map_param_.field_height - boundary_clearance_ + kEpsilon) return false;
    for (const Aabb& shelf : original_shelves_) {
        const double dx = std::max({0.0, shelf.min_x - box.max_x, box.min_x - shelf.max_x});
        const double dy = std::max({0.0, shelf.min_y - box.max_y, box.min_y - shelf.max_y});
        const bool interior_overlap = box.max_x > shelf.min_x + kEpsilon &&
            box.min_x < shelf.max_x - kEpsilon && box.max_y > shelf.min_y + kEpsilon &&
            box.min_y < shelf.max_y - kEpsilon;
        if (interior_overlap || std::hypot(dx, dy) < shelf_inflation_ - kEpsilon) return false;
    }
    return true;
}

bool CorridorBuilder::generateAabb(const Pose2d& seed, Aabb& box, std::string& reason) const {
    if (!pointLegal(seed)) {
        std::ostringstream out;
        out << "seed (" << seed.x << ", " << seed.y
            << ") is inside an inflated shelf or outside the inset map boundary";
        reason = out.str();
        return false;
    }
    box = {seed.x, seed.x, seed.y, seed.y};
    bool open[4] = {true, true, true, true};
    while (open[0] || open[1] || open[2] || open[3]) {
        for (int direction = 0; direction < 4; ++direction) {
            if (!open[direction]) continue;
            Aabb candidate = box;
            const double limit = direction < 2 ? config_.max_height : config_.max_width;
            const double size = direction < 2 ? box.height() : box.width();
            const double room = limit - size;
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

bool CorridorBuilder::findLegalSeed(const Pose2d& requested, Pose2d& legal,
                                    std::string& reason) const {
    if (pointLegal(requested)) {
        legal = requested;
        reason.clear();
        return true;
    }
    if (!(config_.seed_search_step > 0.0) || config_.seed_search_max_offset < 0.0) {
        reason = "seed search parameters are invalid";
        return false;
    }
    for (double offset = config_.seed_search_step;
         offset <= config_.seed_search_max_offset + kEpsilon; offset += config_.seed_search_step) {
        const Pose2d candidates[] = {
            {requested.x + offset, requested.y, requested.yaw},
            {requested.x - offset, requested.y, requested.yaw},
            {requested.x, requested.y + offset, requested.yaw},
            {requested.x, requested.y - offset, requested.yaw}};
        for (const Pose2d& candidate : candidates) {
            if (pointLegal(candidate)) {
                legal = candidate;
                reason.clear();
                return true;
            }
        }
    }
    std::ostringstream out;
    out << "no legal cardinal seed within " << config_.seed_search_max_offset
        << " m of (" << requested.x << ", " << requested.y << ")";
    reason = out.str();
    return false;
}

std::vector<PathPoint> CorridorBuilder::parameterizePath(
    const std::vector<Pose2d>& poses, const std::vector<uint8_t>& types) {
    std::vector<PathPoint> path;
    path.reserve(poses.size());
    double s = 0.0;
    for (size_t i = 0; i < poses.size(); ++i) {
        if (i > 0) s += std::hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
        path.push_back({poses[i], s, i < types.size() ? types[i] : uint8_t{0}});
    }
    return path;
}

PathPoint CorridorBuilder::interpolate(const std::vector<PathPoint>& path, double s) const {
    if (s <= path.front().s) return path.front();
    if (s >= path.back().s) return path.back();
    const auto upper = std::upper_bound(path.begin(), path.end(), s,
        [](double value, const PathPoint& point) { return value < point.s; });
    const PathPoint& second = *upper;
    const PathPoint& first = *(upper - 1);
    const double span = second.s - first.s;
    const double ratio = span > kEpsilon ? (s - first.s) / span : 0.0;
    PathPoint result;
    result.s = s;
    result.pose.x = first.pose.x + ratio * (second.pose.x - first.pose.x);
    result.pose.y = first.pose.y + ratio * (second.pose.y - first.pose.y);
    result.pose.yaw = normalizeAngle(first.pose.yaw + ratio * normalizeAngle(second.pose.yaw - first.pose.yaw));
    result.type = ratio < 0.5 ? first.type : second.type;
    return result;
}

CorridorResult CorridorBuilder::build(const Pose2d& start, const std::vector<PathPoint>& path,
                                      double current_s, int path_gen) const {
    (void)start;
    CorridorResult result;
    if (path.size() < 2) {
        result.reason = "path has fewer than two points";
        return result;
    }
    if (!std::isfinite(current_s) || current_s < path.front().s - kEpsilon ||
        current_s > path.back().s + kEpsilon) {
        result.reason = "current path_s is outside the frozen path";
        return result;
    }
    const double terminal_s = std::min(current_s + config_.rejoin_distance, path.back().s);
    const PathPoint terminal = interpolate(path, terminal_s);
    result.terminal = {terminal.pose, terminal.s, path_gen, terminal.type};
    result.success = true;
    result.reason = "ok";
    return result;
}

}  // namespace forklift_nlp_recovery
