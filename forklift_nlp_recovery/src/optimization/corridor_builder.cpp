#include "forklift_nlp_recovery/corridor_builder.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kMapXExtension = 0.01;  // 左右边界各外扩 1 cm

double normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::string vehicleLabel(int vehicle_id) {
    return vehicle_id >= 0 ? "V" + std::to_string(vehicle_id) : "V?";
}

std::string aabbText(const Aabb& box) {
    std::ostringstream out;
    out << "[" << box.min_x << "," << box.max_x << "]x["
        << box.min_y << "," << box.max_y << "]";
    return out.str();
}

}  // namespace

CorridorBuilder::CorridorBuilder(const MapParam& map_param,
                                 const std::vector<ShelfBlock>& shelves,
                                 const CorridorConfig& config)
    : map_param_(map_param), config_(config) {
    vehicle_radius_ = std::hypot(map_param_.vehicle_length * 0.5,
                                 map_param_.vehicle_width * 0.5);
    boundary_clearance_ = std::max(0.0, vehicle_radius_ - config_.stc_inset_reduction);
    shelf_inflation_ = boundary_clearance_;
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

bool CorridorBuilder::diagnosePointViolation(
    const Pose2d& point, PointLegalityViolation& violation) const {
    violation = PointLegalityViolation{};
    const double min_x = -kMapXExtension + boundary_clearance_;
    const double max_x = map_param_.field_width + kMapXExtension - boundary_clearance_;
    const double min_y = boundary_clearance_;
    const double max_y = map_param_.field_height - boundary_clearance_;
    const double outside_x = std::max({min_x - point.x, point.x - max_x, 0.0});
    const double outside_y = std::max({min_y - point.y, point.y - max_y, 0.0});
    if (outside_x > kEpsilon || outside_y > kEpsilon) {
        violation.type = "map_boundary";
        violation.distance = std::hypot(outside_x, outside_y);
        return true;
    }
    for (size_t index = 0; index < original_shelves_.size(); ++index) {
        const Aabb& shelf = original_shelves_[index];
        const double dx = std::max({0.0, shelf.min_x - point.x, point.x - shelf.max_x});
        const double dy = std::max({0.0, shelf.min_y - point.y, point.y - shelf.max_y});
        const bool inside = point.x > shelf.min_x + kEpsilon &&
            point.x < shelf.max_x - kEpsilon && point.y > shelf.min_y + kEpsilon &&
            point.y < shelf.max_y - kEpsilon;
        const double separation = std::hypot(dx, dy);
        if (!inside && separation >= shelf_inflation_ - kEpsilon) continue;
        violation.type = "inflated_shelf_boundary";
        violation.shelf_index = static_cast<int>(index);
        if (inside) {
            violation.distance = shelf_inflation_ + std::min({
                point.x - shelf.min_x, shelf.max_x - point.x,
                point.y - shelf.min_y, shelf.max_y - point.y});
        } else {
            violation.distance = std::max(0.0, shelf_inflation_ - separation);
        }
        return true;
    }
    return false;
}

bool CorridorBuilder::aabbLegal(const Aabb& box) const {
    if (box.min_x < -kMapXExtension + boundary_clearance_ - kEpsilon ||
        box.max_x > map_param_.field_width + kMapXExtension - boundary_clearance_ + kEpsilon ||
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

bool CorridorBuilder::rawAabbLegal(const Aabb& box) const {
    if (box.min_x < -kMapXExtension - kEpsilon ||  box.max_x > map_param_.field_width + kMapXExtension + kEpsilon ||
        box.min_y < -kEpsilon || box.max_y > map_param_.field_height + kEpsilon) return false;
    for (const Aabb& shelf : original_shelves_) {
        const bool overlap = box.max_x > shelf.min_x + kEpsilon &&
            box.min_x < shelf.max_x - kEpsilon &&
            box.max_y > shelf.min_y + kEpsilon &&
            box.min_y < shelf.max_y - kEpsilon;
        if (overlap) return false;
    }
    return true;
}

bool CorridorBuilder::expandRawAabb(const Pose2d& seed, Aabb& raw,
                                    std::string& reason,
                                    bool horizontal_first) const {
    raw = {seed.x, seed.x, seed.y, seed.y};
    if (!rawAabbLegal(raw)) {
        reason = "raw seed point intersects a shelf or lies outside the map";
        return false;
    }
    if (horizontal_first) {
        bool horizontal_open[2] = {true, true};
        while (horizontal_open[0] || horizontal_open[1]) {
            for (int direction = 0; direction < 2; ++direction) {
                if (!horizontal_open[direction]) continue;
                Aabb candidate = raw;
                if (direction == 0) candidate.min_x -= config_.expansion_step;
                else candidate.max_x += config_.expansion_step;
                if (rawAabbLegal(candidate)) raw = candidate;
                else horizontal_open[direction] = false;
            }
        }
        bool vertical_open[2] = {true, true};
        while (vertical_open[0] || vertical_open[1]) {
            for (int direction = 0; direction < 2; ++direction) {
                if (!vertical_open[direction]) continue;
                Aabb candidate = raw;
                if (direction == 0) candidate.max_y += config_.expansion_step;
                else candidate.min_y -= config_.expansion_step;
                if (rawAabbLegal(candidate)) raw = candidate;
                else vertical_open[direction] = false;
            }
        }
    } else {
        bool open[4] = {true, true, true, true};
        while (open[0] || open[1] || open[2] || open[3]) {
            for (int direction = 0; direction < 4; ++direction) {
                if (!open[direction]) continue;
                Aabb candidate = raw;
                if (direction == 0) candidate.max_y += config_.expansion_step;
                if (direction == 1) candidate.min_y -= config_.expansion_step;
                if (direction == 2) candidate.min_x -= config_.expansion_step;
                if (direction == 3) candidate.max_x += config_.expansion_step;
                if (rawAabbLegal(candidate)) raw = candidate;
                else open[direction] = false;
            }
        }
    }
    reason.clear();
    return true;
}

bool CorridorBuilder::generateAabb(const Pose2d& seed, Aabb& box, std::string& reason) const {
    if (!(config_.expansion_step > 0.0)) {
        reason = "expansion_step must be positive";
        return false;
    }
    Aabb raw;
    if (!expandRawAabb(seed, raw, reason)) return false;
    box = {raw.min_x + boundary_clearance_, raw.max_x - boundary_clearance_,
           raw.min_y + boundary_clearance_, raw.max_y - boundary_clearance_};
    if (box.min_x > box.max_x || box.min_y > box.max_y ||
        seed.x < box.min_x - kEpsilon || seed.x > box.max_x + kEpsilon ||
        seed.y < box.min_y - kEpsilon || seed.y > box.max_y + kEpsilon ||
        !aabbLegal(box)) {
        reason = "expanded free-space AABB cannot contain the fixed disc center after inset";
        return false;
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
                                      double current_s, int path_gen, int vehicle_id,
                                      bool start_on_path) const {
    CorridorResult result;
    const std::string vehicle = vehicleLabel(vehicle_id);
    const Pose2d center = bodyCenter(start);
    result.seeds.push_back(center);
    if (!(config_.expansion_step > 0.0)) {
        std::ostringstream reason;
        reason << "stage=raw_aabb_expansion vehicle=" << vehicle
               << " disc_center=(" << center.x << "," << center.y
               << ") expansion_step=" << config_.expansion_step;
        result.reason = reason.str();
        return result;
    }
    std::string expansion_reason;
    if (!expandRawAabb(center, result.raw_aabb, expansion_reason, !start_on_path)) {
        std::ostringstream reason;
        reason << "stage=raw_aabb_expansion vehicle=" << vehicle
               << " disc_center=(" << center.x << "," << center.y << ") "
               << expansion_reason;
        result.reason = reason.str();
        return result;
    }
    result.raw_aabb_valid = true;
    result.safe_stc = {result.raw_aabb.min_x + boundary_clearance_,
                       result.raw_aabb.max_x - boundary_clearance_,
                       result.raw_aabb.min_y + boundary_clearance_,
                       result.raw_aabb.max_y - boundary_clearance_};
    const Aabb& stc = result.safe_stc;
    if (stc.min_x > stc.max_x || stc.min_y > stc.max_y ||
        !aabbLegal(stc)) {
        std::ostringstream reason;
        reason << "stage=stc_inset vehicle=" << vehicle << " disc_center=("
               << center.x << "," << center.y << ") inset=" << boundary_clearance_
               << " raw_aabb=" << aabbText(result.raw_aabb)
               << " inset_aabb=" << aabbText(stc);
        result.reason = reason.str();
        return result;
    }
    result.stc_valid = true;
    result.boxes.push_back(stc);
    if (path.size() < 2) {
        result.reason = "stage=terminal_search vehicle=" + vehicle +
                        " path has fewer than two points safe_stc=" + aabbText(stc);
        return result;
    }
    if (!std::isfinite(current_s) || current_s < path.front().s - kEpsilon ||
        current_s > path.back().s + kEpsilon) {
        std::ostringstream reason;
        reason << "stage=terminal_search vehicle=" << vehicle
               << " current_s=" << current_s << " path_range=["
               << path.front().s << "," << path.back().s << "]"
               << " safe_stc=" << aabbText(stc);
        result.reason = reason.str();
        return result;
    }
    const auto pointInside = [&](const Pose2d& point) {
        return point.x >= stc.min_x - kEpsilon && point.x <= stc.max_x + kEpsilon &&
               point.y >= stc.min_y - kEpsilon && point.y <= stc.max_y + kEpsilon;
    };
    const auto inside = [&](double s) {
        return pointInside(bodyCenter(interpolate(path, s).pose));
    };
    double optimization_start_s = current_s;
    Pose2d optimization_start_pose = start;
    uint8_t optimization_start_type = interpolate(path, current_s).type;
    if (!pointInside(center)) {
        if (!start_on_path) {
            std::ostringstream reason;
            reason << "stage=start_search vehicle=" << vehicle << " disc_center=("
                   << center.x << "," << center.y << ") is outside safe_stc="
                   << aabbText(stc) << " for off-path optimization start";
            result.reason = reason.str();
            return result;
        }
        const double max_start_correction = std::max(0.0, config_.max_start_correction);
        const double search_end = std::min(path.back().s, current_s + max_start_correction);
        const double search_step = std::max(1e-4, std::min({0.001,
            config_.exit_search_step, map_param_.path_resolution}));
        double outside_s = current_s;
        double entry_s = -1.0;
        for (double s = current_s; s <= search_end + kEpsilon;
             s = std::min(search_end, s + search_step)) {
            if (inside(s)) {
                entry_s = s;
                break;
            }
            outside_s = s;
            if (s >= search_end - kEpsilon) break;
        }
        if (entry_s > current_s + kEpsilon && outside_s < entry_s - kEpsilon) {
            for (int iteration = 0; iteration < 24; ++iteration) {
                const double mid = 0.5 * (outside_s + entry_s);
                if (inside(mid)) entry_s = mid;
                else outside_s = mid;
            }
        }
        if (entry_s >= 0.0) {
            const PathPoint entry = interpolate(path, entry_s);
            if (std::hypot(entry.pose.x - start.x, entry.pose.y - start.y) <=
                max_start_correction + kEpsilon) {
                optimization_start_s = entry.s;
                optimization_start_pose = entry.pose;
                optimization_start_type = entry.type;
            } else {
                entry_s = -1.0;
            }
        }
        if (entry_s < 0.0) {
            std::ostringstream reason;
            reason << "stage=start_search vehicle=" << vehicle << " disc_center=("
                   << center.x << "," << center.y << ") is outside safe_stc="
                   << aabbText(stc) << " and no valid path entry exists within "
                   << max_start_correction << " m";
            result.reason = reason.str();
            return result;
        }
        result.optimization_start_corrected = true;
    }
    result.optimization_start = {optimization_start_pose, optimization_start_s,
                                 optimization_start_type};
    result.optimization_start_valid = true;
    const double step = std::max(1e-4, std::min(config_.exit_search_step,
                                                map_param_.path_resolution));
    double terminal_s = -1.0;
    if (!start_on_path) {
        const double path_end_s = path.back().s;
        if (inside(path_end_s)) {
            terminal_s = path_end_s;
        } else {
            double outside_s = path_end_s;
            for (double s = std::max(current_s, path_end_s - step);
                 s >= current_s - kEpsilon; s = std::max(current_s, s - step)) {
                if (inside(s)) {
                    double inside_s = s;
                    for (int iteration = 0; iteration < 24; ++iteration) {
                        const double mid = 0.5 * (inside_s + outside_s);
                        if (inside(mid)) inside_s = mid;
                        else outside_s = mid;
                    }
                    terminal_s = inside_s;
                    break;
                }
                outside_s = s;
                if (s <= current_s + kEpsilon) break;
            }
        }
        if (terminal_s < current_s - kEpsilon) {
            std::ostringstream reason;
            reason << "stage=terminal_search vehicle=" << vehicle
                   << " no body-center intersection with safe_stc=" << aabbText(stc)
                   << " in reverse search range=[" << current_s << ","
                   << path_end_s << "]";
            result.reason = reason.str();
            return result;
        }
    } else {
        double inside_s = optimization_start_s;
        double outside_s = -1.0;
        for (double s = std::min(path.back().s, optimization_start_s + step);
             s <= path.back().s + kEpsilon; s = std::min(path.back().s, s + step)) {
            if (!inside(s)) {
                outside_s = s;
                break;
            }
            inside_s = s;
            if (s >= path.back().s - kEpsilon) break;
        }
        if (outside_s < 0.0) {
            std::ostringstream reason;
            reason << "stage=terminal_search vehicle=" << vehicle
                   << " no body-center exit from safe_stc=" << aabbText(stc)
                   << " current_s=" << optimization_start_s
                   << " path_end_s=" << path.back().s;
            result.reason = reason.str();
            return result;
        }
        for (int iteration = 0; iteration < 24; ++iteration) {
            const double mid = 0.5 * (inside_s + outside_s);
            if (inside(mid)) inside_s = mid;
            else outside_s = mid;
        }
        terminal_s = inside_s;
    }
    const PathPoint terminal = interpolate(path, terminal_s);
    for (size_t i = 1; i < path.size(); ++i) {
        if (path[i - 1].type == path[i].type) continue;
        const double cusp_s = 0.5 * (path[i - 1].s + path[i].s);
        if (std::abs(cusp_s - terminal_s) < config_.cusp_clearance) {
            std::ostringstream reason;
            reason << "stage=terminal_search vehicle=" << vehicle
                   << " terminal_s=" << terminal_s << " cusp_s=" << cusp_s
                   << " cusp_distance=" << std::abs(cusp_s - terminal_s)
                   << " safe_stc=" << aabbText(stc);
            result.reason = reason.str();
            return result;
        }
    }
    result.terminal = {terminal.pose, terminal.s, path_gen, terminal.type};
    result.terminal_valid = true;
    result.success = true;
    result.reason = "ok";
    return result;
}

}  // namespace forklift_nlp_recovery
