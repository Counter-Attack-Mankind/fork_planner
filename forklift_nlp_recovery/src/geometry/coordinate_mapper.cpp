#include "forklift_nlp_recovery/coordinate_mapper.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool contains(const Aabb& box, const Pose2d& point) {
    return point.x >= box.min_x - kEpsilon && point.x <= box.max_x + kEpsilon &&
           point.y >= box.min_y - kEpsilon && point.y <= box.max_y + kEpsilon;
}

double signedPointToAabbDistance(const Pose2d& point, const Aabb& box) {
    const double dx = std::max({box.min_x - point.x, 0.0, point.x - box.max_x});
    const double dy = std::max({box.min_y - point.y, 0.0, point.y - box.max_y});
    if (dx > 0.0 || dy > 0.0) return std::hypot(dx, dy);
    return -std::min({point.x - box.min_x, box.max_x - point.x,
                      point.y - box.min_y, box.max_y - point.y});
}

struct Axis { double x; double y; };

double projectionRadius(double half_length, double half_width,
                        const Axis& longitudinal, const Axis& lateral,
                        const Axis& axis) {
    return half_length * std::abs(longitudinal.x * axis.x + longitudinal.y * axis.y) +
           half_width * std::abs(lateral.x * axis.x + lateral.y * axis.y);
}

bool obbOverlapsAabb(const Pose2d& center, double length, double width,
                     const Aabb& box) {
    const Axis longitudinal{std::cos(center.yaw), std::sin(center.yaw)};
    const Axis lateral{-longitudinal.y, longitudinal.x};
    const Axis axes[4] = {longitudinal, lateral, {1.0, 0.0}, {0.0, 1.0}};
    const double box_cx = 0.5 * (box.min_x + box.max_x);
    const double box_cy = 0.5 * (box.min_y + box.max_y);
    const double dx = box_cx - center.x;
    const double dy = box_cy - center.y;
    for (const Axis& axis : axes) {
        const double center_distance = std::abs(dx * axis.x + dy * axis.y);
        const double body_radius = projectionRadius(
            0.5 * length, 0.5 * width, longitudinal, lateral, axis);
        const double box_radius = 0.5 * (box.max_x - box.min_x) * std::abs(axis.x) +
                                  0.5 * (box.max_y - box.min_y) * std::abs(axis.y);
        if (center_distance > body_radius + box_radius + kEpsilon) return false;
    }
    return true;
}

bool obbInsideBoundary(const Pose2d& center, const MapParam& map_param) {
    const double c = std::cos(center.yaw);
    const double s = std::sin(center.yaw);
    const double half_length = 0.5 * map_param.vehicle_length;
    const double half_width = 0.5 * map_param.vehicle_width;
    for (const double longitudinal : {-half_length, half_length}) {
        for (const double lateral : {-half_width, half_width}) {
            const double x = center.x + longitudinal * c - lateral * s;
            const double y = center.y + longitudinal * s + lateral * c;
            if (x < -kEpsilon || x > map_param.field_width + kEpsilon ||
                y < -kEpsilon || y > map_param.field_height + kEpsilon) return false;
        }
    }
    return true;
}

}  // namespace

CoordinateMapper::CoordinateMapper(const MapParam& map_param,
                                   const CorridorBuilder& corridor_builder)
    : map_param_(map_param), corridor_builder_(corridor_builder) {}

MappedPathPoint CoordinateMapper::mapPoint(const PathPoint& point,
                                           size_t source_index) const {
    MappedPathPoint mapped;
    mapped.source_index = source_index;
    mapped.source_s = point.s;
    mapped.path_type = point.type;
    mapped.rear_axle_pose = point.pose;
    mapped.body_center_pose = corridor_builder_.bodyCenter(point.pose);
    return mapped;
}

std::vector<MappedPathPoint> CoordinateMapper::mapPath(
    const std::vector<PathPoint>& rear_axle_path) const {
    std::vector<MappedPathPoint> mapped;
    mapped.reserve(rear_axle_path.size());
    for (size_t i = 0; i < rear_axle_path.size(); ++i) {
        mapped.push_back(mapPoint(rear_axle_path[i], i));
    }
    return mapped;
}

GeometryDiagnostic CoordinateMapper::diagnosePose(
    size_t source_index, double source_s, const Pose2d& rear_axle_pose,
    const std::vector<Aabb>& corridor) const {
    GeometryDiagnostic result;
    result.source_index = source_index;
    result.source_s = source_s;
    result.rear_axle_pose = rear_axle_pose;
    result.body_center_pose = corridor_builder_.bodyCenter(rear_axle_pose);
    for (size_t i = 0; i < corridor.size(); ++i) {
        if (contains(corridor[i], result.body_center_pose)) {
            result.containing_aabbs.push_back(i);
        }
    }

    result.minimum_inflated_shelf_clearance = std::numeric_limits<double>::infinity();
    for (const Aabb& shelf : corridor_builder_.originalShelves()) {
        const double clearance = signedPointToAabbDistance(
            result.body_center_pose, shelf) - corridor_builder_.shelfInflation();
        result.minimum_inflated_shelf_clearance = std::min(
            result.minimum_inflated_shelf_clearance, clearance);
        result.body_obb_shelf_collision = result.body_obb_shelf_collision ||
            obbOverlapsAabb(result.body_center_pose, map_param_.vehicle_length,
                            map_param_.vehicle_width, shelf);
    }
    result.boundary_clearance = std::min({
        result.body_center_pose.x - corridor_builder_.boundaryClearance(),
        map_param_.field_width - corridor_builder_.boundaryClearance() -
            result.body_center_pose.x,
        result.body_center_pose.y - corridor_builder_.boundaryClearance(),
        map_param_.field_height - corridor_builder_.boundaryClearance() -
            result.body_center_pose.y});
    result.disc_shelf_collision = result.minimum_inflated_shelf_clearance < -kEpsilon;
    result.disc_boundary_collision = result.boundary_clearance < -kEpsilon;
    result.body_obb_boundary_collision = !obbInsideBoundary(
        result.body_center_pose, map_param_);
    MappedPathPoint mapped{source_index, source_s, 0, rear_axle_pose,
                           result.body_center_pose};
    std::string unused;
    result.mapping_consistent = checkMappingConsistency(mapped, 1e-10, unused);
    return result;
}

GeometryDiagnostic CoordinateMapper::diagnosePoint(
    const MappedPathPoint& point, const std::vector<Aabb>& corridor) const {
    GeometryDiagnostic result = diagnosePose(
        point.source_index, point.source_s, point.rear_axle_pose, corridor);
    std::string unused;
    result.mapping_consistent = checkMappingConsistency(point, 1e-10, unused);
    return result;
}

std::vector<GeometryDiagnostic> CoordinateMapper::diagnosePath(
    const std::vector<MappedPathPoint>& path,
    const std::vector<Aabb>& corridor) const {
    std::vector<GeometryDiagnostic> diagnostics;
    diagnostics.reserve(path.size());
    for (const MappedPathPoint& point : path) {
        diagnostics.push_back(diagnosePoint(point, corridor));
    }
    return diagnostics;
}

std::vector<SweptGeometryDiagnostic> CoordinateMapper::diagnoseSweptPath(
    const std::vector<PathPoint>& rear_axle_path,
    const std::vector<Aabb>& corridor, double sample_step) const {
    std::vector<SweptGeometryDiagnostic> diagnostics;
    if (rear_axle_path.size() < 2) return diagnostics;
    const double step = std::max(0.001, sample_step);
    for (size_t i = 0; i + 1 < rear_axle_path.size(); ++i) {
        const PathPoint& first = rear_axle_path[i];
        const PathPoint& second = rear_axle_path[i + 1];
        const double distance = std::hypot(second.pose.x - first.pose.x,
                                           second.pose.y - first.pose.y);
        const double yaw_delta = normalizeAngle(second.pose.yaw - first.pose.yaw);
        const double center_motion_bound = distance +
            std::abs(yaw_delta) * std::abs(map_param_.rear_axle_to_center);
        const int samples = std::max(1, static_cast<int>(std::ceil(
            center_motion_bound / step)));
        for (int sample = 1; sample < samples; ++sample) {
            const double ratio = static_cast<double>(sample) / samples;
            Pose2d rear;
            rear.x = first.pose.x + ratio * (second.pose.x - first.pose.x);
            rear.y = first.pose.y + ratio * (second.pose.y - first.pose.y);
            rear.yaw = normalizeAngle(first.pose.yaw + ratio * yaw_delta);
            const double source_s = first.s + ratio * (second.s - first.s);
            SweptGeometryDiagnostic diagnostic;
            diagnostic.segment_begin_index = i;
            diagnostic.segment_end_index = i + 1;
            diagnostic.segment_ratio = ratio;
            diagnostic.sample = diagnosePose(i, source_s, rear, corridor);
            diagnostics.push_back(std::move(diagnostic));
        }
    }
    return diagnostics;
}

bool CoordinateMapper::checkMappingConsistency(
    const MappedPathPoint& point, double tolerance, std::string& reason) const {
    const Pose2d expected = corridor_builder_.bodyCenter(point.rear_axle_pose);
    const double position_error = std::hypot(
        expected.x - point.body_center_pose.x,
        expected.y - point.body_center_pose.y);
    const double yaw_error = std::abs(normalizeAngle(
        expected.yaw - point.body_center_pose.yaw));
    if (position_error <= tolerance && yaw_error <= tolerance) {
        reason.clear();
        return true;
    }
    std::ostringstream out;
    out << "body-center mapping mismatch at source node " << point.source_index
        << ": position_error=" << position_error << " yaw_error=" << yaw_error;
    reason = out.str();
    return false;
}

}  // namespace forklift_nlp_recovery
