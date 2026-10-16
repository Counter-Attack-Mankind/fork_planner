#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "forklift_nlp_recovery/corridor_builder.h"

namespace forklift_nlp_recovery {

struct MappedPathPoint {
    size_t source_index = 0;
    double source_s = 0.0;  // Rear-axle path progress; not body-center arc length.
    uint8_t path_type = 0;
    Pose2d rear_axle_pose;
    Pose2d body_center_pose;
};

struct GeometryDiagnostic {
    size_t source_index = 0;
    double source_s = 0.0;
    Pose2d rear_axle_pose;
    Pose2d body_center_pose;
    std::vector<size_t> containing_aabbs;
    // Signed clearance for the current single-disc shelf model. Negative means
    // the body-center point is inside a shelf's configured inflated region.
    double minimum_inflated_shelf_clearance = 0.0;
    double boundary_clearance = 0.0;
    bool disc_shelf_collision = false;
    bool disc_boundary_collision = false;
    // Exact SAT overlap of the uninflated vehicle OBB and real shelf rectangles.
    bool body_obb_shelf_collision = false;
    bool body_obb_boundary_collision = false;
    bool mapping_consistent = false;
};

struct SweptGeometryDiagnostic {
    size_t segment_begin_index = 0;
    size_t segment_end_index = 0;
    double segment_ratio = 0.0;
    GeometryDiagnostic sample;
};

class CoordinateMapper {
public:
    CoordinateMapper(const MapParam& map_param, const CorridorBuilder& corridor_builder);

    MappedPathPoint mapPoint(const PathPoint& point, size_t source_index) const;
    std::vector<MappedPathPoint> mapPath(const std::vector<PathPoint>& rear_axle_path) const;
    GeometryDiagnostic diagnosePoint(const MappedPathPoint& point,
                                     const std::vector<Aabb>& corridor) const;
    std::vector<GeometryDiagnostic> diagnosePath(
        const std::vector<MappedPathPoint>& path,
        const std::vector<Aabb>& corridor) const;

    // Numerically samples the rear-axle pose between adjacent source nodes.
    // This detects between-node issues but is not an analytic swept-volume distance.
    std::vector<SweptGeometryDiagnostic> diagnoseSweptPath(
        const std::vector<PathPoint>& rear_axle_path,
        const std::vector<Aabb>& corridor, double sample_step) const;

    bool checkMappingConsistency(const MappedPathPoint& point, double tolerance,
                                 std::string& reason) const;

private:
    GeometryDiagnostic diagnosePose(size_t source_index, double source_s,
                                    const Pose2d& rear_axle_pose,
                                    const std::vector<Aabb>& corridor) const;

    MapParam map_param_;
    const CorridorBuilder& corridor_builder_;
};

}  // namespace forklift_nlp_recovery
