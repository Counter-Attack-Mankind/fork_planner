#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"

namespace forklift_nlp_recovery {

struct Pose2d {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct PathPoint {
    Pose2d pose;  // rear-axle pose
    double s = 0.0;
    uint8_t type = 0;
};

struct Aabb {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;

    double width() const { return max_x - min_x; }
    double height() const { return max_y - min_y; }
};

struct PointLegalityViolation {
    std::string type;
    int shelf_index = -1;
    double distance = 0.0;
};

struct CorridorConfig {
    double boundary_inset_reduction = 0.0;
    double vehicle_radius = 0.0;
    double shelf_inflation_reduction = 0.0;
    double expansion_step = 0.02;
    double seed_search_step = 0.01;
    double seed_search_max_offset = 0.10;
    double rejoin_distance = 0.80;
    double max_width = 2.50;
    double max_height = 2.00;
    double safety_margin = 0.0;
};

struct RejoinPoint {
    Pose2d pose;  // rear-axle pose on the original path
    double path_s = 0.0;
    int path_gen = 0;
    uint8_t type = 0;
};

struct CorridorResult {
    bool success = false;
    std::string reason;
    std::vector<Aabb> boxes;
    std::vector<Pose2d> seeds;  // body-center seeds used by AABB geometry
    RejoinPoint terminal;
};

class CorridorBuilder {
public:
    CorridorBuilder(const MapParam& map_param,
                    const std::vector<ShelfBlock>& shelves,
                    const CorridorConfig& config);

    double vehicleRadius() const { return vehicle_radius_; }
    double boundaryClearance() const { return boundary_clearance_; }
    double shelfInflation() const { return shelf_inflation_; }
    const std::vector<Aabb>& originalShelves() const {return original_shelves_;}

    Pose2d bodyCenter(const Pose2d& rear_axle_pose) const;
    // The point/seed arguments below are body-center coordinates.
    bool pointLegal(const Pose2d& point) const;
    bool diagnosePointViolation(const Pose2d& point,
                                PointLegalityViolation& violation) const;
    bool aabbLegal(const Aabb& box) const;
    bool generateAabb(const Pose2d& seed, Aabb& box, std::string& reason) const;
    bool findLegalSeed(const Pose2d& requested, Pose2d& legal,
                       std::string& reason) const;
    // start and path are rear-axle poses; only internal geometry is body-centered.
    CorridorResult build(const Pose2d& start, const std::vector<PathPoint>& path,
                         double current_s, int path_gen) const;

    static std::vector<PathPoint> parameterizePath(
        const std::vector<Pose2d>& poses, const std::vector<uint8_t>& types);

private:
    PathPoint interpolate(const std::vector<PathPoint>& path, double s) const;

    MapParam map_param_;
    CorridorConfig config_;
    double vehicle_radius_ = 0.0;
    double boundary_clearance_ = 0.0;
    double shelf_inflation_ = 0.0;
    std::vector<Aabb> original_shelves_;
};

}  // namespace forklift_nlp_recovery
