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
    Pose2d pose;
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

struct CorridorConfig {
    double expansion_step = 0.02;
    double path_sample_step = 0.05;
    double min_rejoin_distance = 0.20;
    double rejoin_distance = 0.80;
    double max_width = 2.50;
    double max_height = 2.00;
    double safety_margin = 0.01;
    int max_bridge_depth = 8;
};

struct RejoinPoint {
    Pose2d pose;
    double path_s = 0.0;
    int path_gen = 0;
    uint8_t type = 0;
};

struct CorridorResult {
    bool success = false;
    std::string reason;
    std::vector<Aabb> boxes;
    std::vector<Pose2d> seeds;
    RejoinPoint terminal;
};

class CorridorBuilder {
public:
    CorridorBuilder(const MapParam& map_param,
                    const std::vector<ShelfBlock>& shelves,
                    const CorridorConfig& config);

    double vehicleRadius() const { return vehicle_radius_; }
    double clearance() const { return clearance_; }
    const std::vector<Aabb>& inflatedShelves() const { return inflated_shelves_; }

    bool pointLegal(const Pose2d& point) const;
    bool aabbLegal(const Aabb& box) const;
    bool connected(const Aabb& first, const Aabb& second) const;
    bool generateAabb(const Pose2d& seed, Aabb& box, std::string& reason) const;
    CorridorResult build(const Pose2d& start, const std::vector<PathPoint>& path,
                         double current_s, int path_gen) const;

    static std::vector<PathPoint> parameterizePath(
        const std::vector<Pose2d>& poses, const std::vector<uint8_t>& types);

private:
    PathPoint interpolate(const std::vector<PathPoint>& path, double s) const;
    bool segmentLegal(const Pose2d& from, const Pose2d& to) const;
    bool appendConnected(const Pose2d& seed, int depth,
                         std::vector<Pose2d>& seeds,
                         std::vector<Aabb>& boxes,
                         std::string& reason) const;
    void mergeRedundant(std::vector<Pose2d>& seeds,
                        std::vector<Aabb>& boxes) const;

    MapParam map_param_;
    CorridorConfig config_;
    double vehicle_radius_ = 0.0;
    double clearance_ = 0.0;
    std::vector<Aabb> inflated_shelves_;
};

}  // namespace forklift_nlp_recovery
