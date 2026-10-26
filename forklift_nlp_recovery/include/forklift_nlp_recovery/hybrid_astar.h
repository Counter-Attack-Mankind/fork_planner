#pragma once

#include <string>
#include <vector>

#include "forklift_nlp_recovery/optimization_input.h"

namespace forklift_nlp_recovery {

struct HybridAstarConfig {
    double integration_step = 0.04;
    double collision_check_step = 0.01;
    double xy_resolution = 0.04;
    int yaw_bins = 72;
    double goal_position_tolerance = 0.04;
    double goal_yaw_tolerance = 0.12;
    double reverse_cost = 1.15;
    double direction_switch_cost = 0.20;
    int max_expansions = 120000;
};

struct HybridAstarResult {
    bool success = false;
    std::string reason;
    int expansions = 0;
    std::vector<PathPoint> path;
};

class HybridAstarPlanner {
public:
    explicit HybridAstarPlanner(const HybridAstarConfig& config);

    HybridAstarResult plan(const NlpVehicleInput& vehicle,
                           const std::vector<StaticVehicleObstacle>& obstacles) const;

private:
    HybridAstarConfig config_;
};

}  // namespace forklift_nlp_recovery
