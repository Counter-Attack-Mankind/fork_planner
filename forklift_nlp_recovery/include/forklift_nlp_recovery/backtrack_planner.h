#pragma once

#include <string>
#include <vector>

#include <sandbox_msgs/RecoverySnapshot.h>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_nlp_recovery/optimization_input.h"

namespace forklift_nlp_recovery {

struct BacktrackConfig {
    bool enabled = true;
    double distance = 0.50;
    double speed = 0.10;
    double path_validation_step = 0.01;
};

struct BacktrackCandidate {
    int vehicle_id = -1;
    int path_gen = -1;
    double current_s = 0.0;
    double target_s = 0.0;
    bool hold_at_source_b = false;
    std::string reason;
    OptimizationInputBuildResult preserved_build;
};

struct BacktrackPlan {
    bool executable = false;
    std::string reason;
    double speed = 0.0;
    std::vector<BacktrackCandidate> candidates;
};

class BacktrackPlanner {
public:
    BacktrackPlanner(const MapParam& map_param, const std::vector<Slot>& slots,
                     const BacktrackConfig& config);

    BacktrackPlan plan(
        const sandbox_msgs::RecoverySnapshot& snapshot,
        const std::vector<int>& deadlock_cycle_ids,
        const std::vector<OptimizationInputBuildResult>& preserved_builds) const;

private:
    const sandbox_msgs::RecoveryVehicleState* findVehicle(
        const sandbox_msgs::RecoverySnapshot& snapshot, int id) const;
    const OptimizationInputBuildResult* findPreserved(
        const std::vector<OptimizationInputBuildResult>& builds, int id) const;
    bool startsAtSourceB(const sandbox_msgs::RecoveryVehicleState& vehicle) const;
    bool oldCorridorContains(const NlpVehicleInput& input, const Pose2d& rear) const;

    MapParam map_param_;
    std::vector<Slot> slots_;
    BacktrackConfig config_;
};

}  // namespace forklift_nlp_recovery
