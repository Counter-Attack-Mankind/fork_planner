#pragma once

#include <string>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_nlp_recovery/optimization_input.h"

namespace forklift_nlp_recovery {

struct CandidateSelectionConfig {
    double stopped_speed_tolerance = 0.01;
    double sweep_step = 0.01;
    double minimum_effective_gain = 0.05;
    double maximum_negative_gain = 0.02;
    double bridge_escape_rear_clearance = -1.0;
};

struct CandidateEvaluation {
    int vehicle_id = -1;
    bool eligible = false;
    std::string reason;
    OptimizationInputBuildResult build;
    std::vector<int> beneficiary_ids;
    std::vector<double> gains;
    int positive_gain_count = 0;
    double total_positive_gain = 0.0;
    double worst_gain = 0.0;
};

struct CandidateSelectionResult {
    bool complete_snapshot = false;
    std::string reason;
    std::vector<CandidateEvaluation> ranked;
};

struct BridgeEscapeEvaluation {
    int vehicle_id = -1;
    int path_gen = -1;
    bool intersects_bridge = false;
    double frozen_path_s = 0.0;
    Pose2d frozen_pose;
    bool escape_valid = false;
    double escape_distance = 0.0;
    int motion_direction = 1;
    Pose2d escape_pose;
    std::vector<PathPoint> escape_segment;
    bool sweep_failure_valid = false;
    double sweep_failure_distance = 0.0;
    Pose2d sweep_failure_pose;
    bool sweep_failure_at_start = false;
    std::string sweep_failure_reason;
    bool eligible = false;
    std::string reason;
    CandidateEvaluation candidate;
};

struct BridgeEscapeSelectionResult {
    bool complete_snapshot = false;
    std::string reason;
    std::vector<BridgeEscapeEvaluation> ranked;
};

class CandidateSelector {
public:
    CandidateSelector(const MapParam& map_param,
                      const std::vector<ShelfBlock>& shelves,
                      const CandidateSelectionConfig& config,
                      const OptimizationInputBuilder& input_builder);

    CandidateSelectionResult evaluate(
        const sandbox_msgs::RecoverySnapshot& snapshot,
        const std::vector<int>& deadlock_cycle_ids,
        int off_path_start_vehicle_id = -1) const;

    BridgeEscapeSelectionResult evaluateBridgeEscapes(
        const sandbox_msgs::RecoverySnapshot& snapshot,
        const std::vector<int>& deadlock_cycle_ids) const;

private:
    const sandbox_msgs::RecoveryVehicleState* findVehicle(
        const sandbox_msgs::RecoverySnapshot& snapshot, int id) const;
    bool intersectsRow2Bridge(
        const sandbox_msgs::RecoveryVehicleState& vehicle) const;
    double continuousSafeDistance(
        const sandbox_msgs::RecoveryVehicleState& vehicle,
        const sandbox_msgs::RecoverySnapshot& snapshot,
        int moved_vehicle_id, const Pose2d* moved_pose) const;
    CandidateEvaluation evaluateBuiltCandidate(
        const sandbox_msgs::RecoverySnapshot& snapshot,
        const std::vector<int>& deadlock_cycle_ids,
        const std::vector<double>& before, int candidate_id,
        OptimizationInputBuildResult build) const;

    MapParam map_param_;
    std::vector<ShelfBlock> shelves_;
    CandidateSelectionConfig config_;
    const OptimizationInputBuilder& input_builder_;
};

}  // namespace forklift_nlp_recovery
