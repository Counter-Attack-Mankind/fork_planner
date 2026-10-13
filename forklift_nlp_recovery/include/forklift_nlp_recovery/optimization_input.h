#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sandbox_msgs/RecoverySnapshot.h>

#include "forklift_nlp_recovery/corridor_builder.h"

namespace forklift_nlp_recovery {

struct VehicleParameters {
    double length = 0.0;
    double width = 0.0;
    double rear_axle_to_center = 0.0;
    double disc_radius = 0.0;
    double wheel_base = 0.0;
    double max_steer_angle = 0.0;
    double max_steer_rate = 0.0;
    double nominal_speed = 0.0;
    double max_speed = 0.0;
    double max_accel = 0.0;
    double max_decel = 0.0;
    double fixed_retreat_speed = 0.0;
};

struct InitialMotionState {
    Pose2d rear_axle_pose;
    double longitudinal_speed = 0.0;
};

struct NlpVehicleInput {
    int id = -1;
    // Rear-axle state remains the optimization reference. Corridor boxes and
    // seeds are body-disc-center geometry produced by CorridorBuilder.
    InitialMotionState initial_state;
    double path_s = 0.0;
    int path_gen = -1;
    std::vector<PathPoint> original_path;
    RejoinPoint terminal;
    std::vector<Aabb> corridor;
    std::vector<Pose2d> corridor_seeds;
    bool corridor_valid = false;
    std::string corridor_reason;
    VehicleParameters parameters;
};

struct StaticVehicleObstacle {
    int id = -1;
    Pose2d rear_axle_pose;
    Pose2d disc_center;
    double disc_radius = 0.0;
    double min_distance_to_corridor = 0.0;
    double influence_distance = 0.0;
    std::vector<int> influencing_vehicle_ids;
};

struct NlpOptimizationInput {
    bool valid = false;
    std::string reason;
    std::string trigger_reason;
    std::string source_mode;
    std::string frame_id;
    uint32_t stamp_sec = 0;
    uint32_t stamp_nsec = 0;
    std::vector<int> participant_ids;
    std::vector<NlpVehicleInput> vehicles;
    std::vector<StaticVehicleObstacle> static_vehicles;
};

struct OptimizationInputBuildResult {
    // complete=false means one or more selected vehicles were missing a usable
    // pose/path. A corridor failure is complete input with input.valid=false.
    bool complete = false;
    std::string reason;
    NlpOptimizationInput input;
};

class OptimizationInputBuilder {
public:
    OptimizationInputBuilder(const CorridorBuilder& corridor_builder,
                             const VehicleParameters& vehicle_parameters);

    OptimizationInputBuildResult build(
        const sandbox_msgs::RecoverySnapshot& snapshot,
        const std::vector<int>& participant_ids,
        const std::string& source_mode) const;

    static double pointToAabbDistance(const Pose2d& point, const Aabb& box);
    static bool discMayAffectCorridor(const Pose2d& center, double static_radius,
                                     const NlpVehicleInput& participant);

private:
    const sandbox_msgs::RecoveryVehicleState* findVehicle(
        const sandbox_msgs::RecoverySnapshot& snapshot, int id) const;
    std::vector<PathPoint> pathFromMessage(
        const sandbox_msgs::RecoveryVehicleState& vehicle) const;
    void selectStaticVehicles(const sandbox_msgs::RecoverySnapshot& snapshot,
                              NlpOptimizationInput& input) const;

    const CorridorBuilder& corridor_builder_;
    VehicleParameters vehicle_parameters_;
};

bool writeOptimizationInputDebugFile(const OptimizationInputBuildResult& result,
                                     const std::string& path,
                                     std::string& error);

}  // namespace forklift_nlp_recovery
