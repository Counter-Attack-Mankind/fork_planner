#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "forklift_nlp_recovery/optimization_input.h"

namespace forklift_nlp_recovery {

enum class GuessValueSource {
    PATH_TIME_DIFFERENCE,
    PATH_GEOMETRY_ESTIMATE,
    FINITE_DIFFERENCE,
    TERMINAL_ZERO_ASSUMPTION,
};

const char* guessValueSourceName(GuessValueSource source);

struct InitialGuessConfig {
    size_t node_count = 51;
    double time_margin = 1.2;
    double containment_tolerance = 1e-8;
};

struct InitialGuessNode {
    size_t index = 0;
    double time = 0.0;
    double path_s = 0.0;
    uint8_t path_type = 0;
    Pose2d rear_axle_pose;
    Pose2d disc_center;
    double signed_speed = 0.0;
    double front_steer = 0.0;
    double acceleration = 0.0;
    double steer_rate = 0.0;
    int aabb_index = -1;
    double seed_offset_x = 0.0;
    double seed_offset_y = 0.0;
    GuessValueSource speed_source = GuessValueSource::PATH_TIME_DIFFERENCE;
    GuessValueSource steer_source = GuessValueSource::PATH_GEOMETRY_ESTIMATE;
    GuessValueSource acceleration_source = GuessValueSource::FINITE_DIFFERENCE;
    GuessValueSource steer_rate_source = GuessValueSource::FINITE_DIFFERENCE;
    double kinematic_position_residual = 0.0;
    double kinematic_heading_residual = 0.0;
};

struct VehicleInitialGuessDiagnostics {
    double reference_length = 0.0;
    double estimated_travel_time = 0.0;
    size_t direction_changes = 0;
    size_t shifted_seed_nodes = 0;
    double max_seed_offset = 0.0;
    double start_position_error = 0.0;
    double start_heading_error = 0.0;
    double start_speed_magnitude_error = 0.0;
    double terminal_position_error = 0.0;
    double terminal_heading_error = 0.0;
    double max_kinematic_position_residual = 0.0;
    double max_kinematic_heading_residual = 0.0;
    double max_speed_violation = 0.0;
    double max_steer_violation = 0.0;
    double max_acceleration_violation = 0.0;
    double max_steer_rate_violation = 0.0;
    double max_aabb_violation = 0.0;
    size_t bound_violation_nodes = 0;
    size_t aabb_violation_nodes = 0;
};

struct VehicleInitialGuess {
    int id = -1;
    bool valid = false;
    bool one_to_one_aabbs_valid = false;
    std::string reason;
    std::vector<InitialGuessNode> nodes;
    VehicleInitialGuessDiagnostics diagnostics;
};

struct InitialCollision {
    size_t node_index = 0;
    double time = 0.0;
    int vehicle_a_id = -1;
    int vehicle_b_id = -1;
    bool vehicle_b_is_static = false;
    double center_distance = 0.0;
    double required_distance = 0.0;
    double clearance = 0.0;
};

struct NlpInitialGuessDiagnostics {
    double max_kinematic_position_residual = 0.0;
    double max_kinematic_heading_residual = 0.0;
    size_t bound_violation_nodes = 0;
    size_t aabb_violation_nodes = 0;
    double max_aabb_violation = 0.0;
    size_t participant_collision_nodes = 0;
    size_t static_collision_nodes = 0;
    double minimum_participant_clearance = 0.0;
    double minimum_static_clearance = 0.0;
    std::vector<InitialCollision> collisions;
};

struct NlpInitialGuess {
    bool generated = false;
    bool constraints_satisfied = false;
    std::string reason;
    size_t node_count = 0;
    double total_time = 0.0;
    double average_dt = 0.0;
    std::vector<double> time_grid;
    std::vector<VehicleInitialGuess> vehicles;
    NlpInitialGuessDiagnostics diagnostics;
};

struct InitialGuessPreparationResult {
    NlpInitialGuess original_guess;
    NlpInitialGuess actual_guess;
};

class InitialGuessGenerator {
public:
    InitialGuessGenerator(const InitialGuessConfig& config,
                          const CorridorBuilder& corridor_builder);

    InitialGuessPreparationResult prepare(NlpOptimizationInput& input) const;

private:
    InitialGuessConfig config_;
    const CorridorBuilder& corridor_builder_;
};

struct CaseSaveResult {
    bool success = false;
    std::string case_id;
    std::string case_directory;
    std::string reason;
};

CaseSaveResult saveInitialGuessCase(const OptimizationInputBuildResult& input,
                                    const NlpInitialGuess& original_guess,
                                    const NlpInitialGuess& actual_guess,
                                    const std::string& results_directory,
                                    const std::string& case_prefix = "");

}  // namespace forklift_nlp_recovery
