#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "forklift_nlp_recovery/initial_guess.h"
#include "forklift_nlp_recovery/nlp_constraint_diagnostics.h"

namespace forklift_nlp_recovery {

struct NlpSolverConfig {
    double min_time_scale = 0.50;
    double max_time_scale = 3.00;
    double time_weight = 1.0;
    double acceleration_weight = 1.0;
    double steer_rate_weight = 5.0;
    double control_smooth_weight = 5.0;
    double reference_direction_weight = 5.0;
    double direction_smoothing_speed = 0.02;
    double direction_switch_speed_threshold = 0.01;
    int max_iterations = 1000;
    double tolerance = 1e-6;
    double acceptable_tolerance = 1e-4;
    int print_level = 3;
    size_t validation_substeps = 10;
    double validation_tolerance = 1e-5;
};

struct OptimizedTrajectoryNode {
    size_t index = 0;
    double time = 0.0;
    Pose2d rear_axle_pose;
    Pose2d disc_center;
    double signed_speed = 0.0;
    double front_steer = 0.0;
    double acceleration = 0.0;
    double steer_rate = 0.0;
    int aabb_index = -1;
};

struct OptimizedVehicleTrajectory {
    int id = -1;
    std::vector<OptimizedTrajectoryNode> nodes;
};

struct StaticMapViolation {
    int vehicle_id = -1;
    size_t interval_index = 0;
    size_t substep_index = 0;
    double sample_time = 0.0;
    Pose2d disc_center;
    std::string type;
    int shelf_index = -1;
    double violation_distance = 0.0;
};

struct NlpSolutionValidation {
    bool passed = false;
    std::string reason;
    double max_start_error = 0.0;
    double max_terminal_error = 0.0;
    double max_kinematic_position_residual = 0.0;
    double max_kinematic_heading_residual = 0.0;
    double max_speed_violation = 0.0;
    double max_steer_violation = 0.0;
    double max_acceleration_violation = 0.0;
    double max_steer_rate_violation = 0.0;
    double max_aabb_violation = 0.0;
    double minimum_participant_obb_separation_margin = 0.0;
    double minimum_static_obb_separation_margin = 0.0;
    size_t participant_obb_collision_samples = 0;
    size_t static_obb_collision_samples = 0;
    size_t aabb_violation_samples = 0;
    size_t static_map_violation_samples = 0;
    std::vector<StaticMapViolation> static_map_violations;
};

struct NlpSolverStatistics {
    bool success = false;
    std::string casadi_version;
    std::string solver_name = "ipopt";
    std::string return_status;
    int iterations = -1;
    double cpu_time = 0.0;
    double wall_time = 0.0;
    double objective = 0.0;
    double max_constraint_violation = 0.0;
};

struct VehicleDirectionDiagnostics {
    int id = -1;
    size_t initial_direction_changes = 0;
    size_t optimized_direction_changes = 0;
    double initial_opposite_direction_distance = 0.0;
    double optimized_opposite_direction_distance = 0.0;
};

struct NlpSolveResult {
    bool problem_built = false;
    bool solver_success = false;
    bool validation_passed = false;
    bool execution_publish_eligible = false;
    bool execution_published = false;
    std::string reason;
    size_t node_count = 0;
    double total_time = 0.0;
    double average_dt = 0.0;
    double direction_switch_speed_threshold = 0.0;
    std::vector<OptimizedVehicleTrajectory> vehicles;
    NlpSolverStatistics statistics;
    std::vector<VehicleDirectionDiagnostics> direction_diagnostics;
    NlpConstraintDiagnostics constraint_diagnostics;
    NlpSolutionValidation validation;
};

class NlpSolver {
public:
    NlpSolver(const NlpSolverConfig& config, const CorridorBuilder& corridor_builder);

    NlpSolveResult solve(const NlpOptimizationInput& input,
                         const NlpInitialGuess& initial_guess) const;

private:
    NlpSolverConfig config_;
    const CorridorBuilder& corridor_builder_;
};

bool saveNlpSolveResult(const NlpSolveResult& result,
                        const std::string& case_directory,
                        std::string& error);

}  // namespace forklift_nlp_recovery
