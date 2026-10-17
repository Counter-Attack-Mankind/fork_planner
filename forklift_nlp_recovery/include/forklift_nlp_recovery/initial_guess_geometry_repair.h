#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "forklift_nlp_recovery/corridor_builder.h"
#include "forklift_nlp_recovery/initial_guess.h"

namespace forklift_nlp_recovery {

struct InitialGuessGeometryRepairConfig {
    bool enabled = true;
    size_t transition_nodes = 2;
    size_t smoothing_iterations = 12;
    size_t interval_samples = 10;
    double max_node_displacement = 0.15;
    double aabb_inset = 0.002;
    double max_heading_adjustment = 0.35;
    double containment_tolerance = 1e-8;
};

struct RepairedNodeDiagnostic {
    size_t node_index = 0;
    int assigned_aabb = -1;
    bool originally_uncovered = false;
    double displacement = 0.0;
};

struct VehicleGeometryRepairDiagnostics {
    int id = -1;
    bool repair_attempted = false;
    bool repaired = false;
    bool aabb_assignment_valid = false;
    std::string reason;
    size_t original_uncovered_nodes = 0;
    size_t moved_nodes = 0;
    double max_displacement = 0.0;
    double max_aabb_violation_before = 0.0;
    double max_aabb_violation_after = 0.0;
    double max_interval_aabb_violation_before = 0.0;
    double max_interval_aabb_violation = 0.0;
    double max_curvature_before = 0.0;
    double max_curvature = 0.0;
    double max_kinematic_position_residual_before = 0.0;
    double max_kinematic_heading_residual_before = 0.0;
    double max_kinematic_position_residual = 0.0;
    double max_kinematic_heading_residual = 0.0;
    size_t disc_safety_violations_before = 0;
    size_t disc_safety_violations = 0;
    size_t static_obb_safety_violations_before = 0;
    size_t static_obb_safety_violations = 0;
    size_t participant_disc_collisions_before = 0;
    size_t participant_obb_collisions_before = 0;
    size_t participant_disc_collisions = 0;
    size_t participant_obb_collisions = 0;
    std::vector<RepairedNodeDiagnostic> nodes;
};

struct InitialGuessGeometryRepairResult {
    bool generated = false;
    bool repaired = false;
    bool aabb_assignment_valid = false;
    std::string reason;
    NlpInitialGuess repaired_guess;
    std::vector<VehicleGeometryRepairDiagnostics> vehicles;
};

class InitialGuessGeometryRepair {
public:
    InitialGuessGeometryRepair(const InitialGuessGeometryRepairConfig& config,
                               const CorridorBuilder& corridor_builder);

    InitialGuessGeometryRepairResult repair(
        const NlpOptimizationInput& input,
        const NlpInitialGuess& original_guess) const;

private:
    InitialGuessGeometryRepairConfig config_;
    const CorridorBuilder& corridor_builder_;
};

bool saveGeometryRepairReport(const InitialGuessGeometryRepairResult& result,
                              const std::string& case_directory,
                              std::string& error);

}  // namespace forklift_nlp_recovery
