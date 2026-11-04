#pragma once

#include <casadi/casadi.hpp>

#include <vector>

#include "forklift_nlp_recovery/nlp_constraint_diagnostics.h"
#include "forklift_nlp_recovery/optimization_input.h"

namespace forklift_nlp_recovery {

void addDynamicObca(
    const casadi::MX& first_state, const casadi::MX& second_state,
    const Pose2d& initial_first_rear, const Pose2d& initial_second_rear,
    const VehicleParameters& first, const VehicleParameters& second,
    int first_id, int second_id, size_t node, const std::string& sample_name,
    NlpConstraintCategory category, std::vector<casadi::MX>& variables,
    std::vector<double>& initial_values, std::vector<double>& lower_bounds,
    std::vector<double>& upper_bounds,
    std::vector<NlpConstraintMetadata>& variable_metadata,
    std::vector<casadi::MX>& constraint_expressions,
    std::vector<double>& constraint_lower, std::vector<double>& constraint_upper,
    std::vector<NlpConstraintMetadata>& constraint_metadata);

}  // namespace forklift_nlp_recovery
