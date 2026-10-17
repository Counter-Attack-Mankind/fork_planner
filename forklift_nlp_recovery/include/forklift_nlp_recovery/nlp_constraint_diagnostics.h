#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace forklift_nlp_recovery {

enum class NlpConstraintCategory {
    RK4,
    AABB_NODE,
    AABB_MIDPOINT,
    PARTICIPANT_DISC,
    STATIC_DISC,
    VARIABLE_BOUND
};

struct NlpConstraintMetadata {
    NlpConstraintCategory category = NlpConstraintCategory::RK4;
    int vehicle_id = -1;
    int other_vehicle_id = -1;
    size_t node_index = 0;
    std::string component;
    double required_distance = 0.0;
};

struct NlpConstraintDiagnosticsInput {
    std::vector<double> constraint_values;
    std::vector<double> constraint_lower_bounds;
    std::vector<double> constraint_upper_bounds;
    std::vector<NlpConstraintMetadata> constraint_metadata;
    std::vector<double> variable_values;
    std::vector<double> variable_lower_bounds;
    std::vector<double> variable_upper_bounds;
    std::vector<NlpConstraintMetadata> variable_metadata;
};

struct NlpConstraintCategoryDiagnostics {
    NlpConstraintCategory category = NlpConstraintCategory::RK4;
    size_t sample_count = 0;
    size_t violation_count = 0;
    double max_violation = 0.0;
    double max_distance_shortfall = 0.0;
};

struct NlpConstraintViolation {
    NlpConstraintMetadata metadata;
    double value = 0.0;
    double lower_bound = 0.0;
    double upper_bound = 0.0;
    double violation = 0.0;
    double actual_distance = 0.0;
    double required_distance = 0.0;
    double distance_shortfall = 0.0;
};

struct NlpConstraintDiagnostics {
    bool available = false;
    std::string reason;
    double max_violation = 0.0;
    std::vector<NlpConstraintCategoryDiagnostics> categories;
    std::vector<NlpConstraintViolation> worst_violations;
};

NlpConstraintDiagnostics diagnoseNlpConstraints(
    const NlpConstraintDiagnosticsInput& input, size_t worst_limit = 10);
const char* nlpConstraintCategoryName(NlpConstraintCategory category);

}  // namespace forklift_nlp_recovery
