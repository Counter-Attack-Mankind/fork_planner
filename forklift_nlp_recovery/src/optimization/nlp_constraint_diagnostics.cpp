#include "forklift_nlp_recovery/nlp_constraint_diagnostics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace forklift_nlp_recovery {
namespace {

constexpr std::array<NlpConstraintCategory, 4> kCategories{
    NlpConstraintCategory::RK4,
    NlpConstraintCategory::AABB_NODE,
    NlpConstraintCategory::STATIC_OBCA,
    NlpConstraintCategory::VARIABLE_BOUND};

double boundViolation(double value, double lower, double upper) {
    if (!std::isfinite(value)) return std::numeric_limits<double>::infinity();
    double violation = 0.0;
    if (std::isfinite(lower)) violation = std::max(violation, lower - value);
    if (std::isfinite(upper)) violation = std::max(violation, value - upper);
    return std::max(0.0, violation);
}

NlpConstraintCategoryDiagnostics& categoryDiagnostics(
    NlpConstraintDiagnostics& result, NlpConstraintCategory category) {
    return *std::find_if(result.categories.begin(), result.categories.end(),
        [&](const NlpConstraintCategoryDiagnostics& value) {
            return value.category == category;
        });
}

void appendDiagnostics(NlpConstraintDiagnostics& result,
                       const std::vector<double>& values,
                       const std::vector<double>& lower,
                       const std::vector<double>& upper,
                       const std::vector<NlpConstraintMetadata>& metadata) {
    for (size_t index = 0; index < values.size(); ++index) {
        NlpConstraintCategoryDiagnostics& category =
            categoryDiagnostics(result, metadata[index].category);
        ++category.sample_count;
        const double violation = boundViolation(values[index], lower[index], upper[index]);
        category.max_violation = std::max(category.max_violation, violation);
        result.max_violation = std::max(result.max_violation, violation);

        NlpConstraintViolation diagnostic;
        diagnostic.metadata = metadata[index];
        diagnostic.value = values[index];
        diagnostic.lower_bound = lower[index];
        diagnostic.upper_bound = upper[index];
        diagnostic.violation = violation;
        if (metadata[index].category == NlpConstraintCategory::STATIC_OBCA &&
            metadata[index].component.find("separation") != std::string::npos) {
            diagnostic.distance_shortfall = violation;
            category.max_distance_shortfall = std::max(
                category.max_distance_shortfall, diagnostic.distance_shortfall);
        }
        if (violation > 0.0) {
            ++category.violation_count;
            result.worst_violations.push_back(std::move(diagnostic));
        }
    }
}

}  // namespace

const char* nlpConstraintCategoryName(NlpConstraintCategory category) {
    switch (category) {
        case NlpConstraintCategory::RK4: return "rk4";
        case NlpConstraintCategory::AABB_NODE: return "aabb_node";
        case NlpConstraintCategory::STATIC_OBCA: return "static_obca";
        case NlpConstraintCategory::VARIABLE_BOUND: return "variable_bound";
    }
    return "unknown";
}

NlpConstraintDiagnostics diagnoseNlpConstraints(
    const NlpConstraintDiagnosticsInput& input, size_t worst_limit) {
    NlpConstraintDiagnostics result;
    for (NlpConstraintCategory category : kCategories) {
        result.categories.push_back({category});
    }
    const bool constraint_sizes_match =
        input.constraint_values.size() == input.constraint_lower_bounds.size() &&
        input.constraint_values.size() == input.constraint_upper_bounds.size() &&
        input.constraint_values.size() == input.constraint_metadata.size();
    const bool variable_sizes_match =
        input.variable_values.size() == input.variable_lower_bounds.size() &&
        input.variable_values.size() == input.variable_upper_bounds.size() &&
        input.variable_values.size() == input.variable_metadata.size();
    if (!constraint_sizes_match || !variable_sizes_match) {
        result.reason = "raw value, bound and metadata sizes differ";
        return result;
    }

    appendDiagnostics(result, input.constraint_values,
                      input.constraint_lower_bounds,
                      input.constraint_upper_bounds,
                      input.constraint_metadata);
    appendDiagnostics(result, input.variable_values,
                      input.variable_lower_bounds,
                      input.variable_upper_bounds,
                      input.variable_metadata);
    std::sort(result.worst_violations.begin(), result.worst_violations.end(),
        [](const NlpConstraintViolation& first,
           const NlpConstraintViolation& second) {
            if (first.violation != second.violation) {
                return first.violation > second.violation;
            }
            return first.distance_shortfall > second.distance_shortfall;
        });
    if (result.worst_violations.size() > worst_limit) {
        result.worst_violations.resize(worst_limit);
    }
    result.available = true;
    result.reason = "ok";
    return result;
}

}  // namespace forklift_nlp_recovery
