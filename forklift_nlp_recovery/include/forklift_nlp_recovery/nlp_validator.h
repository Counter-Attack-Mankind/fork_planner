#pragma once

#include "forklift_nlp_recovery/nlp_solver.h"

namespace forklift_nlp_recovery {

NlpSolutionValidation validateSolution(const NlpOptimizationInput& input,
                                       const NlpSolveResult& result,
                                       const NlpSolverConfig& config,
                                       const CorridorBuilder& corridor_builder);

}  // namespace forklift_nlp_recovery
