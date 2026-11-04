#pragma once

#include "forklift_nlp_recovery/nlp_solver.h"

namespace forklift_nlp_recovery {

class CooperativeNlpSolver {
public:
    CooperativeNlpSolver(const NlpSolverConfig& config,
                         const CorridorBuilder& corridor_builder);

    NlpSolveResult solve(const NlpOptimizationInput& input,
                         const NlpInitialGuess& initial_guess) const;

private:
    NlpSolverConfig config_;
    const CorridorBuilder& corridor_builder_;
};

}  // namespace forklift_nlp_recovery
