#pragma once

#include "forklift_nlp_recovery/initial_guess.h"

namespace forklift_nlp_recovery {

class CooperativeInitialGuessGenerator {
public:
    explicit CooperativeInitialGuessGenerator(const InitialGuessConfig& config);

    NlpInitialGuess generate(const NlpOptimizationInput& input) const;

private:
    InitialGuessConfig config_;
};

}  // namespace forklift_nlp_recovery
