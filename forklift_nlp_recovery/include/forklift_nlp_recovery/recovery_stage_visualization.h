#pragma once

#include <string>

#include <visualization_msgs/MarkerArray.h>

#include "forklift_nlp_recovery/initial_guess.h"

namespace forklift_nlp_recovery {

void appendRecoveryStageMarkers(
    visualization_msgs::MarkerArray& markers,
    const std::string& frame_id,
    double field_width,
    double field_height,
    double containment_tolerance,
    const NlpOptimizationInput& input,
    const NlpInitialGuess& original,
    const NlpInitialGuess& actual,
    const std::string& prefix);

}  // namespace forklift_nlp_recovery
