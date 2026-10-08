#pragma once

#include <string>

#include <visualization_msgs/MarkerArray.h>

#include "forklift_planner/path_generator.h"

namespace forklift_map {
namespace path_debug {

struct PathDebugVisualOptions {
    bool show_layers = false;
    bool show_turn_types = true;
    bool show_corner_labels = true;
    double z_offset = 0.13;
};

void addPathDebugLayers(visualization_msgs::MarkerArray& arr,
                        int& id,
                        const std::string& frame_id,
                        const std::string& ns_prefix,
                        const RoughPath& path,
                        const PathGenerationInfo& info,
                        const std::string& label,
                        const PathDebugVisualOptions& options);

}  // namespace path_debug
}  // namespace forklift_map
