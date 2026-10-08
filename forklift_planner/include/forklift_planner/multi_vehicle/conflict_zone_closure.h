#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace forklift_planner {
namespace multi_vehicle {

// Inserts one newly sampled row component and closes the connected component
// to a fixed point.  A merge can enlarge candidate enough to touch a component
// rejected earlier in the scan, so a single pass is insufficient.
template <typename Zone, typename Touches, typename Merge>
void insertConflictZoneWithClosure(Zone candidate,
                                   std::vector<Zone>& components,
                                   Touches touches, Merge merge) {
    std::size_t insertion_index = components.size();
    bool absorbed = false;
    do {
        absorbed = false;
        for (std::size_t index = 0; index < components.size();) {
            if (!touches(candidate, components[index])) {
                ++index;
                continue;
            }
            insertion_index = std::min(insertion_index, index);
            merge(candidate, components[index]);
            components.erase(components.begin() + index);
            absorbed = true;
        }
    } while (absorbed);

    insertion_index = std::min(insertion_index, components.size());
    components.insert(components.begin() + insertion_index,
                      std::move(candidate));
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
