#pragma once

#include <vector>

#include "forklift_planner/path_generator.h"

namespace forklift_planner {
namespace multi_vehicle {

class PathTrack {
public:
    void set(const RoughPath& path);

    bool empty() const;
    double length() const { return length_; }
    const RoughPath& path() const { return path_; }
    const std::vector<double>& cumulative_s() const { return s_; }

    RoughWp poseAtS(double s) const;
    WpType typeAtS(double s) const;

private:
    RoughPath path_;
    std::vector<double> s_;
    double length_ = 0.0;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
