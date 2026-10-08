#include "forklift_planner/multi_vehicle/path_track.h"

#include <algorithm>
#include <cmath>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

constexpr double kPi = M_PI;

double normAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

}  // namespace

void PathTrack::set(const RoughPath& path) {
    path_ = path;
    s_.clear();
    s_.reserve(path_.size());
    length_ = 0.0;
    for (size_t i = 0; i < path_.size(); ++i) {
        if (i > 0) {
            length_ += std::hypot(path_[i].x - path_[i - 1].x,
                                  path_[i].y - path_[i - 1].y);
        }
        s_.push_back(length_);
    }
}

bool PathTrack::empty() const {
    return path_.size() < 2 || length_ <= 1e-6;
}

RoughWp PathTrack::poseAtS(double query_s) const {
    if (path_.empty()) return {};
    if (query_s <= 0.0) return path_.front();
    if (query_s >= length_ - 1e-9) return path_.back();

    auto it = std::lower_bound(s_.begin(), s_.end(), query_s);
    size_t idx = static_cast<size_t>(std::distance(s_.begin(), it));
    if (idx == 0) return path_.front();
    if (idx >= path_.size()) return path_.back();

    const double seg_len = s_[idx] - s_[idx - 1];
    const double ratio = seg_len > 1e-9 ? (query_s - s_[idx - 1]) / seg_len : 1.0;
    const RoughWp& a = path_[idx - 1];
    const RoughWp& b = path_[idx];

    RoughWp out;
    out.x = a.x + (b.x - a.x) * ratio;
    out.y = a.y + (b.y - a.y) * ratio;
    out.theta = normAngle(a.theta + normAngle(b.theta - a.theta) * ratio);
    out.type = ratio < 0.5 ? a.type : b.type;
    return out;
}

WpType PathTrack::typeAtS(double query_s) const {
    return poseAtS(query_s).type;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
