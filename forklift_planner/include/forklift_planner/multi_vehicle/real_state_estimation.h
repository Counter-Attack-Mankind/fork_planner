#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <utility>

#include "forklift_planner/multi_vehicle/path_track.h"

namespace forklift_planner {
namespace multi_vehicle {

struct RealProjectionResult {
    double path_s = 0.0;
    double best_xy_distance = std::numeric_limits<double>::infinity();
    double selected_xy_distance = std::numeric_limits<double>::infinity();
    double selected_heading_error = std::numeric_limits<double>::infinity();
    double selected_continuity_error = std::numeric_limits<double>::infinity();
    int candidate_count = 0;
};

inline double realProjectionAngleError(double a, double b) {
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

inline double signedPathMotionDirection(const PathTrack& track, double path_s,
                                        int progress_direction,
                                        double cusp_look_distance = 0.10) {
    if (track.empty()) return progress_direction < 0 ? -1.0 : 1.0;
    const double s = std::min(track.length(), std::max(0.0, path_s));
    bool reverse_segment = track.typeAtS(s) == WpType::REVERSE;
    if (progress_direction >= 0) {
        const double look_ahead = std::min(
            track.length(), s + cusp_look_distance);
        reverse_segment = reverse_segment ||
                          track.typeAtS(look_ahead) == WpType::REVERSE;
    }
    const double increasing_s_sign = reverse_segment ? -1.0 : 1.0;
    return progress_direction < 0 ? -increasing_s_sign : increasing_s_sign;
}

inline RealProjectionResult selectRealProjection(
    const PathTrack& track, double x, double y, double body_yaw,
    double previous_s, double current_speed, double dt,
    double search_s_min, double search_s_max,
    bool measured_motion_heading_valid = false,
    double measured_motion_heading = 0.0,
    double distance_slack = 0.08,
    int progress_direction = 1) {
    RealProjectionResult result;
    const double lo = progress_direction < 0
        ? std::min(previous_s, search_s_min)
        : std::max(previous_s, search_s_min);
    const double hi = std::max(lo, search_s_max);
    const double expected_s = std::min(
        hi, std::max(lo, previous_s +
            (progress_direction < 0 ? -1.0 : 1.0) *
            std::max(0.0, current_speed) * dt));

    struct Candidate {
        double s;
        double xy;
        double heading;
        double continuity;
    };
    std::deque<Candidate> candidates;
    auto sample = [&](double s) {
        const RoughWp pose = track.poseAtS(s);
        const bool reverse = track.typeAtS(s) == WpType::REVERSE;
        const double path_motion_heading =
            pose.theta + (reverse ? M_PI : 0.0) +
            (progress_direction < 0 ? M_PI : 0.0);
        const double measured_heading = measured_motion_heading_valid
            ? measured_motion_heading
            : body_yaw + (reverse ? M_PI : 0.0) +
              (progress_direction < 0 ? M_PI : 0.0);
        const double xy = std::hypot(pose.x - x, pose.y - y);
        candidates.push_back(Candidate{
            s, xy,
            realProjectionAngleError(measured_heading, path_motion_heading),
            std::abs(s - expected_s)});
        result.best_xy_distance = std::min(result.best_xy_distance, xy);
    };

    for (double s = lo; s < hi - 1e-9; s += 0.01) sample(s);
    sample(hi);

    double best_score = std::numeric_limits<double>::infinity();
    result.path_s = previous_s;
    for (const Candidate& candidate : candidates) {
        if (candidate.xy > result.best_xy_distance + distance_slack + 1e-9) {
            continue;
        }
        ++result.candidate_count;
        // XY remains the primary gate above. Inside that near-distance set,
        // heading rejects the opposite traversal branch and continuity keeps
        // the estimate near the expected short-term arc-length advance.
        const double score = candidate.xy +
            0.08 * candidate.heading / M_PI +
            0.50 * candidate.continuity;
        if (score < best_score) {
            best_score = score;
            result.path_s = candidate.s;
            result.selected_xy_distance = candidate.xy;
            result.selected_heading_error = candidate.heading;
            result.selected_continuity_error = candidate.continuity;
        }
    }
    return result;
}

struct ArcLengthSpeedResult {
    double raw_single_step_speed = 0.0;
    double window_speed = 0.0;
    double window_duration = 0.0;
    std::size_t window_samples = 0;
};

class ArcLengthSpeedWindow {
public:
    explicit ArcLengthSpeedWindow(double duration = 0.4)
        : duration_(duration) {}

    void clear(double trusted_speed = 0.0) {
        samples_.clear();
        last_speed_ = std::max(0.0, trusted_speed);
    }

    ArcLengthSpeedResult update(double timestamp, double path_s,
                                double previous_path_s, double dt,
                                double max_speed,
                                int progress_direction = 1) {
        ArcLengthSpeedResult result;
        if (dt > 1e-9) {
            result.raw_single_step_speed = std::max(0.0,
                (progress_direction < 0 ? previous_path_s - path_s
                                        : path_s - previous_path_s) / dt);
        }
        samples_.emplace_back(timestamp, path_s);
        while (samples_.size() > 1 &&
               timestamp - samples_.front().first > duration_ + 1e-9) {
            samples_.pop_front();
        }
        result.window_samples = samples_.size();
        if (samples_.size() >= 2) {
            result.window_duration =
                samples_.back().first - samples_.front().first;
            if (result.window_duration > 1e-9) {
                const double signed_ds = samples_.back().second -
                                         samples_.front().second;
                const double ds = std::max(0.0,
                    progress_direction < 0 ? -signed_ds : signed_ds);
                last_speed_ = std::max(
                    0.0, std::min(max_speed, ds / result.window_duration));
            }
        }
        result.window_speed = last_speed_;
        return result;
    }

    std::size_t sampleCount() const { return samples_.size(); }

private:
    double duration_ = 0.4;
    double last_speed_ = 0.0;
    std::deque<std::pair<double, double>> samples_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
