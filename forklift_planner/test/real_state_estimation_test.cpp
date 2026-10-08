#include <cmath>
#include <iostream>

#include "forklift_planner/multi_vehicle/real_state_estimation.h"

namespace {

int fail(const char* message) {
    std::cerr << "real_state_estimation_test: " << message << "\n";
    return 1;
}

}  // namespace

int main() {
    using forklift_planner::multi_vehicle::ArcLengthSpeedWindow;
    using forklift_planner::multi_vehicle::PathTrack;
    using forklift_planner::multi_vehicle::selectRealProjection;
    using forklift_planner::multi_vehicle::signedPathMotionDirection;

    ArcLengthSpeedWindow speed_window(0.4);
    const double samples[] = {0.00, 0.01, 0.01, 0.03, 0.04};
    double previous = samples[0];
    auto speed = speed_window.update(0.0, samples[0], samples[0], 0.1, 0.26);
    for (int i = 1; i < 5; ++i) {
        speed = speed_window.update(0.1 * i, samples[i], previous, 0.1, 0.26);
        previous = samples[i];
        if (i == 2 &&
            (std::abs(speed.raw_single_step_speed) > 1e-9 ||
             speed.window_speed <= 0.0)) {
            return fail("one stationary sample collapsed the window speed");
        }
    }
    if (std::abs(speed.window_speed - 0.10) > 1e-9 ||
        speed.window_samples != 5 ||
        std::abs(speed.window_duration - 0.4) > 1e-9) {
        return fail("0.4 s arc-length speed window mismatch");
    }

    // Two locally close branches: the later branch is 2 mm closer in XY but
    // is traversed in the opposite direction. A pure XY projection would pick
    // it; direction and continuity must keep the forward branch.
    RoughPath path{
        RoughWp{0.00, 0.00, 0.0, WpType::FORWARD},
        RoughWp{0.10, 0.00, 0.0, WpType::FORWARD},
        RoughWp{0.10, 0.02, 0.0, WpType::REVERSE},
        RoughWp{0.00, 0.02, 0.0, WpType::REVERSE},
    };
    PathTrack track;
    track.set(path);
    const auto projection = selectRealProjection(
        track, 0.05, 0.011, 0.0, 0.0, 0.10, 0.1,
        0.0, track.length(), true, 0.0);
    if (projection.path_s >= 0.10 ||
        projection.selected_heading_error > 0.1 ||
        projection.candidate_count < 2) {
        return fail("projection selected the closer opposite-motion branch");
    }

    RoughPath retreat_path{
        RoughWp{0.0, 0.0, 0.0, WpType::FORWARD},
        RoughWp{1.0, 0.0, 0.0, WpType::FORWARD},
    };
    PathTrack retreat_track;
    retreat_track.set(retreat_path);
    const auto retreat_projection = selectRealProjection(
        retreat_track, 0.70, 0.0, 0.0, 0.80, 0.10, 0.1,
        0.30, 0.80, true, M_PI, 0.08, -1);
    if (retreat_projection.path_s >= 0.80 - 1e-9) {
        return fail("decreasing-s projection remained monotonic-forward");
    }
    ArcLengthSpeedWindow retreat_speed_window(0.4);
    retreat_speed_window.update(0.0, 0.80, 0.80, 0.1, 0.26, -1);
    const auto retreat_speed = retreat_speed_window.update(
        0.1, 0.70, 0.80, 0.1, 0.26, -1);
    if (retreat_speed.raw_single_step_speed <= 0.0 ||
        retreat_speed.window_speed <= 0.0) {
        return fail("decreasing-s motion did not produce positive speed magnitude");
    }

    RoughPath cusp_path{
        RoughWp{0.0, 0.0, 0.0, WpType::FORWARD},
        RoughWp{1.0, 0.0, 0.0, WpType::FORWARD},
        RoughWp{1.2, 0.0, 0.0, WpType::REVERSE},
        RoughWp{0.2, 0.0, 0.0, WpType::REVERSE},
    };
    PathTrack cusp_track;
    cusp_track.set(cusp_path);
    if (signedPathMotionDirection(cusp_track, 1.00, -1) >= 0.0) {
        return fail("FORWARD-side retreat used the normal cusp look-ahead sign");
    }
    if (signedPathMotionDirection(cusp_track, 1.20, -1) <= 0.0) {
        return fail("REVERSE-side retreat did not invert segment motion sign");
    }

    std::cout << "real_state_estimation_test: PASS\n";
    return 0;
}
