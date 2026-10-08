#include "forklift_planner/path_generator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ros/console.h>
#include <utility>
#include <vector>

#include "forklift_planner/common/geometry2d.h"
#include "forklift_planner/path_generator_internal.h"


namespace {

using forklift_planner::geometry2d::Pt;
using forklift_planner::geometry2d::cross;
using forklift_planner::geometry2d::dist;
using forklift_planner::geometry2d::dot;
using forklift_planner::geometry2d::left_normal;
using forklift_planner::geometry2d::normalize;
using forklift_planner::geometry2d::right_normal;
using namespace forklift_planner::path_internal;

}

RoughPath PathGenerator::generateRouteBToA2(const Slot& src, const Slot& tgt,
                                             PathGenerationInfo* info) const {
    if (info != nullptr) {
        *info = PathGenerationInfo{};
    }


    const double row1_lane_off = dual_lane_offset(mp_.row1_left_aisle);
    const double row3_lane_off = dual_lane_offset(mp_.row3_center_aisle);

    const double row1_left_down_x  = left_bypass_x_  - row1_lane_off;
    const double row1_left_up_x    = left_bypass_x_  + row1_lane_off;

    const double row1_right_down_x = right_bypass_x_ - row1_lane_off;
    const double row1_right_up_x   = right_bypass_x_ + row1_lane_off;

    const double row3_down_x       = spine_x_        - row3_lane_off;
    const double row3_up_x         = spine_x_        + row3_lane_off;

    const int src_corr = corridor_id(src.row_id);
    const int tgt_corr = corridor_id(tgt.row_id);
    const bool target_is_endpoint = tgt.id < 0;
    const char* route_mode_name = "AUTO";
    switch (route_mode_) {
        case PathGeneratorRouteMode::A1_TO_B: route_mode_name = "A1_TO_B"; break;
        case PathGeneratorRouteMode::B_TO_A1: route_mode_name = "B_TO_A1"; break;
        case PathGeneratorRouteMode::A2_TO_B: route_mode_name = "A2_TO_B"; break;
        case PathGeneratorRouteMode::B_TO_A2: route_mode_name = "B_TO_A2"; break;
        case PathGeneratorRouteMode::AUTO: break;
    }
    const bool debug_row1_target =
        (tgt.row_id == 1 || tgt.row_id == 5 ||
         (target_is_endpoint && (src.row_id == 1 || src.row_id == 5)));

    const double max_curvature  = mp_.turn_max_curvature();
    const double steer_ramp_len = mp_.turn_ramp_len();
    const double sample_ds      = mp_.turn_ds();
    const double final_turn_sign =
        (std::sin(tgt.dock_theta) >= 0.0) ? kPi * 0.5 : -kPi * 0.5;
    const double terminal_margin_y = 0.04;
    const double terminal_min_y = corridor_min_y(mp_, tgt_corr) + terminal_margin_y;
    const double terminal_max_y = corridor_max_y(mp_, tgt_corr) - terminal_margin_y;
    const double terminal_y_space = (std::sin(tgt.dock_theta) >= 0.0)
        ? std::max(0.0, tgt.pre_dock_y - terminal_min_y)
        : std::max(0.0, terminal_max_y - tgt.pre_dock_y);
    TurnCurve final_turn_req = fit_clothoid_turn(final_turn_sign, max_curvature,
                                                 steer_ramp_len,
                                                 std::max(sample_ds, terminal_y_space * 0.98),
                                                 sample_ds);
    if (final_turn_req.pts.empty()) {
        final_turn_req = build_clothoid_turn(final_turn_sign, max_curvature,
                                             steer_ramp_len, sample_ds);
    }
    const TurnCurve final_turn_min =
        build_clothoid_turn(final_turn_sign, max_curvature, sample_ds, sample_ds);
    const double final_req_y = std::max(0.20, final_turn_req.t_out + 0.02);
    const double final_req_x = std::max(0.20, final_turn_req.t_in + 0.02);
    const double final_min_req_y = std::max(0.20, final_turn_min.t_out + 0.02);
    const double final_min_req_x = std::max(0.20, final_turn_min.t_in + 0.02);
    const double target_x = tgt.pre_dock_x;
    auto row3_terminal_lane_x = [&]() {
        return (target_x < spine_x_) ? row3_down_x : row3_up_x;
    };
    const double nominal_terminal_x =
        target_is_endpoint ? spine_x_ : ((tgt_corr == 4) ? row3_terminal_lane_x() : spine_x_);
    const double left_terminal_x = target_x - final_min_req_x;
    const double right_terminal_x = target_x + final_min_req_x;
    double final_reference_x = nominal_terminal_x;
    const bool keep_terminal_lane_center = target_is_endpoint || (tgt_corr == 4);
    if (!keep_terminal_lane_center &&
        final_reference_x > left_terminal_x && final_reference_x < right_terminal_x) {
        const bool left_ok = left_terminal_x >= 0.04;
        const bool right_ok = right_terminal_x <= mp_.field_width - 0.04;
        if (left_ok && right_ok) {
            final_reference_x =
                (std::abs(nominal_terminal_x - left_terminal_x) <=
                 std::abs(nominal_terminal_x - right_terminal_x))
                    ? left_terminal_x : right_terminal_x;
        } else if (left_ok) {
            final_reference_x = left_terminal_x;
        } else if (right_ok) {
            final_reference_x = right_terminal_x;
        }
    }
    const double dock_dir_y_for_lane = std::sin(tgt.dock_theta);
    const bool single_row_terminal = (tgt.row_id == 0 || tgt.row_id == 7);
    const double terminal_rear_stop_y =
        tgt.dock_y() - mp_.rear_axle_to_center * std::sin(tgt.dock_theta);
    const double forward_terminal_y =
        single_row_terminal ? tgt.dock_y() : terminal_rear_stop_y;
    const HDir final_hdir = (target_x >= final_reference_x) ? HDir::RIGHT : HDir::LEFT;
    const HDir far_hdir   = (final_hdir == HDir::RIGHT) ? HDir::LEFT : HDir::RIGHT;
    const double forward_lane_y     = corridor_lane_y(mp_, tgt_corr, final_hdir);
    const double far_lane_y         = corridor_lane_y(mp_, tgt_corr, far_hdir);
    auto lane_approach_gap = [&](double lane_y) {
        return (dock_dir_y_for_lane < 0.0)
            ? (lane_y - forward_terminal_y)
            : (forward_terminal_y - lane_y);
    };
    const double R_min_check = 1.0 / std::max(max_curvature, kEps);
    const double horiz_to_ref = std::abs(tgt.pre_dock_x - final_reference_x);
    const bool horiz_fits_two_turns = (horiz_to_ref >= 2.0 * R_min_check);
    const bool near_lane_can_arc_to_slot = horiz_fits_two_turns &&
        ((dock_dir_y_for_lane < 0.0)
            ? (forward_lane_y - tgt.dock_y() >= R_min_check)
            : (tgt.dock_y() - forward_lane_y >= R_min_check));
    const bool forward_lane_has_final_space =
        (lane_approach_gap(forward_lane_y) >= final_min_req_y &&
         horiz_fits_two_turns) ||
        (lane_approach_gap(far_lane_y)    >= final_min_req_y && horiz_fits_two_turns) ||
        near_lane_can_arc_to_slot;
    const bool auto_reverse_terminal =
        target_is_endpoint &&
        pp_.terminal_docking_mode != "forward" &&
        pp_.terminal_docking_mode != "reverse" &&
        !forward_lane_has_final_space;
    const bool terminal_reverse =
        target_is_endpoint &&
        (pp_.terminal_docking_mode == "reverse" || auto_reverse_terminal);
    if (debug_row1_target) {
        ROS_WARN("[planner][row1-debug] route=%s src=%d tgt=%d src_corr=%d tgt_corr=%d "
                 "target=(%.3f,%.3f th=%.1fdeg) mode=%s terminal_reverse=%d "
                 "near_gap=%.3f far_gap=%.3f horiz=%.3f min_x=%.3f",
                 route_mode_name, src.id, tgt.id, src_corr, tgt_corr,
                 tgt.pre_dock_x, tgt.pre_dock_y,
                 tgt.dock_theta * 180.0 / kPi,
                 pp_.terminal_docking_mode.c_str(),
                 terminal_reverse ? 1 : 0,
                 lane_approach_gap(forward_lane_y),
                 lane_approach_gap(far_lane_y),
                 horiz_to_ref, 2.0 * R_min_check);
    }
    if (terminal_reverse && pp_.terminal_docking_mode == "auto") {
        ROS_INFO("[planner] slot %d: forward terminal infeasible "
                 "(near_gap=%.3f far_gap=%.3f min_y=%.3f horiz=%.3f min_x=%.3f)",
                 tgt.id,
                 lane_approach_gap(forward_lane_y),
                 lane_approach_gap(far_lane_y),
                 final_min_req_y,
                 horiz_to_ref,
                 2.0 * R_min_check);
    }
    const double terminal_heading = tgt.dock_theta;
    const double terminal_reverse_out_heading = norm_angle(tgt.dock_theta + kPi);
    const double terminal_dir_y = std::sin(terminal_heading);

    double terminal_stop_y = tgt.pre_dock_y;
    double planned_goal_lane_y = corridor_lane_y(mp_, tgt_corr, final_hdir);
    if (terminal_reverse) {
        planned_goal_lane_y = corridor_lane_y(mp_, tgt_corr, final_hdir);
        terminal_stop_y = planned_goal_lane_y;
    } else {
        if (!target_is_endpoint && !single_row_terminal) {
            planned_goal_lane_y = (dock_dir_y_for_lane < 0.0)
                ? std::max(forward_lane_y, far_lane_y)
                : std::min(forward_lane_y, far_lane_y);
        }
        const bool lane_has_final_space =
            (dock_dir_y_for_lane < 0.0)
                ? (planned_goal_lane_y - forward_terminal_y >= final_min_req_y)
                : (forward_terminal_y - planned_goal_lane_y >= final_min_req_y);
        if (!lane_has_final_space) {
            const double R_min = 1.0 / std::max(max_curvature, kEps);
            const bool near_arc_ok = horiz_fits_two_turns &&
                ((dock_dir_y_for_lane < 0.0)
                    ? (planned_goal_lane_y - tgt.dock_y() >= R_min)
                    : (tgt.dock_y() - planned_goal_lane_y >= R_min));
            if (!near_arc_ok) {
                if (lane_approach_gap(far_lane_y) >= final_min_req_y) {
                    planned_goal_lane_y = far_lane_y;
                } else {
                    planned_goal_lane_y = far_lane_y;
                    ROS_WARN("[planner] slot %d: forward approach Y not on any lane "
                             "(near_gap=%.3f far_gap=%.3f); clamping to far lane",
                             tgt.id, lane_approach_gap(forward_lane_y),
                             lane_approach_gap(far_lane_y));
                }
            }
        }
    }

    auto choose_row1_bypass_x = [&](bool going_down, double current_x, double future_x) {
        const double left_x  = going_down ? row1_left_down_x  : row1_left_up_x;
        const double right_x = going_down ? row1_right_down_x : row1_right_up_x;
        if (!going_down && tgt_corr == 1) {
            const double min_terminal_run = 2.0 * R_min_check;
            const double left_gap = std::abs(target_x - left_x);
            const double right_gap = std::abs(target_x - right_x);
            if (right_gap < min_terminal_run && left_gap > right_gap) {
                return left_x;
            }
            if (left_gap < min_terminal_run && right_gap > left_gap) {
                return right_x;
            }
        }
        const double left_cost =
            std::abs(current_x - left_x) + std::abs(future_x - left_x);
        const double right_cost =
            std::abs(current_x - right_x) + std::abs(future_x - right_x);
        return (left_cost <= right_cost) ? left_x : right_x;
    };

    auto transition_x = [&](int from_corr, int to_corr, double current_x, double future_x) {
        const bool going_down = to_corr > from_corr;
        if ((from_corr == 1 && to_corr == 2) || (from_corr == 2 && to_corr == 1)) {
            return choose_row1_bypass_x(going_down, current_x, future_x);
        }
        if ((from_corr == 2 && to_corr == 3) || (from_corr == 3 && to_corr == 2)) {
            return spine_x_;
        }
        if ((from_corr == 3 && to_corr == 4) || (from_corr == 4 && to_corr == 3)) {
            if (going_down) {
                if (target_is_endpoint) {
                    return row3_down_x;
                }
                return row3_terminal_lane_x();
            }
            return row3_up_x;
        }
        return spine_x_;
    };

    auto next_reference_x = [&](int corridor, int target_corr, double current_x, double target_x) {
        if (corridor == target_corr) return final_reference_x;
        const int next_corr = corridor + ((target_corr > corridor) ? 1 : -1);
        return transition_x(corridor, next_corr, current_x, target_x);
    };
    auto current_transition_lane_y = [&](int current_corr, int next_corr) {
        const bool going_down = next_corr > current_corr;
        const HDir current_dir = going_down ? HDir::RIGHT : HDir::LEFT;
        return corridor_lane_y(mp_, current_corr, current_dir);
    };

    std::vector<Pt> polyline;
    Pt initial_reverse_end{src.pre_dock_x, src.pre_dock_y};
    TurnCurve initial_reverse_curve;
    Pt initial_reverse_curve_start{src.pre_dock_x, src.pre_dock_y};
    double initial_reverse_motion_heading = 0.0;
    bool force_first_transition_x = false;
    double first_transition_x = 0.0;

    int current_corr = src_corr;
    double current_x = src.dock_x();
    double current_y = src.dock_y();
    if (src.id >= 0) {
        double start_ref_x =
            (src_corr == tgt_corr)
                ? (terminal_reverse ? tgt.pre_dock_x : final_reference_x)
                : transition_x(src_corr,
                               src_corr + ((tgt_corr > src_corr) ? 1 : -1),
                               src.pre_dock_x, tgt.pre_dock_x);
        const bool same_corridor = src_corr == tgt_corr;
        const bool start_to_right = same_corridor
            ? (tgt.pre_dock_x >= src.pre_dock_x)
            : (start_ref_x >= src.pre_dock_x);
        if (!same_corridor &&
            ((src_corr == 1 && tgt_corr > 1) ||
             (src_corr == 2 && tgt_corr == 1))) {
            const bool going_down = tgt_corr > src_corr;
            if (src_corr == 1 && tgt_corr > 1) {
                first_transition_x =
                    start_to_right ? row1_right_down_x : row1_left_down_x;
            } else {
                first_transition_x =
                    start_to_right ? row1_right_up_x : row1_left_up_x;
            }
            force_first_transition_x = true;
            (void)going_down;
        }
        double start_lane_y =
            (src_corr == tgt_corr)
                ? planned_goal_lane_y
                : current_transition_lane_y(
                      src_corr, src_corr + ((tgt_corr > src_corr) ? 1 : -1));
        if (src_corr == tgt_corr && terminal_reverse) {
            const HDir same_corr_drive_dir =
                (tgt.pre_dock_x >= src.pre_dock_x) ? HDir::RIGHT : HDir::LEFT;
            start_lane_y = corridor_lane_y(mp_, src_corr, same_corr_drive_dir);
        }
        const double forward_heading = start_to_right ? 0.0 : kPi;
        initial_reverse_motion_heading = norm_angle(src.dock_theta + kPi);
        const Pt reverse_start_dir{std::cos(initial_reverse_motion_heading),
                                  std::sin(initial_reverse_motion_heading)};
        const double vertical_space =
            std::abs((start_lane_y - src.dock_y()) * reverse_start_dir.y);
        auto fit_initial_reverse = [&](const Pt& reverse_end_dir,
                                       double desired_end_x,
                                       bool minimize_tail) {
            const double signed_reverse_turn =
                std::atan2(cross(reverse_start_dir, reverse_end_dir),
                           dot(reverse_start_dir, reverse_end_dir));
            const double horizontal_space =
                (reverse_end_dir.x < 0.0)
                    ? (src.dock_x() - 0.04)
                    : (mp_.field_width - src.dock_x() - 0.04);
            const double max_tangent =
                0.98 * std::min(std::max(0.0, vertical_space),
                                std::max(0.0, horizontal_space));
            TurnCurve candidate =
                fit_clothoid_turn(signed_reverse_turn, max_curvature,
                                  sample_ds, std::max(sample_ds, max_tangent),
                                  sample_ds);
            if (candidate.pts.empty()) {
                candidate =
                    fit_clothoid_turn(signed_reverse_turn, max_curvature,
                                      steer_ramp_len,
                                      std::max(sample_ds, max_tangent),
                                      sample_ds);
            }
            if (candidate.pts.empty() || std::abs(reverse_start_dir.y) <= 0.5) {
                return false;
            }
            const Pt curve_delta =
                rotate_to_heading(candidate.pts.back(),
                                  initial_reverse_motion_heading);
            const double straight_len =
                (start_lane_y - src.dock_y() - curve_delta.y) /
                reverse_start_dir.y;
            const Pt curve_start{
                src.dock_x() + reverse_start_dir.x * straight_len,
                src.dock_y() + reverse_start_dir.y * straight_len};
            const Pt curve_end = curve_start + curve_delta;
            if (minimize_tail) {
                desired_end_x = curve_end.x;
            }
            const double post_curve_len =
                (desired_end_x - curve_end.x) * reverse_end_dir.x;
            if (straight_len >= -sample_ds &&
                post_curve_len >= -sample_ds &&
                curve_end.x >= 0.04 &&
                curve_end.x <= mp_.field_width - 0.04 &&
                desired_end_x >= 0.04 &&
                desired_end_x <= mp_.field_width - 0.04) {
                initial_reverse_curve = std::move(candidate);
                initial_reverse_curve_start = curve_start;
                initial_reverse_end = {desired_end_x, start_lane_y};
                return true;
            }
            return false;
        };

        const Pt direct_reverse_end_dir{
            std::cos(norm_angle(forward_heading + kPi)),
            std::sin(norm_angle(forward_heading + kPi))};
        double direct_reverse_end_x = src.pre_dock_x;
        if (same_corridor || src_corr != tgt_corr) {
            const double target_dx = std::abs(tgt.pre_dock_x - src.pre_dock_x);
            const bool near_source_target = target_dx < 0.55;
            const bool cross_corridor_exit = src_corr != tgt_corr;
            double reserve_x = std::max(
                final_min_req_x,
                (near_source_target || cross_corridor_exit) ? 0.36 : 0.12);
            direct_reverse_end_x = start_to_right
                ? std::max(0.04, src.pre_dock_x - reserve_x)
                : std::min(mp_.field_width - 0.04,
                           src.pre_dock_x + reserve_x);
        }
        bool direct_ok =
            fit_initial_reverse(direct_reverse_end_dir,
                                direct_reverse_end_x, false);
        if (!direct_ok) {
            if (!fit_initial_reverse(direct_reverse_end_dir,
                                     direct_reverse_end_x, true)) {
                initial_reverse_end = {src.pre_dock_x, start_lane_y};
            }
        }
        if (debug_row1_target) {
            ROS_WARN("[planner][row1-debug] initial_reverse tgt=%d same_corr=%d "
                     "start_to_right=%d start_lane_y=%.3f forward_heading=%.1fdeg "
                     "reverse_end_dir=(%.1fdeg) desired_end_x=%.3f "
                     "actual_end=(%.3f,%.3f) curve_start=(%.3f,%.3f) "
                     "curve_pts=%zu direct_ok=%d",
                     tgt.id, same_corridor ? 1 : 0, start_to_right ? 1 : 0,
                     start_lane_y, forward_heading * 180.0 / kPi,
                     std::atan2(direct_reverse_end_dir.y,
                                direct_reverse_end_dir.x) * 180.0 / kPi,
                     direct_reverse_end_x,
                     initial_reverse_end.x, initial_reverse_end.y,
                     initial_reverse_curve_start.x, initial_reverse_curve_start.y,
                     initial_reverse_curve.pts.size(),
                     direct_ok ? 1 : 0);
        }
        push_point(polyline, initial_reverse_end);
        current_x = initial_reverse_end.x;
        current_y = initial_reverse_end.y;
    } else {
        push_point(polyline, {src.dock_x(), src.dock_y()});
    }

    while (current_corr != tgt_corr) {
        const int next_corr = current_corr + ((tgt_corr > current_corr) ? 1 : -1);
        const double future_x = next_reference_x(next_corr, tgt_corr, current_x, tgt.pre_dock_x);
        double connector_x = transition_x(current_corr, next_corr, current_x, future_x);
        if (force_first_transition_x && current_corr == src_corr) {
            connector_x = first_transition_x;
            force_first_transition_x = false;
        }

        const bool going_down = next_corr > current_corr;
        double current_lane_y = current_transition_lane_y(current_corr, next_corr);

        const HDir next_dir = going_down ? HDir::LEFT : HDir::RIGHT;
        double next_lane_y =
            (next_corr == tgt_corr)
                ? planned_goal_lane_y
                : corridor_lane_y(mp_, next_corr, next_dir);
        push_point(polyline, {current_x, current_lane_y});
        push_point(polyline, {connector_x, current_lane_y});
        push_point(polyline, {connector_x, next_lane_y});

        current_corr = next_corr;
        current_x = connector_x;
        current_y = next_lane_y;
    }

    double goal_lane_y = planned_goal_lane_y;
    const double dock_dir_y = std::sin(tgt.dock_theta);
    if (current_corr == tgt_corr &&
        std::abs(current_y - goal_lane_y) < std::max(sample_ds, 0.02)) {
        goal_lane_y = current_y;
    }
    const bool current_y_has_turn_space =
        (dock_dir_y < 0.0)
            ? (current_y - tgt.pre_dock_y >= final_req_y)
            : (tgt.pre_dock_y - current_y >= final_req_y);
    if (src.id >= 0 && current_y_has_turn_space) {
        goal_lane_y = current_y;
    }
    TurnCurve terminal_reverse_curve;
    Pt terminal_reverse_start{tgt.pre_dock_x, goal_lane_y};
    Pt terminal_reverse_drive_start{tgt.pre_dock_x, goal_lane_y};
    Pt terminal_reverse_corner{tgt.pre_dock_x, goal_lane_y};
    Pt terminal_reverse_end{tgt.pre_dock_x, goal_lane_y};
    double terminal_reverse_motion_heading = 0.0;
    bool terminal_reverse_has_curve = false;
    if (terminal_reverse) {
        const double margin = 0.04;
        const Pt reverse_out{std::cos(terminal_reverse_out_heading),
                             std::sin(terminal_reverse_out_heading)};
        double best_max_tangent = -1.0;
        double best_route_cost = std::numeric_limits<double>::infinity();
        double best_lane_y = goal_lane_y;
        Pt best_reverse_in{0.0, 0.0};
        Pt best_reverse_start{tgt.pre_dock_x, goal_lane_y};
        Pt best_reverse_drive_start{tgt.pre_dock_x, goal_lane_y};
        Pt best_reverse_end{tgt.pre_dock_x, goal_lane_y};
        double best_available_out = 0.0;
        for (double reverse_in_x : {-1.0, 1.0}) {
            const double forward_dir_x = -reverse_in_x;
            const HDir forward_lane_dir =
                (forward_dir_x > 0.0) ? HDir::RIGHT : HDir::LEFT;
            const double candidate_lane_y =
                corridor_lane_y(mp_, tgt_corr, forward_lane_dir);
            const Pt candidate_corner{tgt.dock_x(), candidate_lane_y};
            const Pt reverse_in{reverse_in_x, 0.0};
            const double available_in = (reverse_in.x > 0.0)
                ? (candidate_corner.x - margin)
                : (mp_.field_width - candidate_corner.x - margin);
            const double available_out =
                std::abs((tgt.dock_y() - candidate_corner.y) * reverse_out.y);
            best_available_out = std::max(best_available_out, available_out);
            const double max_tangent =
                0.98 * std::min(std::max(0.0, available_in),
                                std::max(0.0, available_out));
            const double signed_reverse_turn =
                std::atan2(cross(reverse_in, reverse_out), dot(reverse_in, reverse_out));
            TurnCurve candidate =
                fit_clothoid_turn(signed_reverse_turn, max_curvature,
                                  sample_ds,
                                  std::max(sample_ds, max_tangent),
                                  sample_ds);
            if (candidate.pts.empty()) {
                candidate =
                    fit_clothoid_turn(signed_reverse_turn, max_curvature,
                                      steer_ramp_len,
                                      std::max(sample_ds, max_tangent),
                                      sample_ds);
            }
            if (candidate.pts.empty()) continue;

            const Pt candidate_start =
                candidate_corner - reverse_in * candidate.t_in;
            const Pt candidate_end =
                candidate_corner + reverse_out * candidate.t_out;
            const Pt candidate_drive_start = candidate_start;
            const double forward_stage_projection =
                (candidate_drive_start.x - current_x) * forward_dir_x;
            if (forward_stage_projection < sample_ds) continue;
            if (candidate_drive_start.x < margin ||
                candidate_drive_start.x > mp_.field_width - margin) {
                continue;
            }
            const double route_cost =
                std::abs(current_x - candidate_drive_start.x) +
                std::abs(current_y - candidate_lane_y) +
                dist(candidate_end, Pt{tgt.dock_x(), tgt.dock_y()});
            if (route_cost < best_route_cost - 1e-6 ||
                (std::abs(route_cost - best_route_cost) <= 1e-6 &&
                 max_tangent > best_max_tangent)) {
                best_max_tangent = max_tangent;
                best_route_cost = route_cost;
                best_lane_y = candidate_lane_y;
                best_available_out = available_out;
                best_reverse_in = reverse_in;
                best_reverse_start = candidate_start;
                best_reverse_drive_start = candidate_drive_start;
                best_reverse_end = candidate_end;
                terminal_reverse_curve = std::move(candidate);
            }
        }
        if (!terminal_reverse_curve.pts.empty()) {
            goal_lane_y = best_lane_y;
            if (std::abs(current_y - goal_lane_y) > 1e-5 &&
                std::abs(current_y - planned_goal_lane_y) < std::max(sample_ds, 0.02) &&
                !polyline.empty() &&
                std::hypot(polyline.back().x - current_x,
                           polyline.back().y - current_y) < 1e-4) {
                polyline.back().y = goal_lane_y;
                current_y = goal_lane_y;
            }
            terminal_reverse_corner = {tgt.dock_x(), goal_lane_y};
            terminal_reverse_start = best_reverse_start;
            terminal_reverse_drive_start = best_reverse_drive_start;
            terminal_reverse_end = best_reverse_end;
            terminal_reverse_motion_heading = std::atan2(best_reverse_in.y, best_reverse_in.x);
            terminal_reverse_has_curve = true;
        } else {
            ROS_WARN("[planner] reverse dock turn infeasible for slot %d; "
                     "available_out=%.3f",
                     tgt.id, best_available_out);
        }
    }
    if (terminal_reverse && !terminal_reverse_has_curve) {
        return {};
    }
    if (terminal_reverse && terminal_reverse_has_curve) {
        push_point(polyline, {current_x, goal_lane_y});
        push_point(polyline, terminal_reverse_drive_start);
    } else {
        push_point(polyline, {current_x, goal_lane_y});
        current_y = goal_lane_y;
        const double margin_x = 0.04;
        const double preferred_side = (target_x >= current_x) ? -1.0 : 1.0;
        double approach_x = target_x + preferred_side * final_min_req_x;
        if (approach_x < margin_x || approach_x > mp_.field_width - margin_x) {
            approach_x = target_x - preferred_side * final_min_req_x;
        }
        approach_x = std::max(margin_x, std::min(mp_.field_width - margin_x, approach_x));
        const bool approach_is_forward =
            (approach_x - current_x) * (target_x - approach_x) > 0.0;
        if (approach_is_forward &&
            std::abs(approach_x - target_x) > sample_ds &&
            std::abs(current_x - target_x) < final_min_req_x) {
            push_point(polyline, {approach_x, goal_lane_y});
        }
        push_point(polyline, {tgt.pre_dock_x, goal_lane_y});
        push_point(polyline, {tgt.pre_dock_x, terminal_stop_y});
    }
    if (!terminal_reverse || !terminal_reverse_has_curve) {
        push_point(polyline, {tgt.dock_x() - mp_.rear_axle_to_center * std::cos(terminal_heading),
                              tgt.dock_y() - mp_.rear_axle_to_center * std::sin(terminal_heading)});
    }

    const std::vector<Pt> simplified = simplify_polyline(polyline);
    if (simplified.empty()) return {};
    record_debug_layer(info, DebugPathLayerType::SKELETON,
                       "route_skeleton", simplified);
    if (debug_row1_target) {
        ROS_WARN("[planner][row1-debug] skeleton tgt=%d points=%zu "
                 "terminal_reverse=%d goal_lane_y=%.3f terminal_stop_y=%.3f "
                 "final_ref_x=%.3f",
                 tgt.id, simplified.size(), terminal_reverse ? 1 : 0,
                 goal_lane_y, terminal_stop_y, final_reference_x);
        for (size_t dbg_i = 0; dbg_i < simplified.size(); ++dbg_i) {
            ROS_WARN("[planner][row1-debug] skeleton[%zu]=(%.3f,%.3f)",
                     dbg_i, simplified[dbg_i].x, simplified[dbg_i].y);
        }
    }
    if (simplified.size() == 1) {
        return {{simplified.front().x, simplified.front().y,
                 norm_angle(src.dock_theta + kPi), WpType::FORWARD}};
    }

    auto append_terminal_reverse = [&](RoughPath& path) {
        if (!terminal_reverse || path.empty() || !terminal_reverse_has_curve) return;
        const double start_heading = norm_angle(terminal_reverse_motion_heading + kPi);
        if (std::hypot(path.back().x - terminal_reverse_drive_start.x,
                       path.back().y - terminal_reverse_drive_start.y) > sample_ds) {
            const double approach_heading = std::atan2(
                terminal_reverse_drive_start.y - path.back().y,
                terminal_reverse_drive_start.x - path.back().x);
            path.push_back({terminal_reverse_drive_start.x, terminal_reverse_drive_start.y,
                            approach_heading, WpType::FORWARD});
        }
        path.push_back({terminal_reverse_drive_start.x, terminal_reverse_drive_start.y,
                        start_heading, WpType::REVERSE});

        const double lead_len = dist(terminal_reverse_drive_start,
                                     terminal_reverse_start);
        const int lead_steps =
            static_cast<int>(std::ceil(lead_len / sample_ds));
        if (lead_steps > 0) {
            for (int k = 1; k <= lead_steps; ++k) {
                const double u = static_cast<double>(k) /
                                 static_cast<double>(lead_steps);
                path.push_back({
                    terminal_reverse_drive_start.x +
                        (terminal_reverse_start.x - terminal_reverse_drive_start.x) * u,
                    terminal_reverse_drive_start.y +
                        (terminal_reverse_start.y - terminal_reverse_drive_start.y) * u,
                    start_heading,
                    WpType::REVERSE});
            }
        }

        const double curve_heading = terminal_reverse_motion_heading;
        for (size_t j = 1; j < terminal_reverse_curve.pts.size(); ++j) {
            const Pt rotated = rotate_to_heading(terminal_reverse_curve.pts[j],
                                                 curve_heading);
            const Pt p = terminal_reverse_start + rotated;
            const double motion_theta =
                norm_angle(curve_heading + terminal_reverse_curve.headings[j]);
            path.push_back({p.x, p.y, norm_angle(motion_theta + kPi),
                            WpType::REVERSE});
        }

        const Pt a = terminal_reverse_end;
        const double d_axle = mp_.rear_axle_to_center;
        const Pt b{tgt.dock_x() - d_axle * std::cos(terminal_heading),
                   tgt.dock_y() - d_axle * std::sin(terminal_heading)};
        const double reverse_len = dist(a, b);
        const int steps =
            std::max(1, static_cast<int>(std::ceil(reverse_len / sample_ds)));
        for (int k = 1; k <= steps; ++k) {
            const double u = static_cast<double>(k) / static_cast<double>(steps);
            path.push_back({a.x + (b.x - a.x) * u,
                            a.y + (b.y - a.y) * u,
                            terminal_heading,
                            WpType::REVERSE});
        }
    };

    auto prepend_initial_reverse = [&](RoughPath& path) {
        if (src.id < 0 || path.empty()) return;
        const double d_axle = mp_.rear_axle_to_center;
        const Pt b = initial_reverse_end;
        auto build_pts = [&](const Pt& a) {
            const double reverse_len = dist(a, b);
            const int steps =
                std::max(1, static_cast<int>(std::ceil(reverse_len / sample_ds)));
            const int straight_steps = initial_reverse_curve.pts.empty()
                ? steps
                : std::max(1, static_cast<int>(
                        std::ceil(dist(a, initial_reverse_curve_start) / sample_ds)));
            std::vector<Pt> pts;
            pts.reserve(static_cast<size_t>(steps + 1));
            for (int k = 0; k <= straight_steps; ++k) {
                const double denom = static_cast<double>(std::max(1, straight_steps));
                const double v = static_cast<double>(k) / denom;
                const Pt straight_end = initial_reverse_curve.pts.empty()
                    ? b
                    : initial_reverse_curve_start;
                pts.push_back({a.x + (straight_end.x - a.x) * v,
                               a.y + (straight_end.y - a.y) * v});
            }
            if (!initial_reverse_curve.pts.empty()) {
                for (size_t j = 1; j < initial_reverse_curve.pts.size(); ++j) {
                    const Pt rotated =
                        rotate_to_heading(initial_reverse_curve.pts[j],
                                          initial_reverse_motion_heading);
                    pts.push_back(initial_reverse_curve_start + rotated);
                }
                const Pt curve_end = pts.back();
                const double tail_len = dist(curve_end, b);
                const int tail_steps =
                    static_cast<int>(std::ceil(tail_len / sample_ds));
                for (int k = 1; k <= tail_steps; ++k) {
                    const double u =
                        static_cast<double>(k) / static_cast<double>(tail_steps);
                    pts.push_back({curve_end.x + (b.x - curve_end.x) * u,
                                   curve_end.y + (b.y - curve_end.y) * u});
                }
            }
            return pts;
        };

        const Pt a_rev{src.dock_x() - d_axle * std::cos(src.dock_theta),
                       src.dock_y() - d_axle * std::sin(src.dock_theta)};

        std::vector<Pt> pts = build_pts(a_rev);
        const WpType type = WpType::REVERSE;

        RoughPath prefix;
        prefix.reserve(pts.size() + path.size());
        if (!initial_reverse_curve.pts.empty()) {
            const double start_body =
                norm_angle(initial_reverse_motion_heading + kPi);
            const Pt reverse_dir{std::cos(initial_reverse_motion_heading),
                                 std::sin(initial_reverse_motion_heading)};
            const Pt lead_delta = initial_reverse_curve_start - a_rev;
            const double lead_projection = dot(lead_delta, reverse_dir);
            if (lead_projection > sample_ds * 0.25) {
                const double straight_len = dist(a_rev, initial_reverse_curve_start);
                const int straight_steps =
                    std::max(1, static_cast<int>(std::ceil(straight_len / sample_ds)));
                for (int k = 0; k <= straight_steps; ++k) {
                    const double u = static_cast<double>(k) /
                                     static_cast<double>(straight_steps);
                    prefix.push_back({
                        a_rev.x + (initial_reverse_curve_start.x - a_rev.x) * u,
                        a_rev.y + (initial_reverse_curve_start.y - a_rev.y) * u,
                        start_body,
                        WpType::REVERSE});
                }
            } else {
                prefix.push_back({initial_reverse_curve_start.x,
                                  initial_reverse_curve_start.y,
                                  start_body,
                                  WpType::REVERSE});
            }

            for (size_t j = 1; j < initial_reverse_curve.pts.size(); ++j) {
                const Pt rotated =
                    rotate_to_heading(initial_reverse_curve.pts[j],
                                      initial_reverse_motion_heading);
                const Pt p = initial_reverse_curve_start + rotated;
                const double motion =
                    norm_angle(initial_reverse_motion_heading +
                               initial_reverse_curve.headings[j]);
                prefix.push_back({p.x, p.y, norm_angle(motion + kPi),
                                  WpType::REVERSE});
            }

            const Pt curve_end{prefix.back().x, prefix.back().y};
            const double end_motion =
                norm_angle(initial_reverse_motion_heading +
                           initial_reverse_curve.headings.back());
            const double end_body = norm_angle(end_motion + kPi);
            const double tail_len = dist(curve_end, b);
            const int tail_steps =
                static_cast<int>(std::ceil(tail_len / sample_ds));
            for (int k = 1; k <= tail_steps; ++k) {
                const double u = static_cast<double>(k) /
                                 static_cast<double>(tail_steps);
                prefix.push_back({curve_end.x + (b.x - curve_end.x) * u,
                                  curve_end.y + (b.y - curve_end.y) * u,
                                  end_body,
                                  WpType::REVERSE});
            }

            if (!prefix.empty()) {
                prefix.back().x = b.x;
                prefix.back().y = b.y;
                prefix.back().theta = path.front().theta;
                prefix.back().type = WpType::REVERSE;
                path.front().x = b.x;
                path.front().y = b.y;
                path.front().theta = prefix.back().theta;
                path.front().type = WpType::FORWARD;
            }
            prefix.insert(prefix.end(), path.begin(), path.end());
            path.swap(prefix);
            return;
        }

        for (size_t k = 0; k < pts.size(); ++k) {
            const Pt nxt = (k + 1 < pts.size())
                ? pts[k + 1]
                : Pt{path.front().x, path.front().y};
            double motion;
            if (dist(pts[k], nxt) > 1e-9) {
                motion = std::atan2(nxt.y - pts[k].y, nxt.x - pts[k].x);
            } else if (k > 0) {
                motion = norm_angle(prefix.back().theta + kPi);
            } else {
                motion = norm_angle(src.dock_theta + kPi);
            }
            const double body = norm_angle(motion + kPi);
            prefix.push_back({pts[k].x, pts[k].y, norm_angle(body), type});
        }
        if (!prefix.empty()) {
            prefix.back().x = b.x;
            prefix.back().y = b.y;
            prefix.back().theta = path.front().theta;
            prefix.back().type = WpType::REVERSE;
            path.front().x = b.x;
            path.front().y = b.y;
            path.front().theta = prefix.back().theta;
            path.front().type = WpType::FORWARD;
        }
        prefix.insert(prefix.end(), path.begin(), path.end());
        path.swap(prefix);
    };

    if (pp_.turn_model == "arc") {
        ROS_WARN_ONCE("[planner] turn_model=arc: curvature-continuous paths disabled. "
                      "Paths use constant-radius arcs (kinematic discontinuity at turn entry/exit).");
        if (info != nullptr) {
            info->used_arc_fallback = true;
        }
        RoughPath path = build_arc_path(simplified, pp_, src, max_curvature, sample_ds);
        record_debug_layer_from_path(info, DebugPathLayerType::ARC_FALLBACK,
                                     "arc_fallback", path);
        prepend_initial_reverse(path);
        append_terminal_reverse(path);
        warn_if_reverse_segments(path);
        return path;
    }

    const size_t n = simplified.size();
    std::vector<double> seg_len(n - 1, 0.0);
    for (size_t s = 0; s + 1 < n; ++s) {
        seg_len[s] = dist(simplified[s], simplified[s + 1]);
    }

    auto lane_shift_clear = [&](const Pt& b, const Pt& c, const Pt& direction,
                                double lead_in, double lead_out,
                                bool allow_terminal_shelf) {
        const Pt u = normalize(direction);
        const Pt nrm = left_normal(u);
        const double lateral = dot(c - b, nrm);
        const double length = std::max(lead_in + lead_out, sample_ds);
        const Pt start = b - u * lead_in;
        const int steps = std::max(8, static_cast<int>(std::ceil(length / sample_ds)));
        for (int k = 0; k <= steps; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(steps);
            const Pt p = start + u * (length * t) + nrm * (lateral * quintic_blend(t));
            if (point_in_shelf(mp_, p) && !allow_terminal_shelf) return false;
        }
        return true;
    };

    auto turn_curve_clear = [&](const TurnCurve& curve, const Pt& corner,
                                const Pt& u_in, double heading,
                                bool allow_target_shelf) {
        const Pt start = corner - u_in * curve.t_in;
        for (const Pt& local : curve.pts) {
            const Pt p = start + rotate_to_heading(local, heading);
            if (point_in_shelf(mp_, p) && !allow_target_shelf) return false;
        }
        return true;
    };

    std::vector<bool> lane_shift_start(n, false);
    std::vector<double> lane_shift_lead_in(n, 0.0);
    std::vector<double> lane_shift_lead_out(n, 0.0);
    std::vector<bool> suppress_turn(n, false);
    for (size_t j = 1; j + 2 < n; ++j) {
        if (suppress_turn[j] || suppress_turn[j + 1]) continue;
        if (!is_short_parallel_shift(simplified[j - 1], simplified[j],
                                     simplified[j + 1], simplified[j + 2], 1.20)) {
            continue;
        }
        const bool terminal_lane_shift =
            !target_is_endpoint && (j + 2 == n - 1);
        const Pt u = normalize(simplified[j] - simplified[j - 1]);
        const double lateral = std::abs(dot(simplified[j + 1] - simplified[j],
                                            left_normal(u)));
        double min_total =
            std::max(2.0 * sample_ds,
                     std::sqrt(6.0 * lateral / std::max(max_curvature, kEps)));
        const double avail_in = 0.98 * seg_len[j - 1];
        const double avail_out = 0.98 * seg_len[j + 1];
        if (min_total > avail_in + avail_out) {
            if (!terminal_lane_shift) {
                if (debug_row1_target) {
                    ROS_WARN("[planner][row1-debug] lane_shift too short tgt=%d j=%zu "
                             "terminal=0 lateral=%.3f need=%.3f avail=%.3f",
                             tgt.id, j, lateral, min_total, avail_in + avail_out);
                }
                continue;
            }
            min_total = avail_in + avail_out;
        }
        double lead_in = std::min(avail_in, min_total * 0.5);
        double lead_out = min_total - lead_in;
        if (lead_out > avail_out) {
            lead_out = avail_out;
            lead_in = min_total - lead_out;
        }
        if (j > 1) {
            const Pt prev_u = normalize(simplified[j - 1] - simplified[j - 2]);
            const Pt in_u = normalize(simplified[j] - simplified[j - 1]);
            const bool prev_has_turn =
                std::abs(cross(prev_u, in_u)) >= 1e-4 &&
                std::abs(dot(prev_u, in_u)) <= 0.99;
            if (prev_has_turn) {
                const double turn_reserve =
                    std::min(0.45 * seg_len[j - 1],
                             std::max(final_min_req_x, sample_ds));
                const double max_lead_in =
                    std::max(sample_ds, seg_len[j - 1] - turn_reserve - sample_ds);
                if (lead_in > max_lead_in) {
                    const double extra = lead_in - max_lead_in;
                    lead_in = max_lead_in;
                    lead_out = std::min(avail_out, lead_out + extra);
                }
            }
        }
        if (!lane_shift_clear(simplified[j], simplified[j + 1],
                              simplified[j] - simplified[j - 1],
                              lead_in, lead_out,
                              terminal_lane_shift)) {
            if (debug_row1_target) {
                ROS_WARN("[planner][row1-debug] lane_shift rejected tgt=%d j=%zu "
                         "terminal=%d lateral=%.3f lead_in=%.3f lead_out=%.3f",
                         tgt.id, j, terminal_lane_shift ? 1 : 0,
                         lateral, lead_in, lead_out);
            }
            continue;
        }
        if (debug_row1_target) {
            ROS_WARN("[planner][row1-debug] lane_shift accepted tgt=%d j=%zu "
                     "terminal=%d lateral=%.3f lead_in=%.3f lead_out=%.3f",
                     tgt.id, j, terminal_lane_shift ? 1 : 0,
                     lateral, lead_in, lead_out);
        }
        lane_shift_start[j] = true;
        lane_shift_lead_in[j] = lead_in;
        lane_shift_lead_out[j] = lead_out;
        suppress_turn[j] = true;
        suppress_turn[j + 1] = true;
    }

    std::vector<bool> active_turn(n, false);
    std::vector<double> signed_turns(n, 0.0);
    std::vector<double> turn_limits(n, 0.0);
    for (size_t j = 1; j + 1 < n; ++j) {
        if (suppress_turn[j] || seg_len[j - 1] < kEps || seg_len[j] < kEps) continue;
        const Pt u1 = normalize(simplified[j] - simplified[j - 1]);
        const Pt u2 = normalize(simplified[j + 1] - simplified[j]);
        const double bend = cross(u1, u2);
        const double cos_angle = dot(u1, u2);
        if (std::abs(bend) < 1e-4 || std::abs(cos_angle) > 0.99) continue;

        active_turn[j] = true;
        signed_turns[j] = std::atan2(bend, cos_angle);
        turn_limits[j] = 0.90 * std::min(seg_len[j - 1], seg_len[j]);
        if (j + 1 == n - 1) {
            turn_limits[j] = std::min(0.90 * seg_len[j - 1], 0.99 * seg_len[j]);
        } else if (j + 2 == n - 1) {
            if (!terminal_reverse) {
                const double R_min_reserve = 1.005 / std::max(max_curvature, kEps);
                turn_limits[j] = std::min(0.90 * seg_len[j - 1],
                                          std::max(sample_ds,
                                                   seg_len[j] - R_min_reserve - sample_ds));
            } else {
                turn_limits[j] = std::min(0.90 * seg_len[j - 1], 0.98 * seg_len[j]);
            }
        }
    }

    auto fit_clear_turn = [&](size_t j, double max_tangent) {
        const Pt u1 = normalize(simplified[j] - simplified[j - 1]);
        const double heading = std::atan2(u1.y, u1.x);
        const bool allow_target_shelf = (j + 1 == n - 1);
        for (int k = 0; k < 32; ++k) {
            const double ratio = static_cast<double>(k) / 31.0;
            const double ramp =
                steer_ramp_len + (sample_ds - steer_ramp_len) * ratio;
            TurnCurve candidate =
                build_clothoid_turn(signed_turns[j], max_curvature,
                                    std::max(sample_ds, ramp), sample_ds);
            if (candidate.pts.empty()) continue;
            if (std::max(candidate.t_in, candidate.t_out) > max_tangent) continue;
            if (!turn_curve_clear(candidate, simplified[j], u1, heading,
                                  allow_target_shelf)) continue;
            return candidate;
        }
        return TurnCurve{};
    };

    std::vector<PlannedTurn> planned(n);
    for (int iter = 0; iter < 12; ++iter) {
        for (size_t j = 1; j + 1 < n; ++j) {
            if (!active_turn[j]) continue;
            TurnCurve curve = fit_clear_turn(j, turn_limits[j]);
            planned[j].active = !curve.pts.empty();
            planned[j].curve = std::move(curve);
        }

        bool changed = false;
        for (size_t s = 0; s + 1 < n; ++s) {
            double used = sample_ds;
            if (planned[s].active) used += planned[s].curve.t_out;
            if (planned[s + 1].active) used += planned[s + 1].curve.t_in;
            if (s + 1 < n && lane_shift_start[s + 1]) {
                used += lane_shift_lead_in[s + 1];
            }
            if (s > 0 && lane_shift_start[s - 1]) {
                used += lane_shift_lead_out[s - 1];
            }
            if (used <= seg_len[s]) continue;

            const double scale =
                std::max(0.15, (seg_len[s] - sample_ds) / std::max(used - sample_ds, kEps));
            if (planned[s].active) {
                turn_limits[s] = std::max(sample_ds, turn_limits[s] * scale);
                changed = true;
            }
            if (planned[s + 1].active) {
                turn_limits[s + 1] = std::max(sample_ds, turn_limits[s + 1] * scale);
                changed = true;
            }
        }
        if (!changed) break;
    }

    for (size_t j = 1; j + 1 < n; ++j) {
        if (!planned[j].active) continue;
        const Pt u1 = normalize(simplified[j] - simplified[j - 1]);
        planned[j].heading = std::atan2(u1.y, u1.x);
        planned[j].start = simplified[j] - u1 * planned[j].curve.t_in;
    }
    std::vector<size_t> infeasible_turns;
    for (size_t j = 1; j + 1 < n; ++j) {
        if (active_turn[j] && !planned[j].active) {
            infeasible_turns.push_back(j);
        }
    }
    if (!infeasible_turns.empty() &&
        pp_.terminal_docking_mode == "auto" &&
        !terminal_reverse &&
        target_is_endpoint) {
        PlannerParam reverse_pp = pp_;
        reverse_pp.terminal_docking_mode = "reverse";
        PathGenerator reverse_gen(mp_, reverse_pp);
        PathGenerationInfo reverse_info;
        RoughPath reverse_path = reverse_gen.generate(src, tgt, &reverse_info);
        if (!reverse_path.empty()) {
            if (info != nullptr) {
                info->used_arc_fallback = reverse_info.used_arc_fallback;
                info->debug_layers = reverse_info.debug_layers;
            }
            return reverse_path;
        }
    }
    if (!infeasible_turns.empty()) {
        for (size_t j : infeasible_turns) {
            if (debug_row1_target) {
                ROS_WARN("[planner][row1-debug] clothoid infeasible tgt=%d "
                         "j=%zu p=(%.3f, %.3f) prev=(%.3f, %.3f) "
                         "next=(%.3f, %.3f) prev_len=%.3f next_len=%.3f "
                         "limit=%.3f",
                         tgt.id, j, simplified[j].x, simplified[j].y,
                         simplified[j - 1].x, simplified[j - 1].y,
                         simplified[j + 1].x, simplified[j + 1].y,
                         seg_len[j - 1], seg_len[j], turn_limits[j]);
            } else {
                ROS_WARN_THROTTLE(5.0,
                         "[planner] slot %d: clothoid turn infeasible at skeleton point %zu; "
                         "p=(%.3f, %.3f), prev_len=%.3f, next_len=%.3f, "
                         "limit=%.3f, using local arc at this point",
                         tgt.id, j, simplified[j].x, simplified[j].y,
                         seg_len[j - 1], seg_len[j], turn_limits[j]);
            }
        }
        ROS_WARN_THROTTLE(5.0,
                 "[planner] slot %d: using local arc fallback only at infeasible turns; "
                 "accepted lane shifts and clothoids are preserved",
                 tgt.id);
        if (info != nullptr) {
            info->used_arc_fallback = true;
        }
    }

    std::vector<Sample> dense;
    const Pt first_dir = normalize(simplified[1] - simplified[0]);
    push_sample(dense, simplified.front(), std::atan2(first_dir.y, first_dir.x));

    size_t i = 1;
    while (i + 1 < n) {
        const Pt& b = simplified[i];
        const Pt& c = simplified[i + 1];

        if (lane_shift_start[i]) {
            const size_t debug_begin = dense.size();
            append_lane_shift(dense, b, c, b - simplified[i - 1],
                              lane_shift_lead_in[i], lane_shift_lead_out[i],
                              sample_ds);
            record_debug_layer_from_samples(info, DebugPathLayerType::LANE_SHIFT,
                                            "lane_shift", dense, debug_begin);
            i += 2;
            continue;
        }

        if (!planned[i].active) {
            if (active_turn[i]) {
                const size_t debug_begin = dense.size();
                const Pt& a = simplified[i - 1];
                const double len1 = dist(a, b);
                const double len2 = dist(b, c);
                const Pt u1 = normalize(b - a);
                const Pt u2 = normalize(c - b);
                const double bend = cross(u1, u2);
                const double cos_angle = dot(u1, u2);
                const double min_radius = 1.0 / std::max(max_curvature, kEps);
                double max_local_radius = 0.45 * std::min(len1, len2);
                const double neighbor_gap = std::max(sample_ds, 0.015);
                if (i > 0 && planned[i - 1].active) {
                    max_local_radius = std::min(
                        max_local_radius,
                        len1 - planned[i - 1].curve.t_out - neighbor_gap);
                }
                if (i + 1 < planned.size() && planned[i + 1].active) {
                    max_local_radius = std::min(
                        max_local_radius,
                        len2 - planned[i + 1].curve.t_in - neighbor_gap);
                }
                const double local_radius =
                    std::min(min_radius, max_local_radius);
                if (len1 >= kEps && len2 >= kEps &&
                    std::abs(bend) >= 1e-4 &&
                    std::abs(cos_angle) <= 0.99 &&
                    local_radius >= 1e-3) {
                    const Pt t1 = b - u1 * local_radius;
                    const Pt t2 = b + u2 * local_radius;
                    push_sample(dense, t1, std::atan2(u1.y, u1.x));

                    const Pt normal =
                        (bend > 0.0) ? left_normal(u1) : right_normal(u1);
                    const Pt center = t1 + normal * local_radius;
                    double a0 = std::atan2(t1.y - center.y,
                                           t1.x - center.x);
                    double a1 = std::atan2(t2.y - center.y,
                                           t2.x - center.x);
                    if (bend > 0.0) {
                        while (a1 <= a0) a1 += 2.0 * kPi;
                    } else {
                        while (a1 >= a0) a1 -= 2.0 * kPi;
                    }
                    const double arc_angle = std::abs(a1 - a0);
                    const double arc_len = arc_angle * local_radius;
                    const int steps = std::max({
                        2,
                        static_cast<int>(std::ceil(arc_len / sample_ds)),
                        static_cast<int>(std::ceil(arc_angle / 0.04))});
                    for (int s = 1; s < steps; ++s) {
                        const double ratio =
                            static_cast<double>(s) / static_cast<double>(steps);
                        const double ang = a0 + (a1 - a0) * ratio;
                        const Pt p{center.x + local_radius * std::cos(ang),
                                   center.y + local_radius * std::sin(ang)};
                        const double theta = norm_angle(
                            ang + ((bend > 0.0) ? kPi * 0.5 : -kPi * 0.5));
                        push_sample(dense, p, theta);
                    }
                    push_sample(dense, t2, std::atan2(u2.y, u2.x));
                    record_debug_layer_from_samples(
                        info, DebugPathLayerType::ARC_FALLBACK,
                        "local_arc_fallback", dense, debug_begin);
                    if (debug_row1_target) {
                        ROS_WARN("[planner][row1-debug] local arc fallback "
                                 "tgt=%d j=%zu radius=%.3f max_radius=%.3f "
                                 "pts=%zu",
                                 tgt.id, i, local_radius,
                                 max_local_radius,
                                 dense.size() - debug_begin);
                    }
                    ++i;
                    continue;
                }
            }
            const Pt u2 = normalize(c - b);
            push_sample(dense, b, std::atan2(u2.y, u2.x));
            ++i;
            continue;
        }

        const Pt progress_dir{std::cos(planned[i].heading),
                              std::sin(planned[i].heading)};
        auto push_progress_sample = [&](const Pt& p, double theta) {
            if (!dense.empty() &&
                dot(p - dense.back().p, progress_dir) < -1e-4) {
                return;
            }
            push_sample(dense, p, theta);
        };
        const size_t debug_begin = dense.size();
        push_progress_sample(planned[i].start, planned[i].heading);
        for (size_t j = 0; j < planned[i].curve.pts.size(); ++j) {
            const Pt rotated = rotate_to_heading(planned[i].curve.pts[j],
                                                 planned[i].heading);
            push_progress_sample(
                planned[i].start + rotated,
                norm_angle(planned[i].heading + planned[i].curve.headings[j]));
        }
        record_debug_layer_from_samples(info, DebugPathLayerType::CLOTHOID,
                                        "clothoid", dense, debug_begin);
        ++i;
    }

    const Pt last_dir = normalize(simplified.back() - simplified[simplified.size() - 2]);
    push_sample(dense, simplified.back(), std::atan2(last_dir.y, last_dir.x));

    auto remove_micro_reversals = [&]() {
        std::vector<Sample> cleaned;
        cleaned.reserve(dense.size());
        const double max_micro_len = std::max(2.5 * sample_ds, 0.025);
        for (const Sample& s : dense) {
            cleaned.push_back(s);
            while (cleaned.size() >= 3) {
                const size_t k = cleaned.size() - 1;
                const Pt a = cleaned[k - 2].p;
                const Pt b = cleaned[k - 1].p;
                const Pt c = cleaned[k].p;
                const Pt ab = b - a;
                const Pt bc = c - b;
                const double lab = dist(a, b);
                const double lbc = dist(b, c);
                if (lab < kEps || lbc < kEps) {
                    cleaned.erase(cleaned.end() - 2);
                    continue;
                }
                const double cos_angle = dot(ab, bc) / (lab * lbc);
                const bool micro_reversal =
                    cos_angle < -0.98 &&
                    std::min(lab, lbc) <= max_micro_len &&
                    dist(a, c) <= std::max(lab, lbc) + sample_ds;
                if (!micro_reversal) break;
                if (debug_row1_target) {
                    ROS_WARN("[planner][row1-debug] remove micro reversal tgt=%d "
                             "at p=(%.3f,%.3f) len=(%.3f,%.3f)",
                             tgt.id, b.x, b.y, lab, lbc);
                }
                cleaned.erase(cleaned.end() - 2);
            }
        }
        dense.swap(cleaned);
    };
    remove_micro_reversals();

    RoughPath path;
    path.reserve(dense.size());
    for (const Sample& s : dense) {
        path.push_back({s.p.x, s.p.y, norm_angle(s.theta), WpType::FORWARD});
    }

    prepend_initial_reverse(path);
    append_terminal_reverse(path);
    warn_if_reverse_segments(path);
    return path;
}
