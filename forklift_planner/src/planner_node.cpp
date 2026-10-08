#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_map/forklift_map.h"
#include "forklift_planner/planner_param.h"
#include "forklift_planner/path_generator.h"

// ─── helpers ──────────────────────────────────────────────────────────────────
static std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.f) {
    std_msgs::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c;
}
static geometry_msgs::Point pt3(double x, double y, double z = 0.055) {
    geometry_msgs::Point p; p.x=x; p.y=y; p.z=z; return p;
}

static bool in_box(double x, double y, const ShelfBlock& b, double eps = 1e-6) {
    return x >= b.x - eps && x <= b.x_max() + eps &&
           y >= b.y - eps && y <= b.y_max() + eps;
}

static double sqr_dist(double x0, double y0, double x1, double y1) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    return dx * dx + dy * dy;
}

static double norm_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// ─── VehicleSim ───────────────────────────────────────────────────────────────
// Interpolates through a forward-only dense path at constant speed.
class VehicleSim {
public:
    VehicleSim(const PlannerParam& pp)
        : speed_(pp.vehicle_speed) {}

    void set_path(const RoughPath& path) {
        path_ = path;
        idx_ = (path_.size() >= 2) ? 1 : 0;
        seg_s_ = 0.0;
        active_ = path_.size() >= 2;
    }

    bool empty() const { return !active_; }

    // Advance by dt seconds.  Returns true when the full path is completed.
    bool update(double dt) {
        if (!active_ || path_.size() < 2) return false;

        double remain = speed_ * dt;
        while (remain > 1e-9 && idx_ < static_cast<int>(path_.size())) {
            const double seg_len = segment_length(idx_);
            const double seg_remain = seg_len - seg_s_;
            if (remain < seg_remain) {
                seg_s_ += remain;
                remain = 0.0;
            } else {
                remain -= seg_remain;
                ++idx_;
                seg_s_ = 0.0;
            }
        }

        if (idx_ >= static_cast<int>(path_.size())) {
            idx_ = static_cast<int>(path_.size()) - 1;
            active_ = false;
            return true;
        }
        return false;
    }

    // Current interpolated pose
    void pose(double& x, double& y, double& theta) const {
        if (path_.empty()) { x=0; y=0; theta=0; return; }

        if (!active_ || idx_ <= 0 || idx_ >= static_cast<int>(path_.size())) {
            const RoughWp& last = path_.back();
            x = last.x; y = last.y; theta = last.theta;
            return;
        }

        if (idx_ == 1 && seg_s_ <= 1e-9) {
            x = path_[0].x; y = path_[0].y; theta = path_[0].theta;
            return;
        }

        const RoughWp& from = path_[idx_ - 1];
        const RoughWp& to   = path_[idx_];
        const double seg_len = segment_length(idx_);
        const double alpha = (seg_len > 1e-9) ? std::min(seg_s_ / seg_len, 1.0) : 1.0;

        x = from.x + alpha * (to.x - from.x);
        y = from.y + alpha * (to.y - from.y);
        theta = norm_angle(from.theta + alpha * norm_angle(to.theta - from.theta));
    }

    const RoughPath& path() const { return path_; }
    int next_index() const { return active_ ? idx_ : static_cast<int>(path_.size()); }

private:
    double segment_length(int to_idx) const {
        if (to_idx <= 0 || to_idx >= (int)path_.size()) return 0.0;
        const RoughWp& from = path_[to_idx - 1];
        const RoughWp& to   = path_[to_idx];
        return std::hypot(to.x - from.x, to.y - from.y);
    }

    RoughPath path_;
    int    idx_    = 0;
    double seg_s_  = 0.0;
    double speed_;
    bool   active_ = false;
};

// ─── PlannerNode ──────────────────────────────────────────────────────────────

enum class State { MOVING, DONE };

class PlannerNode {
public:
    PlannerNode() : nh_("~") {
        MapParam    mp = MapParam::fromROSParam(nh_);
        PlannerParam pp = PlannerParam::fromROSParam(nh_);

        frame_   = pp.frame_id;
        pc_r_ = (float)pp.path_color_r;
        pc_g_ = (float)pp.path_color_g;
        pc_b_ = (float)pp.path_color_b;

        map_  = std::make_unique<ForkliftMap>(mp);
        gen_  = std::make_unique<PathGenerator>(mp, pp);
        vsim_ = std::make_unique<VehicleSim>(pp);

        ROS_INFO("[planner] seed=%d  slots=%zu  speed=%.2f m/s",
                 pp.random_seed, map_->slots().size(), pp.vehicle_speed);

        run_path_diagnostics();

        // Publisher (not latched: pose changes each frame)
        sim_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                                  &PlannerNode::on_goal, this);
        legacy_goal_sub_ = nh_.subscribe("/goal", 1,
                                         &PlannerNode::on_goal, this);
        initialpose_sub_ = nh_.subscribe("/initialpose", 1,
                                         &PlannerNode::on_initial_pose, this);

        selected_start_ = endpoint_slot(/*top=*/true, /*as_target=*/false);
        set_pose_from_slot(selected_start_);

        double dt = 1.0 / pp.update_rate;
        timer_ = nh_.createTimer(ros::Duration(dt), &PlannerNode::tick, this);

        ROS_INFO("[planner] waiting for RViz start on /initialpose and goal on /move_base_simple/goal or /goal");
    }

private:
    void tick(const ros::TimerEvent& e) {
        double dt = (e.current_real - e.last_real).toSec();
        if (dt <= 0 || dt > 1.0) dt = 0.02;

        switch (state_) {
        case State::MOVING: {
            bool done = vsim_->update(dt);
            vsim_->pose(vehicle_x_, vehicle_y_, vehicle_theta_);
            if (done) {
                state_ = State::DONE;
                ROS_INFO("[planner] arrived target  (%.3f, %.3f)",
                         vehicle_x_, vehicle_y_);
            }
            break;
        }

        case State::DONE:
            break;
        }

        publish_markers();
    }

    void on_goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        const double x = msg->pose.position.x;
        const double y = msg->pose.position.y;
        Slot target;
        std::string label;
        if (!resolve_location(x, y, /*as_target=*/true, target, label)) {
            ROS_WARN("[planner] ignored goal (%.3f, %.3f): choose inside a shelf area or near entrance/exit",
                     x, y);
            return;
        }

        start_task(selected_start_, target, selected_start_label_, label);
    }

    void on_initial_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        Slot start;
        std::string label;
        if (!resolve_location(x, y, /*as_target=*/false, start, label)) {
            ROS_WARN("[planner] ignored start (%.3f, %.3f): choose inside a shelf area or near entrance/exit",
                     x, y);
            return;
        }

        selected_start_ = start;
        selected_start_label_ = label;
        set_pose_from_slot(selected_start_);
        target_slot_id_ = -1;
        ROS_INFO("[planner] start set: %s", selected_start_label_.c_str());
    }

    Slot entrance_slot() const {
        return endpoint_slot(/*top=*/true, /*as_target=*/false);
    }

    Slot endpoint_slot(bool top, bool as_target) const {
        const auto& mp = map_->param();
        const double spine_x = mp.field_width * 0.5;
        const double h_lane_off = std::min(0.14, mp.corridor_height * 0.22);

        Slot src;
        src.id = top ? -1 : -2;
        src.row_id = top ? 0 : 7;
        src.cx = spine_x;
        src.cy = top ? mp.field_height : 0.0;
        if (top) {
            src.dock_theta = as_target ? M_PI * 0.5 : -M_PI * 0.5;
            src.pre_dock_y = mp.corridor1_cy() + h_lane_off;
        } else {
            src.dock_theta = as_target ? -M_PI * 0.5 : M_PI * 0.5;
            src.pre_dock_y = mp.corridor4_cy() - h_lane_off;
        }
        src.pre_dock_x = spine_x;
        return src;
    }

    void set_pose_from_slot(const Slot& s) {
        vehicle_x_ = s.dock_x();
        vehicle_y_ = s.dock_y();
        vehicle_theta_ = s.dock_theta;
        state_ = State::DONE;
    }

    bool resolve_location(double x, double y, bool as_target,
                          Slot& out, std::string& label) const {
        const int slot_id = nearest_slot_in_selected_shelf(x, y);
        if (slot_id >= 0) {
            out = map_->slot(slot_id);
            label = slot_label(out);
            return true;
        }

        const auto& mp = map_->param();
        const double spine_x = mp.field_width * 0.5;
        const double top_d = std::hypot(x - spine_x, y - mp.field_height);
        const double bottom_d = std::hypot(x - spine_x, y);
        const double endpoint_tol = 0.35;
        if (top_d <= endpoint_tol || bottom_d <= endpoint_tol) {
            const bool top = top_d <= bottom_d;
            out = endpoint_slot(top, as_target);
            label = top ? "entrance" : "exit";
            return true;
        }
        return false;
    }

    std::string slot_label(const Slot& s) const {
        if (s.id == -1) return "entrance";
        if (s.id == -2) return "exit";
        return "slot " + std::to_string(s.id) +
               " row=" + std::to_string(s.row_id) +
               " col=" + std::to_string(s.col);
    }

    int nearest_slot_in_selected_shelf(double x, double y) const {
        const ShelfBlock* selected_shelf = nullptr;
        for (const auto& shelf : map_->shelf_blocks()) {
            if (in_box(x, y, shelf)) {
                selected_shelf = &shelf;
                break;
            }
        }
        if (selected_shelf == nullptr) return -1;

        int best_id = -1;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (const auto& slot : map_->slots()) {
            if (!in_box(slot.cx, slot.cy, *selected_shelf)) continue;
            const double d2 = sqr_dist(x, y, slot.cx, slot.cy);
            if (d2 < best_d2) {
                best_d2 = d2;
                best_id = slot.id;
            }
        }
        return best_id;
    }

    // ── Startup path diagnostics ─────────────────────────────────────────────
    void run_path_diagnostics() {
        const Slot src = entrance_slot();
        int n_fwd = 0, n_rev = 0, n_empty = 0, n_issue = 0, n_sharp = 0;
        int n_lane = 0, n_bounds = 0, n_heading = 0;
        int n_arc_fallback = 0;
        const auto& mp = map_->param();
        const double lane_off = std::min(0.14, mp.corridor_height * 0.22);
        auto vertical_lane_off = [](double width) {
            return std::max(0.05, std::min(0.08, width * 0.18));
        };
        const double spine_x = mp.field_width * 0.5;
        const double row1_off = vertical_lane_off(mp.row1_left_aisle);
        const double row3_off = vertical_lane_off(mp.row3_center_aisle);
        const double row1_left_x = mp.row1_left_aisle * 0.5;
        const double row1_right_x =
            (mp.row1_left_aisle + mp.row1_shelf_width +
             mp.field_width - mp.row1_mini_shelf) * 0.5;

        auto corridor_center_y = [&](int corr_id) {
            switch (corr_id) {
                case 1: return mp.corridor1_cy();
                case 2: return mp.corridor2_cy();
                case 3: return mp.corridor3_cy();
                default: return mp.corridor4_cy();
            }
        };
        auto corridor_of_y = [&](double y) {
            if (y >= mp.y7() && y <= mp.y8()) return 1;
            if (y >= mp.y5() && y <= mp.y6()) return 2;
            if (y >= mp.y3() && y <= mp.y4()) return 3;
            if (y >= mp.y1() && y <= mp.y2()) return 4;
            return 0;
        };
        auto vertical_expected_x = [&](double x, double y, double dy,
                                       double& expected_x) {
            const double road_capture = 0.18;
            auto close_to_road = [&](double expected) {
                return std::abs(x - expected) <= road_capture;
            };
            if (y >= mp.y7() && y <= mp.field_height) {
                expected_x = spine_x;
                return close_to_road(expected_x);
            }
            if (y >= mp.y6() && y <= mp.y7()) {
                const double signed_off = (dy < 0.0) ? -row1_off : row1_off;
                const double left_expected = row1_left_x + signed_off;
                const double right_expected = row1_right_x + signed_off;
                expected_x =
                    (std::abs(x - left_expected) <=
                     std::abs(x - right_expected)) ? left_expected : right_expected;
                return close_to_road(expected_x);
            }
            if (y >= mp.y4() && y <= mp.y5()) {
                expected_x = spine_x;
                return close_to_road(expected_x);
            }
            if (y >= mp.y2() && y <= mp.y3()) {
                expected_x = spine_x + ((dy < 0.0) ? -row3_off : row3_off);
                return close_to_road(expected_x);
            }
            if (y >= 0.0 && y <= mp.y1()) {
                expected_x = spine_x;
                return close_to_road(expected_x);
            }
            return false;
        };

        // Max heading change per 0.01 m step for the vehicle's max curvature
        // (3.82 rad/m * 0.01 m ≈ 0.038 rad). Use 3× as sharp-corner threshold.
        const double sharp_thresh = 0.12;  // rad per waypoint (~6.9°)

        ROS_INFO("[planner] ── path diagnostics for all %zu slots ──",
                 map_->slots().size());

        for (const Slot& tgt : map_->slots()) {
            PathGenerationInfo path_info;
            RoughPath path = gen_->generate(src, const_cast<Slot&>(tgt),
                                            &path_info);

            if (path.empty()) {
                ROS_WARN("[diag] slot %2d (row=%d col=%d): EMPTY PATH",
                         tgt.id, tgt.row_id, tgt.col);
                ++n_empty;
                continue;
            }

            const bool has_rev = std::any_of(path.begin(), path.end(),
                [](const RoughWp& w){ return w.type == WpType::REVERSE; });
            has_rev ? ++n_rev : ++n_fwd;

            int lane_count = 0;
            int bounds_count = 0;
            int heading_count = 0;
            for (size_t i = 0; i < path.size(); ++i) {
                if (path[i].x < -1e-5 || path[i].x > mp.field_width + 1e-5 ||
                    path[i].y < -1e-5 || path[i].y > mp.field_height + 1e-5) {
                    ++bounds_count;
                }
                if (i + 1 >= path.size()) continue;
                const double dx = path[i + 1].x - path[i].x;
                const double dy = path[i + 1].y - path[i].y;
                const double len = std::hypot(dx, dy);
                if (len < 1e-5) continue;

                const double forward =
                    std::cos(path[i].theta) * dx / len +
                    std::sin(path[i].theta) * dy / len;
                if (path[i + 1].type == WpType::REVERSE) {
                    if (forward > 0.05) ++heading_count;
                } else if (forward < -0.05) {
                    ++heading_count;
                }

                if (std::abs(dx) > 0.02 && std::abs(dy) < 0.002) {
                    const int corr = corridor_of_y(path[i].y);
                    if (corr > 0) {
                        const double cy = corridor_center_y(corr);
                        const double expected =
                            (dx < 0.0) ? cy + lane_off : cy - lane_off;
                        if (std::abs(path[i].y - expected) > 0.025) {
                            ++lane_count;
                        }
                    }
                }
                if (std::abs(dy) > 0.02 && std::abs(dx) < 0.002) {
                    double expected_x = 0.0;
                    const double mid_y = 0.5 * (path[i].y + path[i + 1].y);
                    if (vertical_expected_x(path[i].x, mid_y, dy, expected_x) &&
                        std::abs(path[i].x - expected_x) > 0.025) {
                        ++lane_count;
                    }
                }
            }

            int sharp_count = 0;
            std::vector<RoughWp> compact;
            for (const RoughWp& wp : path) {
                if (!compact.empty() &&
                    std::hypot(wp.x - compact.back().x,
                               wp.y - compact.back().y) < 1e-5) {
                    compact.back() = wp;
                    continue;
                }
                compact.push_back(wp);
            }
            for (size_t i = 1; i + 1 < compact.size(); ++i) {
                const double h0 = std::atan2(compact[i].y - compact[i - 1].y,
                                             compact[i].x - compact[i - 1].x);
                const double h1 = std::atan2(compact[i + 1].y - compact[i].y,
                                             compact[i + 1].x - compact[i].x);
                double dth = h1 - h0;
                while (dth >  M_PI) dth -= 2*M_PI;
                while (dth < -M_PI) dth += 2*M_PI;
                const bool gear_cusp =
                    compact[i - 1].type != compact[i].type &&
                    std::abs(std::abs(dth) - M_PI) < 0.12;
                if (!gear_cusp && std::abs(dth) > sharp_thresh) ++sharp_count;
            }
            if (sharp_count > 0 || lane_count > 0 ||
                path_info.used_arc_fallback ||
                bounds_count > 0 || heading_count > 0) {
                ROS_WARN("[diag] slot %2d (row=%d col=%d): sharp=%d lane=%d "
                         "bounds=%d heading=%d arc_fallback=%d  terminal=%s  wpts=%zu",
                         tgt.id, tgt.row_id, tgt.col, sharp_count, lane_count,
                         bounds_count, heading_count,
                         path_info.used_arc_fallback ? 1 : 0,
                         has_rev ? "rev" : "fwd", path.size());
                ++n_issue;
                if (sharp_count > 0) ++n_sharp;
                if (lane_count > 0) ++n_lane;
                if (bounds_count > 0) ++n_bounds;
                if (heading_count > 0) ++n_heading;
                if (path_info.used_arc_fallback) ++n_arc_fallback;
            } else {
                ROS_INFO("[diag] slot %2d (row=%d col=%d): OK  terminal=%s  wpts=%zu",
                         tgt.id, tgt.row_id, tgt.col,
                         has_rev ? "rev" : "fwd", path.size());
            }
        }

        ROS_INFO("[planner] diagnostics done: fwd=%d rev=%d empty=%d "
                 "issue=%d sharp=%d lane=%d bounds=%d heading=%d arc_fallback=%d",
                 n_fwd, n_rev, n_empty, n_issue, n_sharp, n_lane, n_bounds,
                 n_heading, n_arc_fallback);
    }

    void start_task(const Slot& src, const Slot& tgt,
                    const std::string& src_label,
                    const std::string& tgt_label) {
        PathGenerationInfo path_info;
        RoughPath path = gen_->generate(src, tgt, &path_info);
        const bool has_reverse =
            std::any_of(path.begin(), path.end(), [](const RoughWp& wp) {
                return wp.type == WpType::REVERSE;
            });
        vsim_->set_path(path);

        if (!path.empty()) {
            vehicle_x_ = path.front().x;
            vehicle_y_ = path.front().y;
            vehicle_theta_ = path.front().theta;
            state_ = State::MOVING;
        } else {
            state_ = State::DONE;
        }

        target_slot_id_ = (tgt.id >= 0) ? tgt.id : -1;
        ROS_INFO("[planner] task: %s -> %s  "
                 "(%zu waypoints, terminal=%s, arc_fallback=%d)",
                 src_label.c_str(), tgt_label.c_str(), path.size(),
                 has_reverse ? "reverse" : "forward",
                 path_info.used_arc_fallback ? 1 : 0);
    }

    // ── Marker publishing ────────────────────────────────────────────────────
    void publish_markers() {
        visualization_msgs::MarkerArray arr;
        int mid = 0;

        if (!vsim_->empty()) {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_;
            m.header.stamp    = ros::Time::now();
            m.ns = "path"; m.id = mid++;
            m.type   = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.014;
            m.color = rgba(pc_r_, pc_g_, pc_b_, 1.0f);

            const auto& wp = vsim_->path();
            for (const auto& p : wp) {
                m.points.push_back(pt3(p.x, p.y));
            }
            arr.markers.push_back(m);
        } else {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_;
            m.ns = "path"; m.id = mid++;
            m.action = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(m);
        }

        {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_;
            m.header.stamp    = ros::Time::now();
            m.ns = "vehicle"; m.id = mid++;
            m.type   = visualization_msgs::Marker::CUBE;
            m.action = visualization_msgs::Marker::ADD;

            m.pose.position.x = vehicle_x_;
            m.pose.position.y = vehicle_y_;
            m.pose.position.z = 0.025;

            double c = std::cos(vehicle_theta_ * 0.5);
            double s = std::sin(vehicle_theta_ * 0.5);
            m.pose.orientation.z = s;
            m.pose.orientation.w = c;

            const auto& mp = map_->param();
            m.scale.x = mp.vehicle_length;
            m.scale.y = mp.vehicle_width;
            m.scale.z = 0.030;
            m.color = rgba(1.0f, 0.6f, 0.0f);
            arr.markers.push_back(m);
        }

        {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_;
            m.header.stamp    = ros::Time::now();
            m.ns = "vehicle_arrow"; m.id = mid++;
            m.type   = visualization_msgs::Marker::ARROW;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.012;  // shaft diam
            m.scale.y = 0.024;  // head diam
            m.scale.z = 0.0;
            m.color = rgba(1.0f, 1.0f, 1.0f);

            double half = map_->param().vehicle_length * 0.6;
            double dx = std::cos(vehicle_theta_) * half;
            double dy = std::sin(vehicle_theta_) * half;
            m.points.push_back(pt3(vehicle_x_ - dx * 0.5, vehicle_y_ - dy * 0.5, 0.040));
            m.points.push_back(pt3(vehicle_x_ + dx,       vehicle_y_ + dy,       0.040));
            arr.markers.push_back(m);
        }

        if (target_slot_id_ >= 0) {
            add_slot_highlight(arr, mid, target_slot_id_, rgba(1.0f, 0.9f, 0.0f));
        }

        sim_pub_.publish(arr);
    }

    void add_slot_highlight(visualization_msgs::MarkerArray& arr, int& mid,
                             int slot_id, const std_msgs::ColorRGBA& col) {
        const Slot& s = map_->slot(slot_id);
        const auto& mp = map_->param();
        double hw = mp.vehicle_width  * 0.5;
        double hd = mp.vehicle_length * 0.5;

        visualization_msgs::Marker m;
        m.header.frame_id = frame_;
        m.header.stamp    = ros::Time::now();
        m.ns = "slot_hi"; m.id = mid++;
        m.type   = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.008;
        m.color = col;

        double cx = s.cx, cy = s.cy;
        m.points = {pt3(cx-hw, cy-hd, 0.016),
                    pt3(cx+hw, cy-hd, 0.016),
                    pt3(cx+hw, cy+hd, 0.016),
                    pt3(cx-hw, cy+hd, 0.016),
                    pt3(cx-hw, cy-hd, 0.016)};
        arr.markers.push_back(m);
    }

    // ── Members ──────────────────────────────────────────────────────────────
    ros::NodeHandle nh_;
    ros::Publisher  sim_pub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber legacy_goal_sub_;
    ros::Subscriber initialpose_sub_;
    ros::Timer      timer_;

    std::unique_ptr<ForkliftMap>    map_;
    std::unique_ptr<PathGenerator>  gen_;
    std::unique_ptr<VehicleSim>     vsim_;

    State      state_      = State::DONE;
    int        target_slot_id_ = -1;
    Slot       selected_start_;
    std::string selected_start_label_ = "entrance";

    double vehicle_x_     = 0.0;
    double vehicle_y_     = 0.0;
    double vehicle_theta_ = 0.0;

    std::string frame_;
    float pc_r_, pc_g_, pc_b_;
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    PlannerNode node;
    ros::spin();
    return 0;
}
