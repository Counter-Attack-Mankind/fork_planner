#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"

namespace {

constexpr double kPi = M_PI;

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

geometry_msgs::Point pt3(double x, double y, double z = 0.08) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

double norm_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

struct OBB {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double half_l = 0.0;
    double half_w = 0.0;
};

void axes(const OBB& b, double ax[2][2]) {
    const double c = std::cos(b.theta);
    const double s = std::sin(b.theta);
    ax[0][0] = c;
    ax[0][1] = s;
    ax[1][0] = -s;
    ax[1][1] = c;
}

double radius_on_axis(const OBB& b, double ux, double uy) {
    double ax[2][2];
    axes(b, ax);
    return b.half_l * std::abs(ux * ax[0][0] + uy * ax[0][1]) +
           b.half_w * std::abs(ux * ax[1][0] + uy * ax[1][1]);
}

bool overlaps(const OBB& a, const OBB& b) {
    double ax_a[2][2];
    double ax_b[2][2];
    axes(a, ax_a);
    axes(b, ax_b);

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double test_axes[4][2] = {
        {ax_a[0][0], ax_a[0][1]},
        {ax_a[1][0], ax_a[1][1]},
        {ax_b[0][0], ax_b[0][1]},
        {ax_b[1][0], ax_b[1][1]},
    };

    for (const auto& u : test_axes) {
        const double center_dist = std::abs(dx * u[0] + dy * u[1]);
        if (center_dist > radius_on_axis(a, u[0], u[1]) +
                              radius_on_axis(b, u[0], u[1])) {
            return false;
        }
    }
    return true;
}

struct PathTrack {
    RoughPath path;
    std::vector<double> s;
    double length = 0.0;
    double duration = 0.0;

    void set(const RoughPath& p, double speed) {
        path = p;
        s.clear();
        s.reserve(path.size());
        length = 0.0;
        for (size_t i = 0; i < path.size(); ++i) {
            if (i > 0) {
                length += std::hypot(path[i].x - path[i - 1].x,
                                     path[i].y - path[i - 1].y);
            }
            s.push_back(length);
        }
        duration = speed > 1e-6 ? length / speed : 0.0;
    }

    RoughWp pose_at(double t, double delay, double speed) const {
        if (path.empty()) return {};
        if (t <= delay) return path.front();
        const double dist = std::min((t - delay) * speed, length);
        if (dist >= length - 1e-9) return path.back();

        auto it = std::lower_bound(s.begin(), s.end(), dist);
        size_t idx = static_cast<size_t>(std::distance(s.begin(), it));
        if (idx == 0) return path.front();
        if (idx >= path.size()) return path.back();

        const double seg_len = s[idx] - s[idx - 1];
        const double a = seg_len > 1e-9 ? (dist - s[idx - 1]) / seg_len : 1.0;
        const RoughWp& p0 = path[idx - 1];
        const RoughWp& p1 = path[idx];
        RoughWp out;
        out.x = p0.x + a * (p1.x - p0.x);
        out.y = p0.y + a * (p1.y - p0.y);
        out.theta = norm_angle(p0.theta + a * norm_angle(p1.theta - p0.theta));
        out.type = p1.type;
        return out;
    }
};

struct CollisionReport {
    bool collision = false;
    double first_t = 0.0;
    double x = 0.0;
    double y = 0.0;
};

struct Schedule {
    double delay_a = 0.0;
    double delay_b = 0.0;
    double makespan = 0.0;
    bool feasible = false;
};

class TwoVehicleSimNode {
public:
    TwoVehicleSimNode() : nh_("~") {
        mp_ = MapParam::fromROSParam(nh_);
        pp_ = PlannerParam::fromROSParam(nh_);
        map_ = std::make_unique<ForkliftMap>(mp_);
        gen_ = std::make_unique<PathGenerator>(mp_, pp_);

        nh_.param("vehicle_a_start_slot", a_start_id_, 0);
        nh_.param("vehicle_a_target_slot", a_target_id_, 67);
        nh_.param("vehicle_b_start_slot", b_start_id_, 67);
        nh_.param("vehicle_b_target_slot", b_target_id_, 0);
        nh_.param("time_step", dt_check_, 0.05);
        nh_.param("delay_step", delay_step_, 0.10);
        nh_.param("safety_margin", safety_margin_, 0.04);
        nh_.param("loop_visualization", loop_visualization_, true);

        pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);

        build_paths();
        schedule_ = solve_by_start_delay();
        sim_period_ = std::max(schedule_.makespan + 3.0, 1.0);

        ROS_INFO("[two_vehicle] A: %s -> %s  len=%.3f dur=%.2f wpts=%zu arc=%d",
                 label(a_src_).c_str(), label(a_tgt_).c_str(),
                 path_a_.length, path_a_.duration, path_a_.path.size(),
                 a_info_.used_arc_fallback ? 1 : 0);
        ROS_INFO("[two_vehicle] B: %s -> %s  len=%.3f dur=%.2f wpts=%zu arc=%d",
                 label(b_src_).c_str(), label(b_tgt_).c_str(),
                 path_b_.length, path_b_.duration, path_b_.path.size(),
                 b_info_.used_arc_fallback ? 1 : 0);

        CollisionReport no_delay = first_collision(0.0, 0.0);
        if (no_delay.collision) {
            ROS_WARN("[two_vehicle] no-delay conflict at t=%.2f near (%.3f, %.3f)",
                     no_delay.first_t, no_delay.x, no_delay.y);
        } else {
            ROS_INFO("[two_vehicle] no-delay schedule is already collision-free");
        }

        if (schedule_.feasible) {
            ROS_INFO("[two_vehicle] schedule feasible: delay_a=%.2f delay_b=%.2f makespan=%.2f",
                     schedule_.delay_a, schedule_.delay_b, schedule_.makespan);
        } else {
            ROS_ERROR("[two_vehicle] no feasible start-delay schedule found");
        }

        timer_ = nh_.createTimer(ros::Duration(1.0 / pp_.update_rate),
                                 &TwoVehicleSimNode::tick, this);
    }

private:
    Slot endpoint_slot(bool top, bool as_target) const {
        const double spine_x = mp_.field_width * 0.5;
        const double h_lane_off = std::min(0.14, mp_.corridor_height * 0.22);
        Slot s;
        s.id = top ? -1 : -2;
        s.row_id = top ? 0 : 7;
        s.cx = spine_x;
        s.cy = top ? mp_.field_height : 0.0;
        if (top) {
            s.dock_theta = as_target ? kPi * 0.5 : -kPi * 0.5;
            s.pre_dock_y = mp_.corridor1_cy() + h_lane_off;
        } else {
            s.dock_theta = as_target ? -kPi * 0.5 : kPi * 0.5;
            s.pre_dock_y = mp_.corridor4_cy() - h_lane_off;
        }
        s.pre_dock_x = spine_x;
        return s;
    }

    Slot resolve_slot(int id, bool as_target) const {
        if (id >= 0) return map_->slot(id);
        if (id == -1) return endpoint_slot(true, as_target);
        if (id == -2) return endpoint_slot(false, as_target);
        ROS_WARN("[two_vehicle] unsupported endpoint id %d, using entrance", id);
        return endpoint_slot(true, as_target);
    }

    std::string label(const Slot& s) const {
        if (s.id == -1) return "entrance";
        if (s.id == -2) return "exit";
        return "slot " + std::to_string(s.id) +
               " row=" + std::to_string(s.row_id) +
               " col=" + std::to_string(s.col);
    }

    void build_paths() {
        a_src_ = resolve_slot(a_start_id_, false);
        a_tgt_ = resolve_slot(a_target_id_, true);
        b_src_ = resolve_slot(b_start_id_, false);
        b_tgt_ = resolve_slot(b_target_id_, true);

        path_a_.set(gen_->generate(a_src_, a_tgt_, &a_info_), pp_.vehicle_speed);
        path_b_.set(gen_->generate(b_src_, b_tgt_, &b_info_), pp_.vehicle_speed);
    }

    OBB body_at(const PathTrack& track, double t, double delay) const {
        const RoughWp p = track.pose_at(t, delay, pp_.vehicle_speed);
        OBB b;
        b.x = p.x;
        b.y = p.y;
        b.theta = p.theta;
        b.half_l = mp_.vehicle_length * 0.5 + safety_margin_;
        b.half_w = mp_.vehicle_width * 0.5 + safety_margin_;
        return b;
    }

    CollisionReport first_collision(double delay_a, double delay_b) const {
        CollisionReport report;
        const double end_t = std::max(delay_a + path_a_.duration,
                                      delay_b + path_b_.duration);
        for (double t = 0.0; t <= end_t + 1e-9; t += dt_check_) {
            const OBB a = body_at(path_a_, t, delay_a);
            const OBB b = body_at(path_b_, t, delay_b);
            if (overlaps(a, b)) {
                report.collision = true;
                report.first_t = t;
                report.x = 0.5 * (a.x + b.x);
                report.y = 0.5 * (a.y + b.y);
                return report;
            }
        }
        return report;
    }

    Schedule earliest_with_delay(bool delay_b) const {
        Schedule best;
        const double max_delay =
            path_a_.duration + path_b_.duration + 10.0;
        for (double delay = 0.0; delay <= max_delay + 1e-9;
             delay += delay_step_) {
            const double da = delay_b ? 0.0 : delay;
            const double db = delay_b ? delay : 0.0;
            if (!first_collision(da, db).collision) {
                best.delay_a = da;
                best.delay_b = db;
                best.makespan = std::max(da + path_a_.duration,
                                         db + path_b_.duration);
                best.feasible = true;
                return best;
            }
        }
        return best;
    }

    Schedule solve_by_start_delay() const {
        Schedule a_first = earliest_with_delay(true);
        Schedule b_first = earliest_with_delay(false);
        if (!a_first.feasible) return b_first;
        if (!b_first.feasible) return a_first;
        if (b_first.makespan + 1e-9 < a_first.makespan) return b_first;
        return a_first;
    }

    void tick(const ros::TimerEvent& event) {
        if (!schedule_.feasible) {
            publish_markers(0.0);
            return;
        }

        if (start_time_.isZero()) start_time_ = event.current_real;
        double t = (event.current_real - start_time_).toSec();
        if (loop_visualization_ && sim_period_ > 1e-6) {
            t = std::fmod(t, sim_period_);
        }
        publish_markers(t);
    }

    void add_path(visualization_msgs::MarkerArray& arr, int id,
                  const PathTrack& track,
                  const std_msgs::ColorRGBA& color) const {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "two_vehicle_path";
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.015;
        m.color = color;
        for (const auto& p : track.path) {
            m.points.push_back(pt3(p.x, p.y, 0.045));
        }
        arr.markers.push_back(m);
    }

    void add_body(visualization_msgs::MarkerArray& arr, int id,
                  const PathTrack& track, double t, double delay,
                  const std_msgs::ColorRGBA& color) const {
        const RoughWp p = track.pose_at(t, delay, pp_.vehicle_speed);
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "two_vehicle_body";
        m.id = id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = p.x;
        m.pose.position.y = p.y;
        m.pose.position.z = 0.035;
        m.pose.orientation.z = std::sin(p.theta * 0.5);
        m.pose.orientation.w = std::cos(p.theta * 0.5);
        m.scale.x = mp_.vehicle_length;
        m.scale.y = mp_.vehicle_width;
        m.scale.z = 0.05;
        m.color = color;
        arr.markers.push_back(m);
    }

    void publish_markers(double t) const {
        visualization_msgs::MarkerArray arr;
        add_path(arr, 0, path_a_, rgba(0.1f, 0.65f, 1.0f, 1.0f));
        add_path(arr, 1, path_b_, rgba(1.0f, 0.25f, 0.75f, 1.0f));
        add_body(arr, 2, path_a_, t, schedule_.delay_a,
                 rgba(0.0f, 0.45f, 1.0f, 1.0f));
        add_body(arr, 3, path_b_, t, schedule_.delay_b,
                 rgba(0.9f, 0.0f, 0.65f, 1.0f));

        const CollisionReport no_delay = first_collision(0.0, 0.0);
        if (no_delay.collision) {
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = ros::Time::now();
            m.ns = "two_vehicle_conflict";
            m.id = 4;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = no_delay.x;
            m.pose.position.y = no_delay.y;
            m.pose.position.z = 0.09;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.08;
            m.scale.y = 0.08;
            m.scale.z = 0.02;
            m.color = rgba(1.0f, 0.0f, 0.0f, 0.85f);
            arr.markers.push_back(m);
        }
        pub_.publish(arr);
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;
    ros::Time start_time_;

    MapParam mp_;
    PlannerParam pp_;
    std::unique_ptr<ForkliftMap> map_;
    std::unique_ptr<PathGenerator> gen_;

    int a_start_id_ = 0;
    int a_target_id_ = 67;
    int b_start_id_ = 67;
    int b_target_id_ = 0;
    double dt_check_ = 0.05;
    double delay_step_ = 0.10;
    double safety_margin_ = 0.04;
    bool loop_visualization_ = true;

    Slot a_src_;
    Slot a_tgt_;
    Slot b_src_;
    Slot b_tgt_;
    PathTrack path_a_;
    PathTrack path_b_;
    PathGenerationInfo a_info_;
    PathGenerationInfo b_info_;
    Schedule schedule_;
    double sim_period_ = 1.0;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "two_vehicle_sim_node");
    TwoVehicleSimNode node;
    ros::spin();
    return 0;
}
