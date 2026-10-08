#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_map/path_debug_visualizer.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"
#include "std_msgs/ColorRGBA.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

//================（1.辅助可视化函数）===============
std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

geometry_msgs::Point point(double x, double y, double z = 0.0) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

visualization_msgs::Marker baseMarker(const std::string& frame,
                                      const std::string& ns,
                                      int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = ros::Time(0);
    m.ns = ns;
    m.id = id;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
}

void addText(visualization_msgs::MarkerArray& arr,
             const std::string& frame,
             const std::string& ns,
             int id,
             double x,
             double y,
             double z,
             double height,
             const std::string& text,
             const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.z = height;
    m.text = text;
    m.color = color;
    arr.markers.push_back(m);
}

void addArrow(visualization_msgs::MarkerArray& arr,
              const std::string& frame,
              const std::string& ns,
              int id,
              double x,
              double y,
              double theta,
              const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::ARROW;
    m.points.push_back(point(x, y, 0.09));
    m.points.push_back(point(x + 0.22 * std::cos(theta),
                             y + 0.22 * std::sin(theta), 0.09));
    m.scale.x = 0.018;
    m.scale.y = 0.045;
    m.scale.z = 0.055;
    m.color = color;
    arr.markers.push_back(m);
}

void addSphere(visualization_msgs::MarkerArray& arr,
               const std::string& frame,
               const std::string& ns,
               int id,
               double x,
               double y,
               const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.08;
    m.scale.x = 0.09;
    m.scale.y = 0.09;
    m.scale.z = 0.05;
    m.color = color;
    arr.markers.push_back(m);
}

void addPath(visualization_msgs::MarkerArray& arr,
             const std::string& frame,
             const std::string& ns,
             int id,
             const RoughPath& path,
             const std_msgs::ColorRGBA& color,
             double z_offset) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = 0.010;
    m.color = color;
    m.points.reserve(path.size());
    for (const RoughWp& wp : path) {
        m.points.push_back(point(wp.x, wp.y, z_offset));
    }
    arr.markers.push_back(m);
}

void addLineStrip(visualization_msgs::MarkerArray& arr,
                  const std::string& frame,
                  const std::string& ns,
                  int id,
                  const std::vector<geometry_msgs::Point>& points,
                  const std_msgs::ColorRGBA& color,
                  double width,
                  double z_offset) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = width;
    m.color = color;
    m.points = points;
    for (geometry_msgs::Point& p : m.points) {
        p.z = z_offset;
    }
    arr.markers.push_back(m);
}

double pathLength(const RoughPath& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        len += std::hypot(path[i].x - path[i - 1].x,
                          path[i].y - path[i - 1].y);
    }
    return len;
}

std::string idsToString(const std::vector<int>& ids) {
    std::ostringstream out;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i > 0) out << ", ";
        out << ids[i];
    }
    return out.str();
}


//======（2.构造虚拟货位A1,A2，用于复用路径生成器）==================
Slot makeVirtualSlot(const std::string& name,
                     int id,
                     int row_id,
                     double x,
                     double y,
                     double pre_x,
                     double pre_y,
                     double theta) {
    Slot s;
    s.id = id;
    s.row_id = row_id;
    s.col = -1;
    s.cx = x;
    s.cy = y;
    s.pre_dock_x = pre_x;
    s.pre_dock_y = pre_y;
    s.dock_theta = theta;
    s.occupied = false;
    ROS_INFO("[path_catalog] %s virtual slot: id=%d row=%d dock=(%.4f,%.4f) pre=(%.4f,%.4f) yaw=%.1fdeg",
             name.c_str(), id, row_id, x, y, pre_x, pre_y,
             theta * 180.0 / kPi);
    return s;
}

//==========（3.数学工具）=============
double midpoint(double a, double b) {
    return 0.5 * (a + b);
}

double normAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

double angleLerp(double a, double b, double u) {
    return normAngle(a + normAngle(b - a) * u);
}

double angleDiffAbs(double a, double b) {
    return std::abs(normAngle(b - a));
}

RoughWp poseAtS(const RoughPath& path, double query_s) {
    if (path.empty()) return {0.0, 0.0, 0.0, WpType::FORWARD};
    if (path.size() == 1 || query_s <= 0.0) return path.front();

    double acc = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        const RoughWp& a = path[i - 1];
        const RoughWp& b = path[i];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        if (seg <= 1e-9) continue;
        if (acc + seg >= query_s) {
            const double u = std::max(0.0, std::min(1.0, (query_s - acc) / seg));
            return {a.x + (b.x - a.x) * u,
                    a.y + (b.y - a.y) * u,
                    angleLerp(a.theta, b.theta, u),
                    u < 0.5 ? a.type : b.type};
        }
        acc += seg;
    }
    return path.back();
}

std::string uppercase(std::string s) {
    for (char& ch : s) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return s;
}

// ================（4.定义路径生成失败的原因，枚举类）==================
// 无，路径为空，曲率不连续，车辆包络超出地图，车辆与货架发生碰撞，路径存在折角或姿态突变

enum class DebugRejectReason {
    NONE,
    EMPTY_PATH,
    CURVATURE_DISCONTINUITY,
    FOOTPRINT_OUT_OF_BOUNDS,
    SHELF_COLLISION,
    KINK,
};

const char* rejectReasonName(DebugRejectReason reason) {
    switch (reason) {
        case DebugRejectReason::NONE: return "none";
        case DebugRejectReason::EMPTY_PATH: return "empty_path";
        case DebugRejectReason::CURVATURE_DISCONTINUITY:
            return "curvature_discontinuity";
        case DebugRejectReason::FOOTPRINT_OUT_OF_BOUNDS:
            return "footprint_out_of_bounds";
        case DebugRejectReason::SHELF_COLLISION: return "shelf_collision";
        case DebugRejectReason::KINK: return "kink";
    }
    return "unknown";
}

const char* debugLayerTypeName(DebugPathLayerType type) {
    switch (type) {
        case DebugPathLayerType::SKELETON: return "skeleton";
        case DebugPathLayerType::CLOTHOID: return "clothoid";
        case DebugPathLayerType::LANE_SHIFT: return "lane_shift";
        case DebugPathLayerType::ARC_FALLBACK: return "arc_fallback";
    }
    return "unknown";
}

}  // namespace

//===================（1.构造函数初始化）=========================
//  完成参数读取 → 地图构造 → 路径生成 → Marker 发布 → 动画启动

class PathCatalogDebugNode {
public:
    PathCatalogDebugNode() : nh_("~") {
        
        //============ 1.1 读取全局参数
        ros::NodeHandle param_nh;
        mp_ = MapParam::fromROSParam(param_nh);
        pp_ = PlannerParam::fromROSParam(param_nh);
        cfg_ = forklift_planner::multi_vehicle::MultiVehicleConfig::fromROSParam(
            param_nh);
        
        //========= 1.2 创建地图对象和路径生成器对象 =========
        map_ = std::make_unique<ForkliftMap>(mp_);
        generator_a1_to_b_ = std::make_unique<PathGenerator>(
            mp_, pp_, PathGeneratorRouteMode::A1_TO_B);
        generator_b_to_a1_ = std::make_unique<PathGenerator>(
            mp_, pp_, PathGeneratorRouteMode::B_TO_A1);
        generator_a2_to_b_ = std::make_unique<PathGenerator>(
            mp_, pp_, PathGeneratorRouteMode::A2_TO_B);
        generator_b_to_a2_ = std::make_unique<PathGenerator>(
            mp_, pp_, PathGeneratorRouteMode::B_TO_A2);
        
        //========  1.3 读取A1,A2，运行参数模式 ===================
        use_exact_midpoints_ = true;
        nh_.param("use_exact_midpoints", use_exact_midpoints_,
                  use_exact_midpoints_);
        nh_.param("a1_x", a1_x_, 1.25);
        nh_.param("a1_y", a1_y_, 4.38);
        nh_.param("a1_pre_x", a1_pre_x_, a1_x_);
        nh_.param("a1_pre_y", a1_pre_y_, a1_y_ - virtual_pre_dock_distance());
        nh_.param("a1_yaw", a1_yaw_, kPi * 0.5);
        nh_.param("a2_x", a2_x_, 1.25);
        nh_.param("a2_y", a2_y_, 0.12);
        nh_.param("a2_pre_x", a2_pre_x_, a2_x_);
        nh_.param("a2_pre_y", a2_pre_y_, a2_y_ + virtual_pre_dock_distance());
        nh_.param("a2_yaw", a2_yaw_, -kPi * 0.5);
        nh_.param("depot", depot_name_, depot_name_);
        nh_.param("direction", direction_, direction_);
        nh_.param("target_slot", target_slot_, target_slot_);
        nh_.param("target_row", target_row_, target_row_);
        nh_.param("validate_paths", validate_paths_, validate_paths_);
        nh_.param("visualize_rejections", visualize_rejections_,
                  visualize_rejections_);
        nh_.param("visualize_path_layers", visualize_path_layers_,
                  visualize_path_layers_);
        nh_.param("visualize_turn_types", visualize_turn_types_,
                  visualize_turn_types_);
        nh_.param("verbose_batch_paths", verbose_batch_paths_,
                  verbose_batch_paths_);
        nh_.param("animate", animate_, animate_);
        nh_.param("animation_period", animation_period_, animation_period_);
        nh_.param("animation_duration", animation_duration_, animation_duration_);

        //========== 1.4 此时A1与A2通过B4,B5;B60,B61得出，确定取货点A1,A2位置 ==========
        if (use_exact_midpoints_ && map_->slots().size() > 61) {
            const Slot& b4 = map_->slots().at(4);
            const Slot& b5 = map_->slots().at(5);
            const Slot& b60 = map_->slots().at(60);
            const Slot& b61 = map_->slots().at(61);
            a1_x_ = midpoint(b4.cx, b5.cx);
            a1_y_ = midpoint(b4.cy, b5.cy);
            a1_pre_x_ = midpoint(b4.pre_dock_x, b5.pre_dock_x);
            a1_pre_y_ = midpoint(b4.pre_dock_y, b5.pre_dock_y);
            a2_x_ = midpoint(b60.cx, b61.cx);
            a2_y_ = midpoint(b60.cy, b61.cy);
            a2_pre_x_ = midpoint(b60.pre_dock_x, b61.pre_dock_x);
            a2_pre_y_ = midpoint(b60.pre_dock_y, b61.pre_dock_y);
            ROS_INFO("[path_catalog] midpoint check: B4(%.4f,%.4f), B5(%.4f,%.4f) -> A1(%.4f,%.4f)",
                     b4.cx, b4.cy, b5.cx, b5.cy, a1_x_, a1_y_);
            ROS_INFO("[path_catalog] midpoint check: B60(%.4f,%.4f), B61(%.4f,%.4f) -> A2(%.4f,%.4f)",
                     b60.cx, b60.cy, b61.cx, b61.cy, a2_x_, a2_y_);
        }

        //========= 1.5 创建发布器来显示rviz中的图形 ======================
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_map/markers", 1, true);

        //=========== 1.6 发布路径 =======================
        publish();
        if (!selected_path_.empty()) {
            animation_start_ = ros::Time::now();
            publishVehicleMarker(poseAtS(selected_path_, 0.0));
            if (animate_) {
                timer_ = nh_.createTimer(ros::Duration(animation_period_),
                                         &PathCatalogDebugNode::onAnimationTimer,
                                         this);
            }
        }
    }

private:
    double virtual_pre_dock_distance() const {
        return mp_.bottom_shelf_depth * 0.5 + mp_.pre_dock_clearance;
    }

    const PathGenerator& activeGenerator(const std::string& depot_label,
                                         bool to_depot) const {
        const bool use_a2 = depot_label == "A2";
        if (use_a2) {
            return to_depot ? *generator_b_to_a2_ : *generator_a2_to_b_;
        }
        return to_depot ? *generator_b_to_a1_ : *generator_a1_to_b_;
    }

    void publish() {
        const Slot a1 = makeVirtualSlot("A1", 101, 0, a1_x_, a1_y_,
                                        a1_pre_x_, a1_pre_y_, a1_yaw_);
        const Slot a2 = makeVirtualSlot("A2", 102, 7, a2_x_, a2_y_,
                                        a2_pre_x_, a2_pre_y_, a2_yaw_);

        std::vector<int> all_targets;
        for (const Slot& s : map_->slots()) {
            if (s.id < 0 || s.id > 65) continue;
            if (target_row_ >= 0 && s.row_id != target_row_) continue;
            all_targets.push_back(s.id);
        }

        const bool use_a2 = uppercase(depot_name_) == "A2";
        const Slot& depot = use_a2 ? a2 : a1;
        const std::string label = use_a2 ? "A2" : "A1";
        const bool to_depot = uppercase(direction_) == "TO_DEPOT";

        ROS_WARN("[path_catalog] selected depot=%s direction=%s target_row=%d "
                 "targets (%zu): [%s]",
                 label.c_str(), to_depot ? "to_depot" : "from_depot",
                 target_row_, all_targets.size(),
                 idsToString(all_targets).c_str());

        if (target_slot_ >= 0) {
            publishSinglePath(depot, label, to_depot);
            return;
        }

        visualization_msgs::MarkerArray arr;
        int id = 0;
        const std_msgs::ColorRGBA color =
            use_a2 ? rgba(1.0f, 0.55f, 0.05f, 1.0f)
                   : rgba(0.1f, 0.65f, 1.0f, 1.0f);
        addDepotMarkers(arr, id, depot, label, color);
        addTargetMarkers(arr, id, all_targets, rgba(1.0f, 1.0f, 1.0f, 0.85f),
                         "all_B_targets");

        buildAndDrawPaths(arr, id, depot, all_targets, label, to_depot,
                          use_a2 ? rgba(1.0f, 0.55f, 0.05f, 0.42f)
                                 : rgba(0.15f, 0.75f, 1.0f, 0.42f),
                          0.065);

        static_markers_ = arr;
        pub_.publish(arr);
        ROS_WARN("[path_catalog] published %zu markers on /forklift_map/markers",
                 arr.markers.size());
    }

    void publishSinglePath(const Slot& depot,
                           const std::string& label,
                           bool to_depot) {
        if (target_slot_ < 0 ||
            target_slot_ >= static_cast<int>(map_->slots().size()) ||
            target_slot_ > 65) {
            ROS_ERROR("[path_catalog] target_slot=%d is out of range [0,%zu)",
                      target_slot_, map_->slots().size());
            return;
        }

        const Slot& slot = map_->slots().at(static_cast<size_t>(target_slot_));
        const Slot& src = to_depot ? slot : depot;
        const Slot& dst = to_depot ? depot : slot;
        const std::string slot_label = "B" + std::to_string(target_slot_);
        PathGenerationInfo info;
        selected_path_ = activeGenerator(label, to_depot).generate(src, dst, &info);
        selected_label_ = to_depot
            ? (slot_label + "_to_" + label)
            : (label + "_to_" + slot_label);
        logPathGenerationDebug(selected_label_, info);
        if (selected_path_.size() < 2) {
            ROS_ERROR("[path_catalog] %s rejected: empty_path",
                      selected_label_.c_str());
            selected_path_.clear();
            return;
        }
        if (validate_paths_) {
            const DebugRejectReason reject =
                validatePath(selected_path_, info, src, dst);
            if (reject != DebugRejectReason::NONE) {
                ROS_ERROR("[path_catalog] %s rejected by task-style validation: %s",
                          selected_label_.c_str(), rejectReasonName(reject));
                logRejectDetails(selected_label_, selected_path_, src, dst, reject);
                visualization_msgs::MarkerArray arr;
                int id = 0;
                const bool use_a2 = label == "A2";
                const std_msgs::ColorRGBA color =
                    use_a2 ? rgba(1.0f, 0.55f, 0.05f, 1.0f)
                           : rgba(0.1f, 0.65f, 1.0f, 1.0f);
                addDepotMarkers(arr, id, depot, label, color);
                addSphere(arr, pp_.frame_id, "single_target_point", id++,
                          slot.cx, slot.cy, rgba(1.0f, 1.0f, 1.0f, 1.0f));
                addPath(arr, pp_.frame_id, "single_rejected_path", id++,
                        selected_path_, rgba(1.0f, 0.1f, 0.1f, 0.65f), 0.085);
                addPathLayerDiagnostics(arr, id, selected_path_, info,
                                        selected_label_);
                addRejectionDiagnostics(arr, id, selected_path_, info, src, dst,
                                        reject, selected_label_);
                static_markers_ = arr;
                pub_.publish(arr);
                if (!visualize_rejections_) selected_path_.clear();
                return;
            }
        }

        const bool use_a2 = label == "A2";
        const std_msgs::ColorRGBA color =
            use_a2 ? rgba(1.0f, 0.55f, 0.05f, 1.0f)
                   : rgba(0.1f, 0.65f, 1.0f, 1.0f);
        const double len = pathLength(selected_path_);
        ROS_WARN("[path_catalog] %ssingle path %s row=%d col=%d wpts=%zu len=%.3f arc=%d animate=%d",
                 validate_paths_ ? "" : "RAW ",
                 selected_label_.c_str(), slot.row_id, slot.col,
                 selected_path_.size(), len, info.used_arc_fallback ? 1 : 0,
                 animate_ ? 1 : 0);

        visualization_msgs::MarkerArray arr;
        int id = 0;
        addDepotMarkers(arr, id, depot, label, color);
        addSphere(arr, pp_.frame_id, "single_target_point", id++, slot.cx, slot.cy,
                  rgba(1.0f, 1.0f, 1.0f, 1.0f));
        addText(arr, pp_.frame_id, "single_target_label", id++, slot.cx, slot.cy,
                0.19, 0.075, slot_label,
                rgba(1.0f, 1.0f, 1.0f, 1.0f));
        addPath(arr, pp_.frame_id, "single_path", id++, selected_path_, color,
                0.075);
        addPathLayerDiagnostics(arr, id, selected_path_, info, selected_label_);
        addText(arr, pp_.frame_id, "single_path_label", id++,
                midpoint(src.cx, dst.cx), midpoint(src.cy, dst.cy), 0.22,
                0.07, selected_label_, color);
        static_markers_ = arr;
        pub_.publish(arr);
        ROS_WARN("[path_catalog] published single path markers on /forklift_map/markers");
    }

    void addDepotMarkers(visualization_msgs::MarkerArray& arr,
                         int& id,
                         const Slot& depot,
                         const std::string& label,
                         const std_msgs::ColorRGBA& color) const {
        addSphere(arr, pp_.frame_id, "virtual_depot", id++, depot.cx, depot.cy,
                  color);
        addArrow(arr, pp_.frame_id, "virtual_depot_heading", id++, depot.cx,
                 depot.cy, depot.dock_theta, color);
        addText(arr, pp_.frame_id, "virtual_depot_label", id++, depot.cx,
                depot.cy, 0.18, 0.10, label, color);
    }

    void addTargetMarkers(visualization_msgs::MarkerArray& arr,
                          int& id,
                          const std::vector<int>& targets,
                          const std_msgs::ColorRGBA& color,
                          const std::string& ns) const {
        for (int target : targets) {
            const Slot& s = map_->slots().at(static_cast<size_t>(target));
            addText(arr, pp_.frame_id, ns, id++, s.cx, s.cy, 0.16, 0.055,
                    "B" + std::to_string(target), color);
        }
    }

    void buildAndDrawPaths(visualization_msgs::MarkerArray& arr,
                           int& id,
                           const Slot& depot,
                           const std::vector<int>& targets,
                           const std::string& depot_label,
                           bool to_depot,
                           const std_msgs::ColorRGBA& color,
                           double z_offset) const {
        int ok = 0;
        int failed = 0;
        for (int target : targets) {
            PathGenerationInfo info;
            const Slot& slot = map_->slots().at(static_cast<size_t>(target));
            const Slot& src = to_depot ? slot : depot;
            const Slot& dst = to_depot ? depot : slot;
            const std::string route_label = to_depot
                ? ("B" + std::to_string(target) + "_to_" + depot_label)
                : (depot_label + "_to_B" + std::to_string(target));
            RoughPath path =
                activeGenerator(depot_label, to_depot).generate(src, dst, &info);
            if (path.size() < 2) {
                ++failed;
                ROS_ERROR("[path_catalog] %s rejected: empty_path",
                          route_label.c_str());
                continue;
            }
            if (validate_paths_) {
                const DebugRejectReason reject = validatePath(path, info, src, dst);
                if (reject != DebugRejectReason::NONE) {
                    ++failed;
                    ROS_ERROR("[path_catalog] %s rejected: %s",
                              route_label.c_str(), rejectReasonName(reject));
                    if (verbose_batch_paths_) {
                        logRejectDetails(route_label, path, src, dst, reject);
                    }
                    if (visualize_rejections_) {
                        addPath(arr, pp_.frame_id, depot_label + "_rejected_B",
                                id++, path, rgba(1.0f, 0.1f, 0.1f, 0.22f),
                                z_offset + 0.018);
                        addRejectionDiagnostics(
                            arr, id, path, info, src, dst, reject,
                            route_label);
                    }
                    continue;
                }
            }
            ++ok;
            addPath(arr, pp_.frame_id, depot_label + "_to_B", id++, path, color,
                    z_offset);
            if (verbose_batch_paths_) {
                ROS_INFO("[path_catalog] %s%s row=%d col=%d wpts=%zu len=%.3f arc=%d",
                         validate_paths_ ? "" : "RAW ",
                         route_label.c_str(), slot.row_id, slot.col,
                         path.size(), pathLength(path),
                         info.used_arc_fallback ? 1 : 0);
            }
        }
        ROS_WARN("[path_catalog] %s%s generated %d/%zu paths, failed=%d",
                 validate_paths_ ? "" : "RAW ",
                 depot_label.c_str(), ok, targets.size(), failed);
    }

    bool poseInSlotSweep(const RoughWp& pose, const Slot& slot) const {
        const double ax = slot.dock_x();
        const double ay = slot.dock_y();
        const double bx = slot.pre_dock_x;
        const double by = slot.pre_dock_y;
        const double vx = bx - ax;
        const double vy = by - ay;
        const double len = std::hypot(vx, vy);
        if (len < 1e-6) return false;

        const double ux = vx / len;
        const double uy = vy / len;
        const double dx = pose.x - ax;
        const double dy = pose.y - ay;
        const double longitudinal = dx * ux + dy * uy;
        const double lateral = -dx * uy + dy * ux;

        const double long_margin = mp_.vehicle_length * 0.65;
        const double lat_margin = mp_.vehicle_width * 0.60;
        return longitudinal >= -long_margin &&
               longitudinal <= len + long_margin &&
               std::abs(lateral) <= lat_margin;
    }

    bool footprintCollidesWithShelf(const RoughWp& pose,
                                    const Slot& src,
                                    const Slot& target) const {
        if (poseInSlotSweep(pose, src) || poseInSlotSweep(pose, target)) {
            return false;
        }
        for (const ShelfBlock& shelf : map_->shelf_blocks()) {
            if (forklift_planner::multi_vehicle::footprintIntersectsShelf(
                    pose, shelf, mp_, 0.0)) {
                return true;
            }
        }
        return false;
    }

    RoughWp interpolatePose(const RoughWp& a, const RoughWp& b,
                            double ratio) const {
        return {a.x + (b.x - a.x) * ratio,
                a.y + (b.y - a.y) * ratio,
                angleLerp(a.theta, b.theta, ratio),
                ratio < 0.5 ? a.type : b.type};
    }

    DebugRejectReason validatePose(const RoughWp& pose,
                                   const Slot& src,
                                   const Slot& target) const {
        if (cfg_.reject_boundary_violations &&
            !forklift_planner::multi_vehicle::footprintInsideField(
                pose, mp_, 0.0)) {
            return DebugRejectReason::FOOTPRINT_OUT_OF_BOUNDS;
        }
        if (cfg_.reject_shelf_collisions &&
            footprintCollidesWithShelf(pose, src, target)) {
            return DebugRejectReason::SHELF_COLLISION;
        }
        return DebugRejectReason::NONE;
    }

    DebugRejectReason validatePath(const RoughPath& path,
                                   const PathGenerationInfo& info,
                                   const Slot& src,
                                   const Slot& target) const {
        if (path.size() < 2) return DebugRejectReason::EMPTY_PATH;
        if (cfg_.reject_curvature_discontinuity && info.used_arc_fallback) {
            return DebugRejectReason::CURVATURE_DISCONTINUITY;
        }

        if (cfg_.reject_path_kinks) {
            const double kink_min_angle = info.used_arc_fallback
                ? std::max(cfg_.kink_min_angle, kPi / 3.0)
                : cfg_.kink_min_angle;
            auto legal_pose_flip_at = [&](size_t idx) {
                bool have_prev_seg = false;
                bool have_next_seg = false;
                double before_dx = 0.0;
                double before_dy = 0.0;
                double before_len = 0.0;
                double after_dx = 0.0;
                double after_dy = 0.0;
                double after_len = 0.0;
                WpType before_type = path.front().type;
                WpType after_type = path.front().type;

                for (size_t j = idx; j > 0; --j) {
                    const double dx = path[j].x - path[j - 1].x;
                    const double dy = path[j].y - path[j - 1].y;
                    const double len = std::hypot(dx, dy);
                    if (len < 1e-4) continue;
                    before_dx = dx;
                    before_dy = dy;
                    before_len = len;
                    before_type = path[j].type;
                    have_prev_seg = true;
                    break;
                }
                for (size_t j = idx; j + 1 < path.size(); ++j) {
                    const double dx = path[j + 1].x - path[j].x;
                    const double dy = path[j + 1].y - path[j].y;
                    const double len = std::hypot(dx, dy);
                    if (len < 1e-4) continue;
                    after_dx = dx;
                    after_dy = dy;
                    after_len = len;
                    after_type = path[j + 1].type;
                    have_next_seg = true;
                    break;
                }
                if (!have_prev_seg || !have_next_seg ||
                    before_type == after_type) {
                    return false;
                }
                double c = (before_dx * after_dx + before_dy * after_dy) /
                           (before_len * after_len);
                c = std::max(-1.0, std::min(1.0, c));
                return std::acos(c) >= cfg_.kink_cusp_angle;
            };

            for (size_t i = 1; i < path.size(); ++i) {
                if (angleDiffAbs(path[i - 1].theta, path[i].theta) <=
                    kink_min_angle) {
                    continue;
                }
                if (!legal_pose_flip_at(i)) {
                    ROS_WARN("[path_catalog] kink pose jump at i=%zu "
                             "pose=(%.3f, %.3f) prev_theta=%.1fdeg theta=%.1fdeg",
                             i, path[i].x, path[i].y,
                             path[i - 1].theta * 180.0 / kPi,
                             path[i].theta * 180.0 / kPi);
                    return DebugRejectReason::KINK;
                }
            }

            bool have_prev = false;
            double prev_dx = 0.0;
            double prev_dy = 0.0;
            double prev_len = 0.0;
            WpType prev_type = path.front().type;
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                const double dx = path[i + 1].x - path[i].x;
                const double dy = path[i + 1].y - path[i].y;
                const double len = std::hypot(dx, dy);
                if (len < 1e-4) continue;

                const WpType type = path[i + 1].type;
                if (!have_prev) {
                    prev_dx = dx;
                    prev_dy = dy;
                    prev_len = len;
                    prev_type = type;
                    have_prev = true;
                    continue;
                }

                double c = (prev_dx * dx + prev_dy * dy) / (prev_len * len);
                c = std::max(-1.0, std::min(1.0, c));
                const double ang = std::acos(c);
                const bool legal_reverse_cusp =
                    prev_type != type && ang >= cfg_.kink_cusp_angle;
                if (ang > kink_min_angle && !legal_reverse_cusp) {
                    ROS_WARN("[path_catalog] kink geometry at segment %zu "
                             "turn=%.1fdeg prev_type=%d type=%d "
                             "prev_vec=(%.3f, %.3f) vec=(%.3f, %.3f)",
                             i, ang * 180.0 / kPi,
                             static_cast<int>(prev_type),
                             static_cast<int>(type),
                             prev_dx, prev_dy, dx, dy);
                    return DebugRejectReason::KINK;
                }

                const double prev_motion =
                    std::atan2(prev_dy, prev_dx);
                const double motion = std::atan2(dy, dx);
                const double prev_body =
                    prev_type == WpType::REVERSE
                        ? normAngle(prev_motion + kPi)
                        : prev_motion;
                const double body =
                    type == WpType::REVERSE ? normAngle(motion + kPi) : motion;
                const bool body_flip =
                    angleDiffAbs(prev_body, body) > kink_min_angle;
                if (body_flip && !legal_reverse_cusp) {
                    ROS_WARN("[path_catalog] kink body flip at segment %zu "
                             "prev_body=%.1fdeg body=%.1fdeg "
                             "prev_type=%d type=%d",
                             i, prev_body * 180.0 / kPi,
                             body * 180.0 / kPi,
                             static_cast<int>(prev_type),
                             static_cast<int>(type));
                    return DebugRejectReason::KINK;
                }

                prev_dx = dx;
                prev_dy = dy;
                prev_len = len;
                prev_type = type;
            }
        }

        const double check_ds = std::max(0.005, cfg_.path_validation_step);
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const double seg_len = std::hypot(path[i + 1].x - path[i].x,
                                              path[i + 1].y - path[i].y);
            const int steps =
                std::max(1, static_cast<int>(std::ceil(seg_len / check_ds)));
            for (int k = 0; k < steps; ++k) {
                const double ratio =
                    static_cast<double>(k) / static_cast<double>(steps);
                const DebugRejectReason reason =
                    validatePose(interpolatePose(path[i], path[i + 1], ratio),
                                 src, target);
                if (reason != DebugRejectReason::NONE) return reason;
            }
        }
        return validatePose(path.back(), src, target);
    }

    bool collidingShelfAtPose(const RoughWp& pose,
                              const Slot& src,
                              const Slot& target,
                              ShelfBlock* out_shelf) const {
        if (poseInSlotSweep(pose, src) || poseInSlotSweep(pose, target)) {
            return false;
        }
        for (const ShelfBlock& shelf : map_->shelf_blocks()) {
            if (forklift_planner::multi_vehicle::footprintIntersectsShelf(
                    pose, shelf, mp_, 0.0)) {
                if (out_shelf != nullptr) *out_shelf = shelf;
                return true;
            }
        }
        return false;
    }

    bool findFirstInvalidPose(const RoughPath& path,
                              const Slot& src,
                              const Slot& target,
                              DebugRejectReason reason,
                              RoughWp* out_pose,
                              ShelfBlock* out_shelf,
                              size_t* out_segment = nullptr,
                              double* out_ratio = nullptr) const {
        const double check_ds = std::max(0.005, cfg_.path_validation_step);
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const double seg_len = std::hypot(path[i + 1].x - path[i].x,
                                              path[i + 1].y - path[i].y);
            const int steps =
                std::max(1, static_cast<int>(std::ceil(seg_len / check_ds)));
            for (int k = 0; k <= steps; ++k) {
                const double ratio =
                    static_cast<double>(k) / static_cast<double>(steps);
                const RoughWp pose = interpolatePose(path[i], path[i + 1], ratio);
                if (reason == DebugRejectReason::FOOTPRINT_OUT_OF_BOUNDS &&
                    !forklift_planner::multi_vehicle::footprintInsideField(
                        pose, mp_, 0.0)) {
                    if (out_pose != nullptr) *out_pose = pose;
                    if (out_segment != nullptr) *out_segment = i;
                    if (out_ratio != nullptr) *out_ratio = ratio;
                    return true;
                }
                if (reason == DebugRejectReason::SHELF_COLLISION &&
                    collidingShelfAtPose(pose, src, target, out_shelf)) {
                    if (out_pose != nullptr) *out_pose = pose;
                    if (out_segment != nullptr) *out_segment = i;
                    if (out_ratio != nullptr) *out_ratio = ratio;
                    return true;
                }
            }
        }
        return false;
    }

    void logRejectDetails(const std::string& label,
                          const RoughPath& path,
                          const Slot& src,
                          const Slot& target,
                          DebugRejectReason reason) const {
        const bool to_depot_mode = uppercase(direction_) == "TO_DEPOT";
        const Slot& focus = (to_depot_mode && src.id >= 0) ? src : target;
        const bool focused_target =
            (focus.row_id == 1 || focus.row_id == 5 || to_depot_mode);
        if (!focused_target && target_slot_ < 0) return;

        if (reason == DebugRejectReason::CURVATURE_DISCONTINUITY ||
            reason == DebugRejectReason::KINK) {
            RoughWp prev;
            RoughWp mid;
            RoughWp next;
            double angle = 0.0;
            if (!findSharpestPathPoint(path, &mid, &prev, &next, &angle)) {
                ROS_WARN("[path_catalog][reject-detail] %s B%d %s: "
                         "sharp point not found",
                         label.c_str(), focus.id, rejectReasonName(reason));
                return;
            }
            auto modeName = [](WpType type) {
                return type == WpType::REVERSE ? "REVERSE" : "FORWARD";
            };
            ROS_WARN("[path_catalog][reject-detail] %s B%d row=%d col=%d "
                     "%s sharp_turn=%.1fdeg "
                     "prev=(%.3f,%.3f,%.1fdeg,%s) "
                     "mid=(%.3f,%.3f,%.1fdeg,%s) "
                     "next=(%.3f,%.3f,%.1fdeg,%s)",
                     label.c_str(), focus.id, focus.row_id, focus.col,
                     rejectReasonName(reason), angle * 180.0 / kPi,
                     prev.x, prev.y, prev.theta * 180.0 / kPi,
                     modeName(prev.type),
                     mid.x, mid.y, mid.theta * 180.0 / kPi,
                     modeName(mid.type),
                     next.x, next.y, next.theta * 180.0 / kPi,
                     modeName(next.type));
            return;
        }

        if (reason != DebugRejectReason::SHELF_COLLISION &&
            reason != DebugRejectReason::FOOTPRINT_OUT_OF_BOUNDS) {
            return;
        }

        RoughWp bad_pose;
        ShelfBlock bad_shelf;
        size_t segment = 0;
        double ratio = 0.0;
        if (!findFirstInvalidPose(path, src, target, reason, &bad_pose,
                                  &bad_shelf, &segment, &ratio)) {
            ROS_WARN("[path_catalog][reject-detail] %s B%d %s: "
                     "first invalid pose not found",
                     label.c_str(), focus.id, rejectReasonName(reason));
            return;
        }

        const RoughWp center =
            forklift_planner::multi_vehicle::bodyCenterPose(bad_pose, mp_);
        const auto corners =
            forklift_planner::multi_vehicle::footprintCorners(bad_pose, mp_, 0.0);
        const char* motion =
            bad_pose.type == WpType::REVERSE ? "REVERSE" : "FORWARD";

        if (reason == DebugRejectReason::SHELF_COLLISION) {
            ROS_WARN("[path_catalog][reject-detail] %s B%d row=%d col=%d "
                     "shelf_collision seg=%zu ratio=%.3f motion=%s "
                     "rear=(%.3f,%.3f,%.1fdeg) body=(%.3f,%.3f) "
                     "shelf=[x %.3f..%.3f y %.3f..%.3f]",
                     label.c_str(), focus.id, focus.row_id, focus.col,
                     segment, ratio, motion,
                     bad_pose.x, bad_pose.y, bad_pose.theta * 180.0 / kPi,
                     center.x, center.y,
                     bad_shelf.x, bad_shelf.x_max(),
                     bad_shelf.y, bad_shelf.y_max());
        } else {
            ROS_WARN("[path_catalog][reject-detail] %s B%d row=%d col=%d "
                     "footprint_out_of_bounds seg=%zu ratio=%.3f motion=%s "
                     "rear=(%.3f,%.3f,%.1fdeg) body=(%.3f,%.3f) "
                     "field=[0..%.3f, 0..%.3f]",
                     label.c_str(), focus.id, focus.row_id, focus.col,
                     segment, ratio, motion,
                     bad_pose.x, bad_pose.y, bad_pose.theta * 180.0 / kPi,
                     center.x, center.y, mp_.field_width, mp_.field_height);
        }

        ROS_WARN("[path_catalog][reject-detail] %s B%d footprint corners: "
                 "(%.3f,%.3f) (%.3f,%.3f) (%.3f,%.3f) (%.3f,%.3f)",
                 label.c_str(), focus.id,
                 corners[0].x, corners[0].y,
                 corners[1].x, corners[1].y,
                 corners[2].x, corners[2].y,
                 corners[3].x, corners[3].y);
    }

    bool findSharpestPathPoint(const RoughPath& path,
                               RoughWp* out_pose,
                               RoughWp* out_prev,
                               RoughWp* out_next,
                               double* out_angle) const {
        if (path.size() < 3) return false;
        bool have_prev = false;
        RoughWp prev_a = path.front();
        double prev_dx = 0.0;
        double prev_dy = 0.0;
        double prev_len = 0.0;
        WpType prev_type = path.front().type;
        double best_ang = -1.0;
        RoughWp best_prev = path.front();
        RoughWp best_mid = path.front();
        RoughWp best_next = path.front();

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const double dx = path[i + 1].x - path[i].x;
            const double dy = path[i + 1].y - path[i].y;
            const double len = std::hypot(dx, dy);
            if (len < 1e-4) continue;

            const WpType type = path[i + 1].type;
            if (have_prev) {
                double c = (prev_dx * dx + prev_dy * dy) / (prev_len * len);
                c = std::max(-1.0, std::min(1.0, c));
                const double ang = std::acos(c);
                const bool legal_reverse_cusp =
                    prev_type != type && ang >= cfg_.kink_cusp_angle;
                if (!legal_reverse_cusp && ang > best_ang) {
                    best_ang = ang;
                    best_prev = prev_a;
                    best_mid = path[i];
                    best_next = path[i + 1];
                }
            }

            prev_a = path[i];
            prev_dx = dx;
            prev_dy = dy;
            prev_len = len;
            prev_type = type;
            have_prev = true;
        }
        if (best_ang < 0.0) return false;
        if (out_pose != nullptr) *out_pose = best_mid;
        if (out_prev != nullptr) *out_prev = best_prev;
        if (out_next != nullptr) *out_next = best_next;
        if (out_angle != nullptr) *out_angle = best_ang;
        return true;
    }

    void addFootprintBox(visualization_msgs::MarkerArray& arr,
                         int& id,
                         const std::string& ns,
                         const RoughWp& pose,
                         const std_msgs::ColorRGBA& color,
                         double z_offset) const {
        const auto corners =
            forklift_planner::multi_vehicle::footprintCorners(pose, mp_, 0.0);
        std::vector<geometry_msgs::Point> pts;
        pts.reserve(5);
        for (const auto& c : corners) {
            pts.push_back(point(c.x, c.y, z_offset));
        }
        pts.push_back(point(corners.front().x, corners.front().y, z_offset));
        addLineStrip(arr, pp_.frame_id, ns, id++, pts, color, 0.018, z_offset);
    }

    void addShelfBox(visualization_msgs::MarkerArray& arr,
                     int& id,
                     const std::string& ns,
                     const ShelfBlock& shelf,
                     const std_msgs::ColorRGBA& color,
                     double z_offset) const {
        addLineStrip(arr, pp_.frame_id, ns, id++,
                     {point(shelf.x, shelf.y, z_offset),
                      point(shelf.x_max(), shelf.y, z_offset),
                      point(shelf.x_max(), shelf.y_max(), z_offset),
                      point(shelf.x, shelf.y_max(), z_offset),
                      point(shelf.x, shelf.y, z_offset)},
                     color, 0.022, z_offset);
    }

    void addRejectionDiagnostics(visualization_msgs::MarkerArray& arr,
                                 int& id,
                                 const RoughPath& path,
                                 const PathGenerationInfo& info,
                                 const Slot& src,
                                 const Slot& target,
                                 DebugRejectReason reason,
                                 const std::string& label) const {
        if (!visualize_rejections_ || path.size() < 2) return;
        const double z = 0.145;

        if (reason == DebugRejectReason::SHELF_COLLISION ||
            reason == DebugRejectReason::FOOTPRINT_OUT_OF_BOUNDS) {
            RoughWp bad_pose;
            ShelfBlock bad_shelf;
            const bool found =
                findFirstInvalidPose(path, src, target, reason,
                                     &bad_pose, &bad_shelf);
            if (!found) return;
            addSphere(arr, pp_.frame_id, "reject_pose", id++,
                      bad_pose.x, bad_pose.y, rgba(1.0f, 0.0f, 0.0f, 0.95f));
            addFootprintBox(arr, id, "reject_footprint",
                            bad_pose, rgba(1.0f, 0.0f, 0.0f, 0.95f), z);
            if (reason == DebugRejectReason::SHELF_COLLISION) {
                addShelfBox(arr, id, "reject_shelf",
                            bad_shelf, rgba(1.0f, 0.9f, 0.0f, 0.95f), z + 0.012);
            }
            addText(arr, pp_.frame_id, "reject_label", id++,
                    bad_pose.x, bad_pose.y, z + 0.07, 0.055,
                    label + " " + rejectReasonName(reason),
                    rgba(1.0f, 0.1f, 0.1f, 1.0f));
            return;
        }

        if (reason == DebugRejectReason::CURVATURE_DISCONTINUITY ||
            reason == DebugRejectReason::KINK) {
            RoughWp prev;
            RoughWp mid;
            RoughWp next;
            double angle = 0.0;
            if (!findSharpestPathPoint(path, &mid, &prev, &next, &angle)) return;
            const std_msgs::ColorRGBA color =
                reason == DebugRejectReason::CURVATURE_DISCONTINUITY
                    ? rgba(0.95f, 0.1f, 1.0f, 0.95f)
                    : rgba(1.0f, 0.55f, 0.0f, 0.95f);
            addLineStrip(arr, pp_.frame_id, "reject_sharp_segment", id++,
                         {point(prev.x, prev.y, z),
                          point(mid.x, mid.y, z),
                          point(next.x, next.y, z)},
                         color, 0.028, z);
            addSphere(arr, pp_.frame_id, "reject_sharp_point", id++,
                      mid.x, mid.y, color);
            std::ostringstream ss;
            ss << label << " " << rejectReasonName(reason);
            if (reason == DebugRejectReason::CURVATURE_DISCONTINUITY &&
                info.used_arc_fallback) {
                ss << " arc_fallback";
            }
            ss << " turn=" << std::fixed << std::setprecision(0)
               << angle * 180.0 / kPi << "deg";
            addText(arr, pp_.frame_id, "reject_label", id++,
                    mid.x, mid.y, z + 0.07, 0.052, ss.str(), color);
        }
    }

    void addPathLayerDiagnostics(visualization_msgs::MarkerArray& arr,
                                 int& id,
                                 const RoughPath& path,
                                 const PathGenerationInfo& info,
                                 const std::string& label) const {
        if (!visualize_path_layers_) return;
        forklift_map::path_debug::PathDebugVisualOptions options;
        options.show_layers = true;
        options.show_turn_types = visualize_turn_types_;
        options.show_corner_labels = true;
        options.z_offset = 0.135;
        forklift_map::path_debug::addPathDebugLayers(
            arr, id, pp_.frame_id, "single_path", path, info, label, options);
    }

    void logPathGenerationDebug(const std::string& label,
                                const PathGenerationInfo& info) const {
        if (target_slot_ < 0) return;
        ROS_WARN("[path_catalog][debug-layers] %s layers=%zu arc=%d",
                 label.c_str(), info.debug_layers.size(),
                 info.used_arc_fallback ? 1 : 0);
        for (size_t i = 0; i < info.debug_layers.size(); ++i) {
            const DebugPathLayer& layer = info.debug_layers[i];
            ROS_WARN("[path_catalog][debug-layers] layer[%zu] type=%s label=%s pts=%zu",
                     i, debugLayerTypeName(layer.type),
                     layer.label.c_str(), layer.points.size());
            if (layer.type != DebugPathLayerType::SKELETON) continue;
            for (size_t j = 0; j < layer.points.size(); ++j) {
                const DebugPathPoint& p = layer.points[j];
                ROS_WARN("[path_catalog][debug-layers] S%zu=(%.3f,%.3f)",
                         j, p.x, p.y);
            }
        }
    }

    void onAnimationTimer(const ros::TimerEvent&) {
        if (selected_path_.empty()) return;
        const double len = pathLength(selected_path_);
        if (len <= 1e-9) return;
        const double elapsed =
            std::max(0.0, (ros::Time::now() - animation_start_).toSec());
        const double period = std::max(0.5, animation_duration_);
        const double phase = std::fmod(elapsed, period) / period;
        publishVehicleMarker(poseAtS(selected_path_, len * phase));
    }

    void publishVehicleMarker(const RoughWp& ref) {
        visualization_msgs::MarkerArray arr = static_markers_;

        RoughWp center = ref;
        center.x += mp_.rear_axle_to_center * std::cos(ref.theta);
        center.y += mp_.rear_axle_to_center * std::sin(ref.theta);

        auto body = baseMarker(pp_.frame_id, "single_vehicle_body", 0);
        body.type = visualization_msgs::Marker::CUBE;
        body.pose.position.x = center.x;
        body.pose.position.y = center.y;
        body.pose.position.z = 0.13;
        body.pose.orientation.z = std::sin(center.theta * 0.5);
        body.pose.orientation.w = std::cos(center.theta * 0.5);
        body.scale.x = mp_.vehicle_length;
        body.scale.y = mp_.vehicle_width;
        body.scale.z = 0.045;
        body.color = ref.type == WpType::REVERSE
            ? rgba(1.0f, 0.25f, 0.25f, 0.85f)
            : rgba(0.2f, 1.0f, 0.35f, 0.85f);
        arr.markers.push_back(body);

        auto arrow = baseMarker(pp_.frame_id, "single_vehicle_heading", 1);
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.points.push_back(point(center.x, center.y, 0.18));
        arrow.points.push_back(point(center.x + 0.18 * std::cos(center.theta),
                                     center.y + 0.18 * std::sin(center.theta), 0.18));
        arrow.scale.x = 0.018;
        arrow.scale.y = 0.045;
        arrow.scale.z = 0.055;
        arrow.color = rgba(1.0f, 1.0f, 1.0f, 1.0f);
        arr.markers.push_back(arrow);

        pub_.publish(arr);
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;
    visualization_msgs::MarkerArray static_markers_;
    MapParam mp_;
    PlannerParam pp_;
    forklift_planner::multi_vehicle::MultiVehicleConfig cfg_;
    std::unique_ptr<ForkliftMap> map_;
    std::unique_ptr<PathGenerator> generator_a1_to_b_;
    std::unique_ptr<PathGenerator> generator_b_to_a1_;
    std::unique_ptr<PathGenerator> generator_a2_to_b_;
    std::unique_ptr<PathGenerator> generator_b_to_a2_;

    bool use_exact_midpoints_ = true;
    double a1_x_ = 1.25;
    double a1_y_ = 4.38;
    double a1_pre_x_ = 1.25;
    double a1_pre_y_ = 4.105;
    double a1_yaw_ = kPi * 0.5;
    double a2_x_ = 1.25;
    double a2_y_ = 0.12;
    double a2_pre_x_ = 1.25;
    double a2_pre_y_ = 0.395;
    double a2_yaw_ = -kPi * 0.5;

    std::string depot_name_ = "A1";
    std::string direction_ = "from_depot";
    int target_slot_ = -1;
    int target_row_ = -1;
    bool validate_paths_ = true;
    bool visualize_rejections_ = true;
    bool visualize_path_layers_ = false;
    bool visualize_turn_types_ = true;
    bool verbose_batch_paths_ = false;
    bool animate_ = true;
    double animation_period_ = 0.05;
    double animation_duration_ = 8.0;
    ros::Time animation_start_;
    RoughPath selected_path_;
    std::string selected_label_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_catalog_debug_node");
    PathCatalogDebugNode node;
    ros::spin();
    return 0;
}
