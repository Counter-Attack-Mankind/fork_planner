#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_nlp_recovery/coordinate_mapper.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

std_msgs::ColorRGBA color(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA value;
    value.r = r;
    value.g = g;
    value.b = b;
    value.a = a;
    return value;
}

geometry_msgs::Point point(double x, double y, double z = 0.0) {
    geometry_msgs::Point value;
    value.x = x;
    value.y = y;
    value.z = z;
    return value;
}

visualization_msgs::Marker marker(const std::string& frame,
                                  const std::string& ns, int id, int type) {
    visualization_msgs::Marker value;
    value.header.frame_id = frame;
    value.header.stamp = ros::Time::now();
    value.ns = "coordinate_mapping/" + ns;
    value.id = id;
    value.type = type;
    value.action = visualization_msgs::Marker::ADD;
    value.pose.orientation.w = 1.0;
    return value;
}

void addRectangle(visualization_msgs::MarkerArray& markers,
                  const std::string& frame, const std::string& ns, int id,
                  const Aabb& box, double z, double width,
                  const std_msgs::ColorRGBA& marker_color) {
    auto value = marker(frame, ns, id, visualization_msgs::Marker::LINE_STRIP);
    value.scale.x = width;
    value.color = marker_color;
    value.points = {point(box.min_x, box.min_y, z), point(box.max_x, box.min_y, z),
                    point(box.max_x, box.max_y, z), point(box.min_x, box.max_y, z),
                    point(box.min_x, box.min_y, z)};
    markers.markers.push_back(std::move(value));
}

void addRoundedInflatedShelf(visualization_msgs::MarkerArray& markers,
                             const std::string& frame, int id,
                             const Aabb& shelf, double inflation) {
    auto value = marker(frame, "inflated_shelves", id,
                        visualization_msgs::Marker::LINE_STRIP);
    value.scale.x = 0.012;
    value.color = color(0.85f, 0.15f, 0.85f, 0.9f);
    const double centers_x[4] = {shelf.max_x, shelf.min_x, shelf.min_x, shelf.max_x};
    const double centers_y[4] = {shelf.max_y, shelf.max_y, shelf.min_y, shelf.min_y};
    const double start_angle[4] = {0.0, M_PI * 0.5, M_PI, M_PI * 1.5};
    if (inflation <= kEpsilon) {
        addRectangle(markers, frame, "inflated_shelves", id, shelf, 0.045, 0.012,
                     value.color);
        return;
    }
    for (int corner = 0; corner < 4; ++corner) {
        for (int sample = 0; sample <= 8; ++sample) {
            const double angle = start_angle[corner] + sample * M_PI / 16.0;
            value.points.push_back(point(centers_x[corner] + inflation * std::cos(angle),
                                         centers_y[corner] + inflation * std::sin(angle),
                                         0.045));
        }
    }
    value.points.push_back(value.points.front());
    markers.markers.push_back(std::move(value));
}

std::string trim(const std::string& value) {
    const size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

bool startsWith(const std::string& value, const std::string& prefix) {
    return value.compare(0, prefix.size(), prefix) == 0;
}

bool loadCatalogLeg(const std::string& file, const std::string& direction,
                    int slot_id, RoughPath& path, std::string& reason) {
    std::ifstream input(file);
    if (!input) {
        reason = "cannot open offline catalog: " + file;
        return false;
    }
    std::string active_section;
    int active_leg = -1;
    bool format_ok = false;
    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty() || line.front() == '#') continue;
        if (startsWith(line, "format:")) {
            format_ok = trim(line.substr(7)) == "forklift_a1_cycle_path_catalog_v1";
        } else if (startsWith(line, "section:")) {
            active_section = trim(line.substr(8));
            active_leg = -1;
        } else if (startsWith(line, "leg:")) {
            std::string tag;
            std::istringstream stream(line);
            stream >> tag >> active_leg;
        } else if (startsWith(line, "p:") && active_section == direction &&
                   active_leg == slot_id) {
            std::string tag;
            int type = 0;
            RoughWp waypoint{};
            std::istringstream stream(line);
            stream >> tag >> waypoint.x >> waypoint.y >> waypoint.theta >> type;
            if (!stream || (type != 0 && type != 1)) {
                reason = "invalid waypoint in offline catalog";
                return false;
            }
            waypoint.type = static_cast<WpType>(type);
            path.push_back(waypoint);
        }
    }
    if (!format_ok) {
        reason = "offline catalog format is not forklift_a1_cycle_path_catalog_v1";
        return false;
    }
    if (path.size() < 2) {
        reason = "selected route/slot is absent or has fewer than two points";
        return false;
    }
    reason.clear();
    return true;
}

Pose2d interpolateRearPose(const std::vector<PathPoint>& path, double query_s) {
    if (query_s <= path.front().s) return path.front().pose;
    if (query_s >= path.back().s) return path.back().pose;
    const auto upper = std::upper_bound(
        path.begin(), path.end(), query_s,
        [](double value, const PathPoint& candidate) { return value < candidate.s; });
    const PathPoint& second = *upper;
    const PathPoint& first = *(upper - 1);
    const double ratio = (query_s - first.s) / (second.s - first.s);
    const double yaw_delta = std::atan2(std::sin(second.pose.yaw - first.pose.yaw),
                                        std::cos(second.pose.yaw - first.pose.yaw));
    return {first.pose.x + ratio * (second.pose.x - first.pose.x),
            first.pose.y + ratio * (second.pose.y - first.pose.y),
            first.pose.yaw + ratio * yaw_delta};
}

std::string aabbList(const std::vector<size_t>& boxes) {
    if (boxes.empty()) return "none";
    std::ostringstream out;
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (i > 0) out << ',';
        out << boxes[i];
    }
    return out.str();
}

double signedPointToAabbDistance(const Pose2d& value, const Aabb& box) {
    const double dx = std::max({box.min_x - value.x, 0.0, value.x - box.max_x});
    const double dy = std::max({box.min_y - value.y, 0.0, value.y - box.max_y});
    if (dx > 0.0 || dy > 0.0) return std::hypot(dx, dy);
    return -std::min({value.x - box.min_x, box.max_x - value.x,
                      value.y - box.min_y, box.max_y - value.y});
}

class CoordinateMappingVisualizer {
public:
    CoordinateMappingVisualizer() : private_nh_("~") {
        ros::NodeHandle global_nh;
        MapParam map_param = MapParam::fromROSParam(global_nh);
        PlannerParam planner_param = PlannerParam::fromROSParam(global_nh);
        map_ = std::make_unique<ForkliftMap>(map_param);
        frame_ = map_param.frame_id;

        CorridorConfig corridor_config;
        private_nh_.param("vehicle_radius", corridor_config.vehicle_radius,
                          corridor_config.vehicle_radius);
        private_nh_.param("safety_margin", corridor_config.safety_margin,
                          corridor_config.safety_margin);
        private_nh_.param("expansion_step", corridor_config.expansion_step,
                          corridor_config.expansion_step);
        private_nh_.param("rejoin_distance", corridor_config.rejoin_distance,
                          corridor_config.rejoin_distance);
        private_nh_.param("route", route_, std::string("a1_to_b"));
        private_nh_.param("slot_id", slot_id_, 0);
        private_nh_.param("catalog_file", catalog_file_, std::string());
        private_nh_.param("current_s", current_s_, -1.0);
        private_nh_.param("sweep_sample_step", sweep_sample_step_, 0.01);
        private_nh_.param("max_issue_labels", max_issue_labels_, 80);

        corridor_builder_ = std::make_unique<CorridorBuilder>(
            map_param, map_->shelf_blocks(), corridor_config);
        mapper_ = std::make_unique<CoordinateMapper>(map_param, *corridor_builder_);
        if (!loadPath(map_param, planner_param)) {
            ROS_FATAL("[coordinate_mapping] %s", error_.c_str());
            ros::shutdown();
            return;
        }
        path_ = CorridorBuilder::parameterizePath(rearPoses(), pathTypes());
        if (current_s_ < 0.0) {
            current_s_ = path_.front().s;
            for (const PathPoint& candidate : path_) {
                if (corridor_builder_->pointLegal(
                        corridor_builder_->bodyCenter(candidate.pose))) {
                    current_s_ = candidate.s;
                    break;
                }
            }
            ROS_INFO("[coordinate_mapping] auto-selected first legal current_s=%.6f",
                     current_s_);
        }
        current_s_ = std::clamp(current_s_, 0.0, path_.back().s);
        const Pose2d start = interpolateRearPose(path_, current_s_);
        corridor_ = corridor_builder_->build(start, path_, current_s_, 0);
        mapped_ = mapper_->mapPath(path_);
        diagnostics_ = mapper_->diagnosePath(mapped_, corridor_.boxes);
        swept_ = mapper_->diagnoseSweptPath(path_, corridor_.boxes,
                                            sweep_sample_step_);
        buildMarkers();
        publishDiagnostics();
        publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_nlp_recovery/coordinate_mapping/markers", 1, true);
        timer_ = private_nh_.createTimer(
            ros::Duration(1.0), &CoordinateMappingVisualizer::publish, this);
        publish(ros::TimerEvent{});
    }

private:
    bool loadPath(const MapParam& map_param, const PlannerParam& planner_param) {
        if (route_ != "a1_to_b" && route_ != "b_to_a1") {
            error_ = "route must be a1_to_b or b_to_a1";
            return false;
        }
        if (slot_id_ < 0 || static_cast<size_t>(slot_id_) >= map_->slots().size()) {
            error_ = "slot_id is outside the current forklift_map slot range";
            return false;
        }
        if (!catalog_file_.empty()) {
            return loadCatalogLeg(catalog_file_, route_, slot_id_, rough_path_, error_);
        }

        ros::NodeHandle global;
        Slot a1;
        a1.id = 101;
        a1.row_id = 0;
        a1.col = -1;
        global.param("forklift_planner/multi_vehicle/a1_pickup_center_x", a1.cx, 1.250);
        global.param("forklift_planner/multi_vehicle/a1_pickup_center_y", a1.cy, 4.375);
        global.param("forklift_planner/multi_vehicle/a1_pre_dock_x", a1.pre_dock_x, 1.250);
        global.param("forklift_planner/multi_vehicle/a1_pre_dock_y", a1.pre_dock_y, 4.100);
        global.param("forklift_planner/multi_vehicle/a1_pickup_theta", a1.dock_theta,
                     M_PI * 0.5);
        const Slot& slot = map_->slots().at(slot_id_);
        const PathGeneratorRouteMode mode = route_ == "a1_to_b"
            ? PathGeneratorRouteMode::A1_TO_B : PathGeneratorRouteMode::B_TO_A1;
        PathGenerator generator(map_param, planner_param, mode);
        rough_path_ = route_ == "a1_to_b" ? generator.generate(a1, slot)
                                           : generator.generate(slot, a1);
        if (rough_path_.size() < 2) {
            error_ = "path generator returned fewer than two points";
            return false;
        }
        return true;
    }

    std::vector<Pose2d> rearPoses() const {
        std::vector<Pose2d> poses;
        poses.reserve(rough_path_.size());
        for (const RoughWp& point : rough_path_) {
            poses.push_back({point.x, point.y, point.theta});
        }
        return poses;
    }

    std::vector<uint8_t> pathTypes() const {
        std::vector<uint8_t> types;
        types.reserve(rough_path_.size());
        for (const RoughWp& point : rough_path_) {
            types.push_back(static_cast<uint8_t>(point.type));
        }
        return types;
    }

    bool inCorridorWindow(double source_s) const {
        return corridor_.success && source_s + kEpsilon >= current_s_ &&
               source_s <= corridor_.terminal.path_s + kEpsilon;
    }

    bool isRed(const GeometryDiagnostic& diagnostic) const {
        return diagnostic.disc_shelf_collision || diagnostic.disc_boundary_collision;
    }

    bool isOrange(const GeometryDiagnostic& diagnostic) const {
        return inCorridorWindow(diagnostic.source_s) && !isRed(diagnostic) &&
               diagnostic.containing_aabbs.empty();
    }

    void buildMarkers() {
        markers_.markers.clear();
        auto rear = marker(frame_, "rear_axle_path", 0,
                           visualization_msgs::Marker::LINE_STRIP);
        rear.scale.x = 0.025;
        rear.color = color(0.1f, 0.35f, 1.0f, 1.0f);
        auto center = marker(frame_, "body_center_path", 0,
                             visualization_msgs::Marker::LINE_STRIP);
        center.scale.x = 0.018;
        center.color = color(0.1f, 1.0f, 0.25f, 1.0f);
        for (const MappedPathPoint& path_point : mapped_) {
            rear.points.push_back(point(path_point.rear_axle_pose.x,
                                        path_point.rear_axle_pose.y, 0.075));
            center.points.push_back(point(path_point.body_center_pose.x,
                                          path_point.body_center_pose.y, 0.085));
        }
        markers_.markers.push_back(std::move(rear));
        markers_.markers.push_back(std::move(center));

        for (size_t i = 0; i < corridor_builder_->originalShelves().size(); ++i) {
            addRectangle(markers_, frame_, "real_shelves", static_cast<int>(i),
                         corridor_builder_->originalShelves()[i], 0.035, 0.01,
                         color(0.65f, 0.65f, 0.65f, 0.9f));
            addRoundedInflatedShelf(markers_, frame_, static_cast<int>(i),
                                    corridor_builder_->originalShelves()[i],
                                    corridor_builder_->shelfInflation());
        }
        for (size_t i = 0; i < corridor_.boxes.size(); ++i) {
            addRectangle(markers_, frame_, "corridor_aabbs", static_cast<int>(i),
                         corridor_.boxes[i], 0.065, 0.012,
                         color(0.1f, 0.9f, 0.95f, 0.85f));
        }

        int red_id = 0;
        int orange_id = 0;
        int label_id = 0;
        for (const GeometryDiagnostic& diagnostic : diagnostics_) {
            const bool red = isRed(diagnostic);
            const bool orange = isOrange(diagnostic);
            if (!red && !orange) continue;
            auto violation = marker(frame_, red ? "inflated_obstacle_intrusion"
                                                 : "aabb_coverage_gap",
                                    red ? red_id++ : orange_id++,
                                    visualization_msgs::Marker::SPHERE);
            violation.pose.position = point(diagnostic.body_center_pose.x,
                                            diagnostic.body_center_pose.y, 0.105);
            violation.scale.x = violation.scale.y = violation.scale.z = red ? 0.055 : 0.045;
            violation.color = red ? color(1.0f, 0.05f, 0.05f, 1.0f)
                                  : color(1.0f, 0.48f, 0.0f, 1.0f);
            markers_.markers.push_back(std::move(violation));
            if (label_id >= max_issue_labels_) continue;
            std::ostringstream text;
            text << "k=" << diagnostic.source_index
                 << " rear=(" << std::fixed << std::setprecision(3)
                 << diagnostic.rear_axle_pose.x << ',' << diagnostic.rear_axle_pose.y
                 << ") center=(" << diagnostic.body_center_pose.x << ','
                 << diagnostic.body_center_pose.y << ") AABB="
                 << aabbList(diagnostic.containing_aabbs) << " clr="
                 << diagnostic.minimum_inflated_shelf_clearance;
            auto label = marker(frame_, "issue_labels", label_id++,
                                visualization_msgs::Marker::TEXT_VIEW_FACING);
            label.pose.position = point(diagnostic.body_center_pose.x,
                                        diagnostic.body_center_pose.y, 0.16);
            label.scale.z = 0.035;
            label.color = red ? color(1.0f, 0.25f, 0.25f, 1.0f)
                              : color(1.0f, 0.65f, 0.05f, 1.0f);
            label.text = text.str();
            markers_.markers.push_back(std::move(label));
        }

        int swept_red_id = 0;
        int swept_orange_id = 0;
        for (const SweptGeometryDiagnostic& diagnostic : swept_) {
            const bool red = isRed(diagnostic.sample);
            const bool orange = isOrange(diagnostic.sample);
            if (!red && !orange) continue;
            auto sample = marker(frame_, red ? "swept_intrusion_samples"
                                              : "swept_aabb_gap_samples",
                                 red ? swept_red_id++ : swept_orange_id++,
                                 visualization_msgs::Marker::SPHERE);
            sample.pose.position = point(diagnostic.sample.body_center_pose.x,
                                         diagnostic.sample.body_center_pose.y, 0.095);
            sample.scale.x = sample.scale.y = sample.scale.z = 0.018;
            sample.color = red ? color(0.8f, 0.0f, 0.0f, 0.85f)
                               : color(1.0f, 0.45f, 0.0f, 0.8f);
            markers_.markers.push_back(std::move(sample));
        }
    }

    void publishDiagnostics() const {
        size_t rear_only_safe = 0;
        size_t center_intrusions = 0;
        size_t corridor_window_intrusions = 0;
        size_t corridor_gaps = 0;
        size_t model_disagreements = 0;
        size_t mapping_mismatches = 0;
        for (const GeometryDiagnostic& diagnostic : diagnostics_) {
            bool rear_inflated = false;
            for (const Aabb& shelf : corridor_builder_->originalShelves()) {
                rear_inflated = rear_inflated ||
                    signedPointToAabbDistance(diagnostic.rear_axle_pose, shelf) <
                        corridor_builder_->shelfInflation() - kEpsilon;
            }
            if (rear_inflated && !diagnostic.disc_shelf_collision) {
                ++rear_only_safe;
            }
            if (isRed(diagnostic)) ++center_intrusions;
            if (inCorridorWindow(diagnostic.source_s) && isRed(diagnostic)) {
                ++corridor_window_intrusions;
            }
            if (isOrange(diagnostic)) ++corridor_gaps;
            if (diagnostic.disc_shelf_collision != diagnostic.body_obb_shelf_collision ||
                diagnostic.disc_boundary_collision != diagnostic.body_obb_boundary_collision) {
                ++model_disagreements;
            }
            if (!diagnostic.mapping_consistent) ++mapping_mismatches;
            if (!isRed(diagnostic) && !isOrange(diagnostic)) continue;
            ROS_WARN_STREAM("[coordinate_mapping][point] route=" << route_
                << " slot=" << slot_id_ << " k=" << diagnostic.source_index
                << " s_rear=" << diagnostic.source_s << " rear=("
                << diagnostic.rear_axle_pose.x << ',' << diagnostic.rear_axle_pose.y
                << ") center=(" << diagnostic.body_center_pose.x << ','
                << diagnostic.body_center_pose.y << ") aabb="
                << aabbList(diagnostic.containing_aabbs) << " inflated_clearance="
                << diagnostic.minimum_inflated_shelf_clearance << " disc_collision="
                << isRed(diagnostic) << " obb_shelf_collision="
                << diagnostic.body_obb_shelf_collision);
        }
        size_t swept_intrusions = 0;
        size_t swept_window_intrusions = 0;
        size_t swept_gaps = 0;
        for (const SweptGeometryDiagnostic& diagnostic : swept_) {
            if (isRed(diagnostic.sample)) ++swept_intrusions;
            if (inCorridorWindow(diagnostic.sample.source_s) &&
                isRed(diagnostic.sample)) {
                ++swept_window_intrusions;
            }
            if (isOrange(diagnostic.sample)) ++swept_gaps;
        }
        ROS_INFO("[coordinate_mapping][summary] route=%s slot=%d source=%s points=%zu "
                 "rear_inflated_but_center_safe=%zu center_intrusions=%zu "
                 "window_intrusions=%zu corridor_gaps=%zu model_disagreements=%zu "
                 "mapping_mismatches=%zu "
                 "swept_intrusion_samples=%zu swept_window_intrusions=%zu "
                 "swept_gap_samples=%zu corridor=%s boxes=%zu "
                 "disc_radius=%.4f shelf_inflation=%.4f rear_axle_to_center=%.4f",
                 route_.c_str(), slot_id_, catalog_file_.empty() ? "generator" : "catalog",
                 diagnostics_.size(), rear_only_safe, center_intrusions,
                 corridor_window_intrusions, corridor_gaps, model_disagreements,
                 mapping_mismatches, swept_intrusions, swept_window_intrusions,
                 swept_gaps,
                 corridor_.reason.c_str(), corridor_.boxes.size(),
                 corridor_builder_->vehicleRadius(), corridor_builder_->shelfInflation(),
                 map_->param().rear_axle_to_center);
    }

    void publish(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        for (auto& value : markers_.markers) value.header.stamp = now;
        publisher_.publish(markers_);
    }

    ros::NodeHandle private_nh_;
    ros::Publisher publisher_;
    ros::Timer timer_;
    std::unique_ptr<ForkliftMap> map_;
    std::unique_ptr<CorridorBuilder> corridor_builder_;
    std::unique_ptr<CoordinateMapper> mapper_;
    std::string frame_;
    std::string route_;
    std::string catalog_file_;
    std::string error_;
    int slot_id_ = 0;
    int max_issue_labels_ = 80;
    double current_s_ = -1.0;
    double sweep_sample_step_ = 0.01;
    RoughPath rough_path_;
    std::vector<PathPoint> path_;
    CorridorResult corridor_;
    std::vector<MappedPathPoint> mapped_;
    std::vector<GeometryDiagnostic> diagnostics_;
    std::vector<SweptGeometryDiagnostic> swept_;
    visualization_msgs::MarkerArray markers_;
};

}  // namespace
}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinate_mapping_visualizer");
    forklift_nlp_recovery::CoordinateMappingVisualizer node;
    ros::spin();
    return 0;
}
