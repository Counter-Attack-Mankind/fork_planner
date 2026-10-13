#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <sandbox_msgs/RecoverySnapshot.h>
#include <sandbox_msgs/RecoveryVehicleState.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_nlp_recovery/corridor_builder.h"

namespace forklift_nlp_recovery {
namespace {

std_msgs::ColorRGBA color(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA value;
    value.r = r;
    value.g = g;
    value.b = b;
    value.a = a;
    return value;
}

geometry_msgs::Point point(double x, double y, double z) {
    geometry_msgs::Point value;
    value.x = x;
    value.y = y;
    value.z = z;
    return value;
}

void setYaw(geometry_msgs::Quaternion& orientation, double yaw) {
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
}

std::string poseText(const std::string& name, const Pose2d& pose) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << name << " (" << pose.x << ", "
        << pose.y << ") yaw=" << pose.yaw * 180.0 / M_PI << " deg";
    return out.str();
}

}  // namespace

class RecoveryVisualizerNode {
public:
    RecoveryVisualizerNode() : private_nh_("~") {
        ros::NodeHandle global_nh;
        map_param_ = MapParam::fromROSParam(global_nh);
        map_ = std::make_unique<ForkliftMap>(map_param_);
        private_nh_.param("vehicle_a_id", vehicle_a_id_, 0);
        private_nh_.param("vehicle_b_id", vehicle_b_id_, 1);
        private_nh_.param<std::string>("snapshot_topic", snapshot_topic_,
                                       "/forklift_planner/recovery_snapshot");
        private_nh_.param("vehicle_radius", config_.vehicle_radius,
                          config_.vehicle_radius);
        private_nh_.param("shelf_inflation_reduction",
                          config_.shelf_inflation_reduction,
                          config_.shelf_inflation_reduction);
        private_nh_.param("expansion_step", config_.expansion_step, config_.expansion_step);
        private_nh_.param("path_sample_step", config_.path_sample_step,
                          config_.path_sample_step);
        private_nh_.param("min_rejoin_distance", config_.min_rejoin_distance,
                          config_.min_rejoin_distance);
        private_nh_.param("rejoin_distance", config_.rejoin_distance,
                          config_.rejoin_distance);
        private_nh_.param("max_width", config_.max_width, map_param_.field_width);
        private_nh_.param("max_height", config_.max_height, config_.max_height);
        private_nh_.param("safety_margin", config_.safety_margin, config_.safety_margin);
        private_nh_.param("max_bridge_depth", config_.max_bridge_depth,
                          config_.max_bridge_depth);
        builder_ = std::make_unique<CorridorBuilder>(
            map_param_, map_->shelf_blocks(), config_);
        marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_nlp_recovery/markers", 1, true);
        snapshot_sub_ = private_nh_.subscribe(
            snapshot_topic_, 1, &RecoveryVisualizerNode::snapshotCallback, this);
        ROS_INFO("[nlp_recovery] read-only phase-1 visualizer: A=%d B=%d "
                 "disc_radius=%.4f boundary_clearance=%.4f "
                 "shelf_inflation=%.4f topic=%s",
                 vehicle_a_id_, vehicle_b_id_, builder_->vehicleRadius(),
                 builder_->boundaryClearance(), builder_->shelfInflation(),
                 snapshot_topic_.c_str());
    }

private:
    visualization_msgs::Marker baseMarker(const std::string& ns, int id,
                                          int type) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = map_param_.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        return marker;
    }

    void addRectangle(visualization_msgs::MarkerArray& markers,
                      const std::string& ns, int id, const Aabb& box,
                      double z, double width, const std_msgs::ColorRGBA& rgba) const {
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::LINE_STRIP);
        marker.scale.x = width;
        marker.color = rgba;
        marker.points = {point(box.min_x, box.min_y, z), point(box.max_x, box.min_y, z),
                         point(box.max_x, box.max_y, z), point(box.min_x, box.max_y, z),
                         point(box.min_x, box.min_y, z)};
        markers.markers.push_back(marker);
    }

    void addText(visualization_msgs::MarkerArray& markers, const std::string& ns,
                 int id, double x, double y, double z, const std::string& text,
                 const std_msgs::ColorRGBA& rgba, double height = 0.07) const {
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::TEXT_VIEW_FACING);
        marker.pose.position = point(x, y, z);
        marker.scale.z = height;
        marker.color = rgba;
        marker.text = text;
        markers.markers.push_back(marker);
    }

    void addArrow(visualization_msgs::MarkerArray& markers, const std::string& ns,
                  int id, const Pose2d& pose, const std_msgs::ColorRGBA& rgba) const {
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::ARROW);
        marker.pose.position = point(pose.x, pose.y, 0.12);
        setYaw(marker.pose.orientation, pose.yaw);
        marker.scale.x = 0.18;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        marker.color = rgba;
        markers.markers.push_back(marker);
    }

    void addSolidPoint(visualization_msgs::MarkerArray& markers,
                       const std::string& ns, int id, const Pose2d& pose,
                       const std_msgs::ColorRGBA& rgba) const {
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::SPHERE);
        marker.pose.position = point(pose.x, pose.y, 0.13);
        marker.scale.x = 0.075;
        marker.scale.y = 0.075;
        marker.scale.z = 0.075;
        marker.color = rgba;
        marker.color.a = 1.0f;
        markers.markers.push_back(marker);
    }

    void addDisc(visualization_msgs::MarkerArray& markers, const std::string& ns,
                 int id, const Pose2d& rear_axle_pose,
                 const std_msgs::ColorRGBA& rgba) const {
        const Pose2d center = builder_->bodyCenter(rear_axle_pose);
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::CYLINDER);
        marker.pose.position = point(center.x, center.y, 0.025);
        marker.scale.x = 2.0 * builder_->vehicleRadius();
        marker.scale.y = 2.0 * builder_->vehicleRadius();
        marker.scale.z = 0.05;
        marker.color = rgba;
        markers.markers.push_back(marker);
    }

    void addBody(visualization_msgs::MarkerArray& markers, const std::string& ns,
                 int id, const Pose2d& rear_axle_pose,
                 const std_msgs::ColorRGBA& rgba) const {
        const Pose2d center = builder_->bodyCenter(rear_axle_pose);
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::CUBE);
        marker.pose.position = point(center.x, center.y, 0.055);
        setYaw(marker.pose.orientation, rear_axle_pose.yaw);
        marker.scale.x = map_param_.vehicle_length;
        marker.scale.y = map_param_.vehicle_width;
        marker.scale.z = 0.07;
        marker.color = rgba;
        markers.markers.push_back(marker);
    }

    void addMap(visualization_msgs::MarkerArray& markers) const {
        int id = 0;
        for (const auto& shelf : map_->shelf_blocks()) {
            auto cube = baseMarker("shelves_original", id,
                                   visualization_msgs::Marker::CUBE);
            cube.pose.position = point(shelf.cx(), shelf.cy(), 0.04);
            cube.scale.x = shelf.w;
            cube.scale.y = shelf.h;
            cube.scale.z = 0.08;
            cube.color = color(0.35f, 0.35f, 0.42f, 0.55f);
            markers.markers.push_back(cube);
            ++id;
        }
        id = 0;
        for (const auto& obstacle : builder_->inflatedShelves()) {
            addRectangle(markers, "shelves_inflated", id++, obstacle, 0.09, 0.012,
                         color(0.95f, 0.15f, 0.15f, 0.9f));
        }
        const Aabb field{0.0, map_param_.field_width, 0.0, map_param_.field_height};
        const double c = builder_->boundaryClearance();
        const Aabb inset{c, map_param_.field_width - c,
                         c, map_param_.field_height - c};
        addRectangle(markers, "map_boundary", 0, field, 0.02, 0.02,
                     color(0.1f, 0.1f, 0.1f, 1.0f));
        addRectangle(markers, "map_boundary_inset", 0, inset, 0.03, 0.01,
                     color(0.95f, 0.45f, 0.05f, 0.9f));
    }

    std::vector<PathPoint> pathFromMessage(
        const sandbox_msgs::RecoveryVehicleState& vehicle) const {
        std::vector<Pose2d> poses;
        poses.reserve(vehicle.path.size());
        for (const auto& pose : vehicle.path) poses.push_back({pose.x, pose.y, pose.theta});
        return CorridorBuilder::parameterizePath(poses, vehicle.path_types);
    }

    void addVehicleAnalysis(visualization_msgs::MarkerArray& markers,
                            const sandbox_msgs::RecoveryVehicleState& vehicle,
                            const std::string& name, int id_offset,
                            const std_msgs::ColorRGBA& rgba) {
        const Pose2d start{vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta};
        addDisc(markers, name + "_disc", id_offset, start,
                color(rgba.r, rgba.g, rgba.b, 0.20f));
        addBody(markers, name + "_body", id_offset, start,
                color(rgba.r, rgba.g, rgba.b, 0.48f));
        addSolidPoint(markers, name + "_start_point", id_offset, start,
                      color(1.0f, 0.0f, 0.0f, 1.0f));
        addArrow(markers, name + "_start_arrow", id_offset, start,
                 color(1.0f, 0.0f, 0.0f, 1.0f));
        addText(markers, name + "_start_text", id_offset, start.x, start.y, 0.28,
                poseText("V" + std::to_string(vehicle.id) + " NLP start rear", start),
                color(1.0f, 0.0f, 0.0f, 1.0f));

        auto path_marker = baseMarker(name + "_offline_path", id_offset,
                                      visualization_msgs::Marker::LINE_STRIP);
        path_marker.scale.x = 0.018;
        path_marker.color = rgba;
        for (const auto& pose : vehicle.path) {
            path_marker.points.push_back(point(pose.x, pose.y, 0.11));
        }
        markers.markers.push_back(path_marker);

        const CorridorResult result = builder_->build(
            start, pathFromMessage(vehicle), vehicle.path_s, vehicle.path_gen);
        if (!result.success) {
            addText(markers, name + "_failure", id_offset, start.x, start.y + 0.15, 0.3,
                    name + " FAILED: " + result.reason,
                    color(1.0f, 0.05f, 0.05f, 1.0f), 0.06);
            ROS_WARN_THROTTLE(2.0, "[nlp_recovery] %s corridor failed: %s",
                              name.c_str(), result.reason.c_str());
            return;
        }
        for (size_t i = 0; i < result.boxes.size(); ++i) {
            addRectangle(markers, name + "_aabb", id_offset + static_cast<int>(i),
                         result.boxes[i], 0.15, 0.018, rgba);
            const auto& box = result.boxes[i];
            addText(markers, name + "_aabb_id", id_offset + static_cast<int>(i),
                    (box.min_x + box.max_x) * 0.5,
                    (box.min_y + box.max_y) * 0.5, 0.18,
                    name + "[" + std::to_string(i) + "]", rgba, 0.055);
        }
        addSolidPoint(markers, name + "_terminal_point", id_offset,
                      result.terminal.pose, color(0.0f, 1.0f, 0.0f, 1.0f));
        addArrow(markers, name + "_terminal_arrow", id_offset,
                 result.terminal.pose, color(0.0f, 1.0f, 0.0f, 1.0f));
        std::ostringstream terminal_text;
        terminal_text << poseText("V" + std::to_string(vehicle.id) +
                                      " NLP end rear", result.terminal.pose)
                      << " s=" << std::fixed << std::setprecision(2)
                      << result.terminal.path_s << " gen=" << result.terminal.path_gen
                      << " type=" << static_cast<int>(result.terminal.type);
        addText(markers, name + "_terminal_text", id_offset,
                result.terminal.pose.x, result.terminal.pose.y, 0.28,
                terminal_text.str(), color(0.0f, 1.0f, 0.0f, 1.0f));
        ROS_INFO_THROTTLE(2.0, "[nlp_recovery] %s corridor valid: boxes=%zu "
                          "start=(%.3f,%.3f) end=(%.3f,%.3f) s=%.3f gen=%d",
                          name.c_str(), result.boxes.size(), start.x, start.y,
                          result.terminal.pose.x, result.terminal.pose.y,
                          result.terminal.path_s, result.terminal.path_gen);
    }

    void snapshotCallback(const sandbox_msgs::RecoverySnapshot::ConstPtr& snapshot) {
        const sandbox_msgs::RecoveryVehicleState* vehicle_a = nullptr;
        const sandbox_msgs::RecoveryVehicleState* vehicle_b = nullptr;
        for (const auto& vehicle : snapshot->vehicles) {
            if (vehicle.id == vehicle_a_id_) vehicle_a = &vehicle;
            if (vehicle.id == vehicle_b_id_) vehicle_b = &vehicle;
        }
        visualization_msgs::MarkerArray markers;
        auto clear = baseMarker("clear", 0, visualization_msgs::Marker::CUBE);
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);
        addMap(markers);

        int other_id = 0;
        for (const auto& vehicle : snapshot->vehicles) {
            if (vehicle.id == vehicle_a_id_ || vehicle.id == vehicle_b_id_ ||
                !vehicle.pose_valid) continue;
            const Pose2d pose{vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta};
            addDisc(markers, "static_vehicle_discs", other_id, pose,
                    color(1.0f, 0.55f, 0.05f, 0.20f));
            addBody(markers, "static_vehicle_bodies", other_id, pose,
                    color(1.0f, 0.55f, 0.05f, 0.48f));
            addText(markers, "static_vehicle_ids", other_id++, pose.x, pose.y, 0.16,
                    "static V" + std::to_string(vehicle.id),
                    color(1.0f, 0.45f, 0.0f, 1.0f), 0.055);
        }

        if (!vehicle_a || !vehicle_b) {
            std::ostringstream out;
            out << "waiting for selected IDs A=" << vehicle_a_id_
                << " B=" << vehicle_b_id_;
            addText(markers, "status", 0, map_param_.field_width * 0.5,
                    map_param_.field_height - 0.12, 0.3, out.str(),
                    color(1.0f, 0.1f, 0.1f, 1.0f), 0.08);
            ROS_WARN_THROTTLE(2.0, "[nlp_recovery] %s", out.str().c_str());
        } else {
            if (!vehicle_a->pose_valid || vehicle_a->path.size() < 2) {
                addText(markers, "status", 1, 1.25, 4.25, 0.3,
                        "A pose/path unavailable", color(1.0f, 0.1f, 0.1f, 1.0f));
            } else {
                addVehicleAnalysis(markers, *vehicle_a, "A", 1000,
                                   color(0.05f, 0.45f, 1.0f, 0.95f));
            }
            if (!vehicle_b->pose_valid || vehicle_b->path.size() < 2) {
                addText(markers, "status", 2, 1.25, 4.12, 0.3,
                        "B pose/path unavailable", color(1.0f, 0.1f, 0.1f, 1.0f));
            } else {
                addVehicleAnalysis(markers, *vehicle_b, "B", 2000,
                                   color(0.1f, 0.85f, 0.25f, 0.95f));
            }
        }
        marker_pub_.publish(markers);
    }

    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber snapshot_sub_;
    MapParam map_param_;
    std::unique_ptr<ForkliftMap> map_;
    CorridorConfig config_;
    std::unique_ptr<CorridorBuilder> builder_;
    int vehicle_a_id_ = 0;
    int vehicle_b_id_ = 1;
    std::string snapshot_topic_;
};

}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ros::init(argc, argv, "forklift_nlp_recovery");
    forklift_nlp_recovery::RecoveryVisualizerNode node;
    ros::spin();
    return 0;
}
