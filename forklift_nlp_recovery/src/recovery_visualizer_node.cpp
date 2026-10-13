#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cctype>
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

enum class OperatingMode { DEBUG, AUTO };

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

std_msgs::ColorRGBA analysisColor(size_t index) {
    static const std::vector<std_msgs::ColorRGBA> palette{
        color(0.05f, 0.45f, 1.0f, 0.95f),
        color(0.75f, 0.20f, 0.95f, 0.95f),
        color(0.10f, 0.80f, 0.30f, 0.95f),
        color(0.95f, 0.65f, 0.05f, 0.95f),
        color(0.10f, 0.80f, 0.85f, 0.95f),
        color(0.95f, 0.25f, 0.45f, 0.95f),
    };
    return palette[index % palette.size()];
}

}  // namespace

class RecoveryVisualizerNode {
public:
    RecoveryVisualizerNode() : private_nh_("~") {
        ros::NodeHandle global_nh;
        map_param_ = MapParam::fromROSParam(global_nh);
        map_ = std::make_unique<ForkliftMap>(map_param_);
        private_nh_.param<std::string>("mode", mode_name_, "debug");
        std::transform(mode_name_.begin(), mode_name_.end(), mode_name_.begin(),
                       [](unsigned char value) { return std::tolower(value); });
        if (mode_name_ == "auto") {
            mode_ = OperatingMode::AUTO;
        } else if (mode_name_ != "debug") {
            ROS_ERROR("[nlp_recovery] unsupported mode='%s'; using debug",
                      mode_name_.c_str());
            mode_name_ = "debug";
        }
        XmlRpc::XmlRpcValue configured_ids;
        if (private_nh_.getParam("vehicle_ids", configured_ids) &&
            configured_ids.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < configured_ids.size(); ++i) {
                if (configured_ids[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                    ROS_WARN("[nlp_recovery] ignoring non-integer vehicle_ids[%d]", i);
                    continue;
                }
                const int id = static_cast<int>(configured_ids[i]);
                if (std::find(debug_vehicle_ids_.begin(), debug_vehicle_ids_.end(), id) ==
                    debug_vehicle_ids_.end()) {
                    debug_vehicle_ids_.push_back(id);
                }
            }
        }
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
        private_nh_.param("boundary_inset_reduction",config_.boundary_inset_reduction,
                          config_.boundary_inset_reduction);
        builder_ = std::make_unique<CorridorBuilder>(
            map_param_, map_->shelf_blocks(), config_);
        marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_nlp_recovery/markers", 1, true);
        snapshot_sub_ = private_nh_.subscribe(
            snapshot_topic_, 1, &RecoveryVisualizerNode::snapshotCallback, this);
        std::ostringstream selected_ids;
        for (size_t i = 0; i < debug_vehicle_ids_.size(); ++i) {
            if (i != 0) selected_ids << ",";
            selected_ids << debug_vehicle_ids_[i];
        }
        ROS_INFO("[nlp_recovery] read-only phase-1 visualizer: mode=%s debug_ids=[%s] "
                 "disc_radius=%.4f boundary_clearance=%.4f "
                 "shelf_inflation=%.4f topic=%s",
                 mode_name_.c_str(), selected_ids.str().c_str(),
                 builder_->vehicleRadius(),
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

    void addRoundedRectangle(
        visualization_msgs::MarkerArray& markers,
        const std::string& ns,
        int id,
        const Aabb& box,
        double radius,
        double z,
        double width,
        const std_msgs::ColorRGBA& rgba) const
    {
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::LINE_STRIP);
        marker.scale.x = width;
        marker.color = rgba;
        constexpr int kArcSegments = 12;
        constexpr double kPi = 3.14159265358979323846;
        auto addArc = [&](double cx, double cy, double start) {
            for (int i = 0; i <= kArcSegments; ++i) {
                const double angle = start + (kPi * 0.5) * i / kArcSegments;
                marker.points.push_back(point(cx + radius * std::cos(angle), cy + radius * std::sin(angle), z));
            }
        };

        // 从右下角开始，逆时针依次绘制
        addArc(box.max_x, box.min_y, -kPi * 0.5);
        addArc(box.max_x, box.max_y, 0.0);
        addArc(box.min_x, box.max_y, kPi * 0.5);
        addArc(box.min_x, box.min_y, kPi);

        // 闭合轮廓
        marker.points.push_back(marker.points.front());
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

    void addDiscCenter(visualization_msgs::MarkerArray& markers,
                       const std::string& ns, int id,
                       const Pose2d& rear_axle_pose,
                       const std_msgs::ColorRGBA& rgba) const {
        const Pose2d center = builder_->bodyCenter(rear_axle_pose);
        auto marker = baseMarker(ns, id, visualization_msgs::Marker::SPHERE);
        marker.pose.position = point(center.x, center.y, 0.09);
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
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

        for (const auto& shelf : builder_->originalShelves()) 
            addRoundedRectangle(markers,"shelves_inflated",id++,shelf,builder_->shelfInflation(),0.09,0.012,color(0.95f, 0.15f, 0.15f, 0.9f));


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

    CorridorResult analyze(
        const sandbox_msgs::RecoveryVehicleState& vehicle) const {
        const Pose2d start{vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta};
        return builder_->build(
            start, pathFromMessage(vehicle), vehicle.path_s, vehicle.path_gen);
    }

    void addVehicleAnalysis(visualization_msgs::MarkerArray& markers,
                            const sandbox_msgs::RecoveryVehicleState& vehicle,
                            const CorridorResult& result,
                            const std::string& name, int id_offset,
                            const std_msgs::ColorRGBA& rgba) {
        const Pose2d start{vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta};
        addDisc(markers, name + "_disc", id_offset, start,
                color(rgba.r, rgba.g, rgba.b, 0.20f));
        addDiscCenter(markers, name + "_disc_center", id_offset, start,
                      color(rgba.r, rgba.g, rgba.b, 1.0f));
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

    void addOrangeVehicle(visualization_msgs::MarkerArray& markers,
                          const sandbox_msgs::RecoveryVehicleState& vehicle) const {
        if (!vehicle.pose_valid) return;
        const Pose2d pose{vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta};
        addDisc(markers, "normal_vehicle_discs", vehicle.id, pose,
                color(1.0f, 0.55f, 0.05f, 0.20f));
        addDiscCenter(markers, "normal_vehicle_centers", vehicle.id, pose,
                      color(1.0f, 0.45f, 0.0f, 1.0f));
        addBody(markers, "normal_vehicle_bodies", vehicle.id, pose,
                color(1.0f, 0.55f, 0.05f, 0.48f));
        addText(markers, "normal_vehicle_ids", vehicle.id, pose.x, pose.y, 0.16,
                "V" + std::to_string(vehicle.id),
                color(1.0f, 0.45f, 0.0f, 1.0f), 0.055);
    }

    const sandbox_msgs::RecoveryVehicleState* findVehicle(
        const sandbox_msgs::RecoverySnapshot& snapshot, int id) const {
        const auto found = std::find_if(
            snapshot.vehicles.begin(), snapshot.vehicles.end(),
            [id](const auto& vehicle) { return vehicle.id == id; });
        return found == snapshot.vehicles.end() ? nullptr : &*found;
    }

    bool validForAnalysis(
        const sandbox_msgs::RecoveryVehicleState* vehicle) const {
        return vehicle && vehicle->pose_valid && vehicle->path.size() >= 2;
    }

    void clearLockedTransaction() {
        if (locked_.active) {
            ROS_INFO("[nlp_recovery][auto] clear locked NLP_PREP transaction");
        }
        locked_ = LockedTransaction{};
    }

    bool snapshotMatchesLocked(
        const sandbox_msgs::RecoverySnapshot& snapshot) const {
        if (!locked_.active || locked_.vehicle_ids != snapshot.nlp_vehicle_ids) {
            return false;
        }
        for (size_t i = 0; i < locked_.vehicle_ids.size(); ++i) {
            const auto* vehicle = findVehicle(snapshot, locked_.vehicle_ids[i]);
            if (vehicle == nullptr || vehicle->path_gen != locked_.path_gens[i]) {
                return false;
            }
        }
        return true;
    }

    void addAutoMarkers(visualization_msgs::MarkerArray& markers,
                        const sandbox_msgs::RecoverySnapshot& snapshot) {
        if (snapshot.recovery_phase !=
            sandbox_msgs::RecoverySnapshot::PHASE_NLP_PREP) {
            clearLockedTransaction();
            for (const auto& vehicle : snapshot.vehicles) {
                addOrangeVehicle(markers, vehicle);
            }
            return;
        }

        if (locked_.active && !snapshotMatchesLocked(snapshot)) {
            clearLockedTransaction();
        }

        std::vector<const sandbox_msgs::RecoveryVehicleState*> participants;
        participants.reserve(snapshot.nlp_vehicle_ids.size());
        bool valid = !snapshot.nlp_vehicle_ids.empty();
        for (int id : snapshot.nlp_vehicle_ids) {
            const auto* vehicle = findVehicle(snapshot, id);
            participants.push_back(vehicle);
            valid = valid && validForAnalysis(vehicle);
        }
        if (!locked_.active && !valid) {
            for (const auto& vehicle : snapshot.vehicles) {
                addOrangeVehicle(markers, vehicle);
            }
            addText(markers, "auto_status", 0, map_param_.field_width * 0.5,
                    map_param_.field_height - 0.12, 0.3,
                    "NLP_PREP waiting for valid participant snapshots",
                    color(1.0f, 0.1f, 0.1f, 1.0f), 0.065);
            ROS_WARN_THROTTLE(2.0, "[nlp_recovery][auto] NLP_PREP participant "
                                   "list is empty or has invalid pose/path");
            return;
        }

        if (!locked_.active) {
            locked_.active = true;
            locked_.vehicle_ids.assign(snapshot.nlp_vehicle_ids.begin(),
                                       snapshot.nlp_vehicle_ids.end());
            locked_.reason = snapshot.reason;
            for (const auto* vehicle : participants) {
                locked_.path_gens.push_back(vehicle->path_gen);
                locked_.vehicles.push_back(*vehicle);
                locked_.results.push_back(analyze(*vehicle));
            }
            ROS_WARN("[nlp_recovery][auto] locked NLP_PREP transaction "
                     "participants=%zu reason=%s", locked_.vehicles.size(),
                     locked_.reason.c_str());
        }

        for (const auto& vehicle : snapshot.vehicles) {
            if (std::find(locked_.vehicle_ids.begin(), locked_.vehicle_ids.end(),
                          vehicle.id) == locked_.vehicle_ids.end()) {
                addOrangeVehicle(markers, vehicle);
            }
        }
        std::ostringstream ids;
        for (size_t i = 0; i < locked_.vehicles.size(); ++i) {
            if (i != 0) ids << ",";
            ids << "V" << locked_.vehicles[i].id;
            addVehicleAnalysis(markers, locked_.vehicles[i], locked_.results[i],
                               "nlp_V" + std::to_string(locked_.vehicles[i].id),
                               1000 + static_cast<int>(i) * 1000,
                               analysisColor(i));
        }
        addText(markers, "auto_status", 0, map_param_.field_width * 0.5,
                map_param_.field_height - 0.12, 0.3,
                "NLP_PREP locked: " + ids.str() + " " + locked_.reason,
                color(1.0f, 0.2f, 0.2f, 1.0f), 0.06);
    }

    void addDebugMarkers(visualization_msgs::MarkerArray& markers,
                         const sandbox_msgs::RecoverySnapshot& snapshot) {
        for (const auto& vehicle : snapshot.vehicles) {
            if (std::find(debug_vehicle_ids_.begin(), debug_vehicle_ids_.end(),
                          vehicle.id) == debug_vehicle_ids_.end()) {
                addOrangeVehicle(markers, vehicle);
            }
        }
        std::vector<int> unavailable;
        for (size_t i = 0; i < debug_vehicle_ids_.size(); ++i) {
            const int id = debug_vehicle_ids_[i];
            const auto* vehicle = findVehicle(snapshot, id);
            if (!validForAnalysis(vehicle)) {
                unavailable.push_back(id);
                continue;
            }
            addVehicleAnalysis(markers, *vehicle, analyze(*vehicle),
                               "debug_V" + std::to_string(id),
                               1000 + static_cast<int>(i) * 1000,
                               analysisColor(i));
        }
        if (debug_vehicle_ids_.empty() || !unavailable.empty()) {
            std::ostringstream status;
            status << (debug_vehicle_ids_.empty() ? "vehicle_ids is empty"
                                                  : "waiting for V");
            for (size_t i = 0; i < unavailable.size(); ++i) {
                if (i != 0) status << ",V";
                status << unavailable[i];
            }
            addText(markers, "status", 0, map_param_.field_width * 0.5,
                    map_param_.field_height - 0.12, 0.3, status.str(),
                    color(1.0f, 0.1f, 0.1f, 1.0f), 0.07);
            ROS_WARN_THROTTLE(2.0, "[nlp_recovery] %s", status.str().c_str());
        }
    }

    void snapshotCallback(const sandbox_msgs::RecoverySnapshot::ConstPtr& snapshot) {
        visualization_msgs::MarkerArray markers;
        auto clear = baseMarker("clear", 0, visualization_msgs::Marker::CUBE);
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);
        addMap(markers);

        if (mode_ == OperatingMode::AUTO) addAutoMarkers(markers, *snapshot);
        else addDebugMarkers(markers, *snapshot);
        marker_pub_.publish(markers);
    }

    struct LockedTransaction {
        bool active = false;
        std::vector<int> vehicle_ids;
        std::vector<int> path_gens;
        std::string reason;
        std::vector<sandbox_msgs::RecoveryVehicleState> vehicles;
        std::vector<CorridorResult> results;
    };

    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber snapshot_sub_;
    MapParam map_param_;
    std::unique_ptr<ForkliftMap> map_;
    CorridorConfig config_;
    std::unique_ptr<CorridorBuilder> builder_;
    OperatingMode mode_ = OperatingMode::DEBUG;
    std::string mode_name_ = "debug";
    LockedTransaction locked_;
    std::vector<int> debug_vehicle_ids_;
    std::string snapshot_topic_;
};

}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ros::init(argc, argv, "forklift_nlp_recovery");
    forklift_nlp_recovery::RecoveryVisualizerNode node;
    ros::spin();
    return 0;
}
