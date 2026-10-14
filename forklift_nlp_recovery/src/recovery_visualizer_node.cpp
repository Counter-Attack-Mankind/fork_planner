#include <ros/ros.h>
#include <ros/package.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <set>
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
#include "forklift_nlp_recovery/initial_guess.h"
#include "forklift_nlp_recovery/nlp_solver.h"
#include "forklift_nlp_recovery/optimization_input.h"

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
        private_nh_.param<std::string>("debug_output_file", debug_output_file_,
                                       "/tmp/forklift_nlp_recovery_input.yaml");
        private_nh_.param<std::string>("results_dir", results_directory_,
                                       "package://forklift_nlp_recovery/results");
        private_nh_.param("initial_guess_auto", initial_guess_config_.automatic,
                          initial_guess_config_.automatic);
        int initial_guess_nodes = static_cast<int>(initial_guess_config_.node_count);
        private_nh_.param("initial_guess_nodes", initial_guess_nodes,
                          initial_guess_nodes);
        initial_guess_config_.node_count = initial_guess_nodes > 0 ?
            static_cast<size_t>(initial_guess_nodes) : 0;
        private_nh_.param("initial_guess_total_time",
                          initial_guess_config_.total_time,
                          initial_guess_config_.total_time);
        private_nh_.param("initial_guess_target_dt", initial_guess_config_.target_dt,
                          initial_guess_config_.target_dt);
        int initial_guess_min_nodes = static_cast<int>(initial_guess_config_.min_nodes);
        int initial_guess_max_nodes = static_cast<int>(initial_guess_config_.max_nodes);
        private_nh_.param("initial_guess_min_nodes", initial_guess_min_nodes,
                          initial_guess_min_nodes);
        private_nh_.param("initial_guess_max_nodes", initial_guess_max_nodes,
                          initial_guess_max_nodes);
        initial_guess_config_.min_nodes = initial_guess_min_nodes > 0 ?
            static_cast<size_t>(initial_guess_min_nodes) : 0;
        initial_guess_config_.max_nodes = initial_guess_max_nodes > 0 ?
            static_cast<size_t>(initial_guess_max_nodes) : 0;
        private_nh_.param("initial_guess_time_margin",
                          initial_guess_config_.time_margin,
                          initial_guess_config_.time_margin);
        private_nh_.param("solver_min_time_scale", solver_config_.min_time_scale,
                          solver_config_.min_time_scale);
        private_nh_.param("solver_max_time_scale", solver_config_.max_time_scale,
                          solver_config_.max_time_scale);
        private_nh_.param("solver_time_weight", solver_config_.time_weight,
                          solver_config_.time_weight);
        private_nh_.param("solver_acceleration_weight",
                          solver_config_.acceleration_weight,
                          solver_config_.acceleration_weight);
        private_nh_.param("solver_steer_rate_weight",
                          solver_config_.steer_rate_weight,
                          solver_config_.steer_rate_weight);
        private_nh_.param("solver_control_smooth_weight",
                          solver_config_.control_smooth_weight,
                          solver_config_.control_smooth_weight);
        private_nh_.param("solver_max_iterations", solver_config_.max_iterations,
                          solver_config_.max_iterations);
        private_nh_.param("solver_tolerance", solver_config_.tolerance,
                          solver_config_.tolerance);
        private_nh_.param("solver_acceptable_tolerance",
                          solver_config_.acceptable_tolerance,
                          solver_config_.acceptable_tolerance);
        private_nh_.param("solver_print_level", solver_config_.print_level,
                          solver_config_.print_level);
        int validation_substeps = static_cast<int>(solver_config_.validation_substeps);
        private_nh_.param("solver_validation_substeps", validation_substeps,
                          validation_substeps);
        solver_config_.validation_substeps = validation_substeps > 0 ?
            static_cast<size_t>(validation_substeps) : 0;
        private_nh_.param("solver_validation_tolerance",
                          solver_config_.validation_tolerance,
                          solver_config_.validation_tolerance);
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
        VehicleParameters vehicle_parameters;
        vehicle_parameters.length = map_param_.vehicle_length;
        vehicle_parameters.width = map_param_.vehicle_width;
        vehicle_parameters.rear_axle_to_center = map_param_.rear_axle_to_center;
        vehicle_parameters.disc_radius = builder_->vehicleRadius();
        vehicle_parameters.wheel_base = map_param_.wheel_base;
        vehicle_parameters.max_steer_angle = map_param_.max_steer_angle;
        vehicle_parameters.max_steer_rate = map_param_.max_steer_rate;
        const std::string planner_ns = "forklift_planner/multi_vehicle/";
        global_nh.param(planner_ns + "nominal_speed",
                        vehicle_parameters.nominal_speed, 0.20);
        global_nh.param(planner_ns + "max_speed",
                        vehicle_parameters.max_speed, 0.26);
        global_nh.param(planner_ns + "max_accel",
                        vehicle_parameters.max_accel, 0.20);
        global_nh.param(planner_ns + "max_decel",
                        vehicle_parameters.max_decel, 0.30);
        global_nh.param(planner_ns + "deadlock_retreat_speed",
                        vehicle_parameters.fixed_retreat_speed, 0.10);
        input_builder_ = std::make_unique<OptimizationInputBuilder>(
            *builder_, vehicle_parameters);
        initial_guess_generator_ = std::make_unique<InitialGuessGenerator>(
            initial_guess_config_);
        nlp_solver_ = std::make_unique<NlpSolver>(solver_config_);
        const std::string package_prefix = "package://forklift_nlp_recovery";
        if (results_directory_.compare(0, package_prefix.size(), package_prefix) == 0) {
            const std::string package_path = ros::package::getPath("forklift_nlp_recovery");
            if (!package_path.empty()) {
                results_directory_ = package_path +
                    results_directory_.substr(package_prefix.size());
            }
        }
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
                 "shelf_inflation=%.4f topic=%s output=%s timing=%s "
                 "manual=(N=%zu,T=%.3f) auto=(dt=%.3f,N=[%zu,%zu],margin=%.2f) "
                 "solver_time_scale=[%.2f,%.2f] validation_substeps=%zu results=%s",
                 mode_name_.c_str(), selected_ids.str().c_str(),
                 builder_->vehicleRadius(),
                 builder_->boundaryClearance(), builder_->shelfInflation(),
                 snapshot_topic_.c_str(), debug_output_file_.c_str(),
                 initial_guess_config_.automatic ? "auto" : "manual",
                 initial_guess_config_.node_count, initial_guess_config_.total_time,
                 initial_guess_config_.target_dt, initial_guess_config_.min_nodes,
                 initial_guess_config_.max_nodes, initial_guess_config_.time_margin,
                 solver_config_.min_time_scale, solver_config_.max_time_scale,
                 solver_config_.validation_substeps,
                 results_directory_.c_str());
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

    void addVehicleAnalysis(visualization_msgs::MarkerArray& markers,
                            const NlpVehicleInput& vehicle,
                            const std::string& name, int id_offset,
                            const std_msgs::ColorRGBA& rgba) {
        const Pose2d& start = vehicle.initial_state.rear_axle_pose;
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
        for (const PathPoint& path_point : vehicle.original_path) {
            path_marker.points.push_back(
                point(path_point.pose.x, path_point.pose.y, 0.11));
        }
        markers.markers.push_back(path_marker);

        if (!vehicle.corridor_valid) {
            addText(markers, name + "_failure", id_offset, start.x, start.y + 0.15, 0.3,
                    name + " FAILED: " + vehicle.corridor_reason,
                    color(1.0f, 0.05f, 0.05f, 1.0f), 0.06);
            ROS_WARN_THROTTLE(2.0, "[nlp_recovery] %s corridor failed: %s",
                              name.c_str(), vehicle.corridor_reason.c_str());
            return;
        }
        for (size_t i = 0; i < vehicle.corridor.size(); ++i) {
            addRectangle(markers, name + "_aabb", id_offset + static_cast<int>(i),
                         vehicle.corridor[i], 0.15, 0.018, rgba);
            const auto& box = vehicle.corridor[i];
            addText(markers, name + "_aabb_id", id_offset + static_cast<int>(i),
                    (box.min_x + box.max_x) * 0.5,
                    (box.min_y + box.max_y) * 0.5, 0.18,
                    name + "[" + std::to_string(i) + "]", rgba, 0.055);
        }
        addSolidPoint(markers, name + "_terminal_point", id_offset,
                      vehicle.terminal.pose, color(0.0f, 1.0f, 0.0f, 1.0f));
        addArrow(markers, name + "_terminal_arrow", id_offset,
                 vehicle.terminal.pose, color(0.0f, 1.0f, 0.0f, 1.0f));
        std::ostringstream terminal_text;
        terminal_text << poseText("V" + std::to_string(vehicle.id) +
                                      " NLP end rear", vehicle.terminal.pose)
                      << " s=" << std::fixed << std::setprecision(2)
                      << vehicle.terminal.path_s << " gen=" << vehicle.terminal.path_gen
                      << " type=" << static_cast<int>(vehicle.terminal.type);
        addText(markers, name + "_terminal_text", id_offset,
                vehicle.terminal.pose.x, vehicle.terminal.pose.y, 0.28,
                terminal_text.str(), color(0.0f, 1.0f, 0.0f, 1.0f));
        ROS_INFO_THROTTLE(2.0, "[nlp_recovery] %s corridor valid: boxes=%zu "
                          "start=(%.3f,%.3f) end=(%.3f,%.3f) s=%.3f gen=%d",
                          name.c_str(), vehicle.corridor.size(), start.x, start.y,
                          vehicle.terminal.pose.x, vehicle.terminal.pose.y,
                          vehicle.terminal.path_s, vehicle.terminal.path_gen);
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

    void clearLockedTransaction() {
        if (locked_.active) {
            ROS_INFO("[nlp_recovery][auto] clear locked NLP_PREP transaction");
        }
        locked_ = LockedTransaction{};
    }

    bool snapshotMatchesLocked(
        const sandbox_msgs::RecoverySnapshot& snapshot) const {
        const auto& input = locked_.build.input;
        if (!locked_.active || input.participant_ids != snapshot.nlp_vehicle_ids) {
            return false;
        }
        for (const NlpVehicleInput& locked_vehicle : input.vehicles) {
            const auto* vehicle = findVehicle(snapshot, locked_vehicle.id);
            if (vehicle == nullptr || vehicle->path_gen != locked_vehicle.path_gen) {
                return false;
            }
        }
        return true;
    }

    void writeDebugInput(const OptimizationInputBuildResult& build) {
        std::string error;
        if (!writeOptimizationInputDebugFile(build, debug_output_file_, error)) {
            ROS_ERROR_THROTTLE(2.0, "[nlp_recovery] input dump failed: %s",
                               error.c_str());
        }
    }

    std::string inputIdentity(const OptimizationInputBuildResult& build) const {
        std::ostringstream key;
        key << build.input.source_mode;
        for (const NlpVehicleInput& vehicle : build.input.vehicles) {
            key << ':' << vehicle.id << '@' << vehicle.path_gen;
        }
        return key.str();
    }

    void logInitialGuess(const NlpInitialGuess& guess) const {
        if (!guess.generated) {
            ROS_ERROR("[nlp_recovery][initial_guess] structural failure: %s",
                      guess.reason.c_str());
            return;
        }
        ROS_INFO("[nlp_recovery][initial_guess] vehicles=%zu timing=%s N=%zu "
                 "T_init=%.3f dt=%.3f "
                 "status=%s max_pos_res=%.6f max_yaw_res=%.6f bounds=%zu "
                 "vehicle_collisions=%zu static_collisions=%zu min_clear=(%.6f,%.6f)",
                 guess.vehicles.size(), guess.automatic_timing ? "auto" : "manual",
                 guess.node_count, guess.total_time, guess.average_dt,
                 guess.constraints_satisfied ? "valid" : "constraint_violation",
                 guess.diagnostics.max_kinematic_position_residual,
                 guess.diagnostics.max_kinematic_heading_residual,
                 guess.diagnostics.bound_violation_nodes,
                 guess.diagnostics.participant_collision_nodes,
                 guess.diagnostics.static_collision_nodes,
                 guess.diagnostics.minimum_participant_clearance,
                 guess.diagnostics.minimum_static_clearance);
        for (const VehicleInitialGuess& vehicle : guess.vehicles) {
            const auto& d = vehicle.diagnostics;
            ROS_INFO("[nlp_recovery][initial_guess] V%d length=%.3f estimate=%.3f "
                     "cusps=%zu initial_error=(%.3g m,%.3g rad,%.3g m/s) "
                     "end=(%.3g m,%.3g rad) aabb=%s max_res=(%.6f m,%.6f rad)",
                     vehicle.id, d.reference_length, d.estimated_travel_time,
                     d.direction_changes, d.start_position_error, d.start_heading_error,
                     d.start_speed_magnitude_error, d.terminal_position_error,
                     d.terminal_heading_error,
                     vehicle.aabb_assignment_valid ? "ok" : "failed",
                     d.max_kinematic_position_residual,
                     d.max_kinematic_heading_residual);
        }
        std::set<std::string> collision_pairs;
        for (const InitialCollision& collision : guess.diagnostics.collisions) {
            collision_pairs.insert("V" + std::to_string(collision.vehicle_a_id) +
                (collision.vehicle_b_is_static ? "-staticV" : "-V") +
                std::to_string(collision.vehicle_b_id));
        }
        if (!collision_pairs.empty()) {
            std::ostringstream pairs;
            for (const std::string& pair : collision_pairs) {
                if (pairs.tellp() > 0) pairs << ',';
                pairs << pair;
            }
            ROS_WARN("[nlp_recovery][initial_guess] colliding_pairs=%s",
                     pairs.str().c_str());
        }
    }

    void logSolveResult(const NlpSolveResult& result,
                        const std::string& case_id) const {
        ROS_INFO("[nlp_recovery][solver] case=%s vehicles=%zu N=%zu T=%.3f "
                 "status=%s iterations=%d wall=%.3f objective=%.6f "
                 "max_violation=%.3g validation=%s reason=%s",
                 case_id.c_str(), result.vehicles.size(), result.node_count,
                 result.total_time, result.statistics.return_status.c_str(),
                 result.statistics.iterations, result.statistics.wall_time,
                 result.statistics.objective,
                 result.statistics.max_constraint_violation,
                 result.validation_passed ? "passed" : "failed",
                 result.reason.c_str());
        if (!result.validation_passed) {
            ROS_WARN("[nlp_recovery][solver] independent validation: "
                     "boundary=(%.3g,%.3g) dynamics=(%.3g,%.3g) "
                     "bounds=(v %.3g,steer %.3g,a %.3g,omega %.3g) "
                     "aabb=%.3g/%zu pair=%zu static=%zu switches=%zu",
                     result.validation.max_start_error,
                     result.validation.max_terminal_error,
                     result.validation.max_kinematic_position_residual,
                     result.validation.max_kinematic_heading_residual,
                     result.validation.max_speed_violation,
                     result.validation.max_steer_violation,
                     result.validation.max_acceleration_violation,
                     result.validation.max_steer_rate_violation,
                     result.validation.max_aabb_violation,
                     result.validation.aabb_violation_samples,
                     result.validation.participant_collision_samples,
                     result.validation.static_collision_samples,
                     result.validation.invalid_aabb_switches);
        }
    }

    NlpSolveResult solveAndSave(const OptimizationInputBuildResult& build,
                                const NlpInitialGuess& guess,
                                const CaseSaveResult& saved) {
        NlpSolveResult result = nlp_solver_->solve(build.input, guess);
        logSolveResult(result, saved.case_id);
        std::string error;
        if (!saveNlpSolveResult(result, saved.case_directory, error)) {
            ROS_ERROR("[nlp_recovery][solver] result save failed: %s",
                      error.c_str());
        } else {
            ROS_INFO("[nlp_recovery][solver] saved optimized_trajectory.csv and "
                     "solver_report.yaml at %s", saved.case_directory.c_str());
        }
        return result;
    }

    void addOptimizedResult(visualization_msgs::MarkerArray& markers,
                            const NlpSolveResult& result,
                            const std::string& prefix) const {
        if (!result.problem_built && result.reason.empty()) return;
        const std_msgs::ColorRGBA status_color = result.validation_passed ?
            color(0.05f, 0.95f, 0.35f, 0.95f) :
            color(1.0f, 0.1f, 0.1f, 0.95f);
        for (size_t vehicle_index = 0; vehicle_index < result.vehicles.size();
             ++vehicle_index) {
            const auto& vehicle = result.vehicles[vehicle_index];
            auto path = baseMarker(prefix + "_optimized_path", vehicle.id,
                                   visualization_msgs::Marker::LINE_STRIP);
            path.scale.x = 0.028;
            path.color = status_color;
            auto centers = baseMarker(prefix + "_optimized_centers", vehicle.id,
                                      visualization_msgs::Marker::SPHERE_LIST);
            centers.scale.x = 0.025;
            centers.scale.y = 0.025;
            centers.scale.z = 0.025;
            centers.color = status_color;
            const size_t body_stride = std::max<size_t>(
                1, vehicle.nodes.size() / 10);
            for (size_t node_index = 0; node_index < vehicle.nodes.size();
                 ++node_index) {
                const auto& node = vehicle.nodes[node_index];
                path.points.push_back(point(node.rear_axle_pose.x,
                                            node.rear_axle_pose.y, 0.22));
                centers.points.push_back(point(node.disc_center.x,
                                               node.disc_center.y, 0.21));
                if (node_index % body_stride == 0 ||
                    node_index + 1 == vehicle.nodes.size()) {
                    addBody(markers, prefix + "_optimized_body_V" +
                            std::to_string(vehicle.id),
                            static_cast<int>(node_index), node.rear_axle_pose,
                            color(status_color.r, status_color.g,
                                  status_color.b, 0.16f));
                }
            }
            markers.markers.push_back(path);
            markers.markers.push_back(centers);
        }
        std::ostringstream status;
        status << "IPOPT " << result.statistics.return_status
               << " T=" << std::fixed << std::setprecision(2)
               << result.total_time << "s iter=" << result.statistics.iterations
               << " validation=" << (result.validation_passed ? "PASS" : "FAIL");
        addText(markers, prefix + "_solver_status", 0,
                map_param_.field_width * 0.5,
                map_param_.field_height - 0.22, 0.32, status.str(),
                status_color, 0.065);
    }

    NlpInitialGuess generateInitialGuess(const OptimizationInputBuildResult& build,
                                         bool save_case,
                                         CaseSaveResult* case_result = nullptr) {
        if (case_result != nullptr) *case_result = CaseSaveResult{};
        NlpInitialGuess guess = initial_guess_generator_->generate(build.input);
        if (save_case || build.input.source_mode == "auto") logInitialGuess(guess);
        if (!guess.generated || !save_case) return guess;
        const CaseSaveResult saved = saveInitialGuessCase(
            build, guess, results_directory_);
        if (saved.success) {
            if (case_result != nullptr) *case_result = saved;
            ROS_INFO("[nlp_recovery][initial_guess] saved %s at %s",
                     saved.case_id.c_str(), saved.case_directory.c_str());
        } else {
            ROS_ERROR("[nlp_recovery][initial_guess] save failed: %s",
                      saved.reason.c_str());
        }
        return guess;
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

        if (!locked_.active) {
            OptimizationInputBuildResult build = input_builder_->build(
                snapshot, snapshot.nlp_vehicle_ids, "auto");
            writeDebugInput(build);
            if (build.complete) {
                locked_.active = true;
                locked_.build = std::move(build);
                CaseSaveResult saved;
                locked_.initial_guess = generateInitialGuess(
                    locked_.build, locked_.build.input.valid, &saved);
                if (saved.success) {
                    locked_.solver_result = solveAndSave(
                        locked_.build, locked_.initial_guess, saved);
                }
                ROS_WARN("[nlp_recovery][auto] locked NLP_PREP input "
                         "participants=%zu valid=%d trigger=%s input=%s",
                         locked_.build.input.vehicles.size(),
                         locked_.build.input.valid,
                         locked_.build.input.trigger_reason.c_str(),
                         locked_.build.reason.c_str());
            }
        }

        if (!locked_.active) {
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

        for (const auto& vehicle : snapshot.vehicles) {
            const auto& ids = locked_.build.input.participant_ids;
            if (std::find(ids.begin(), ids.end(), vehicle.id) == ids.end()) {
                addOrangeVehicle(markers, vehicle);
            }
        }
        std::ostringstream ids;
        const auto& input = locked_.build.input;
        for (size_t i = 0; i < input.vehicles.size(); ++i) {
            if (i != 0) ids << ",";
            ids << "V" << input.vehicles[i].id;
            addVehicleAnalysis(markers, input.vehicles[i],
                               "nlp_V" + std::to_string(input.vehicles[i].id),
                               1000 + static_cast<int>(i) * 1000,
                               analysisColor(i));
        }
        addText(markers, "auto_status", 0, map_param_.field_width * 0.5,
                map_param_.field_height - 0.12, 0.3,
                "NLP_PREP locked: " + ids.str() + " " + input.trigger_reason +
                    " input=" + locked_.build.reason,
                color(1.0f, 0.2f, 0.2f, 1.0f), 0.06);
        addOptimizedResult(markers, locked_.solver_result, "auto");
    }

    void addDebugMarkers(visualization_msgs::MarkerArray& markers,
                         const sandbox_msgs::RecoverySnapshot& snapshot) {
        for (const auto& vehicle : snapshot.vehicles) {
            if (std::find(debug_vehicle_ids_.begin(), debug_vehicle_ids_.end(),
                          vehicle.id) == debug_vehicle_ids_.end()) {
                addOrangeVehicle(markers, vehicle);
            }
        }
        const OptimizationInputBuildResult build = input_builder_->build(
            snapshot, debug_vehicle_ids_, "debug");
        writeDebugInput(build);
        const std::string identity = inputIdentity(build);
        const bool save_case = build.complete && build.input.valid &&
                               identity != last_debug_case_identity_;
        CaseSaveResult saved;
        latest_debug_initial_guess_ = generateInitialGuess(
            build, save_case, &saved);
        if (saved.success) {
            last_debug_case_identity_ = identity;
            latest_debug_solver_result_ = solveAndSave(
                build, latest_debug_initial_guess_, saved);
        }
        for (size_t i = 0; i < build.input.vehicles.size(); ++i) {
            const NlpVehicleInput& vehicle = build.input.vehicles[i];
            addVehicleAnalysis(markers, vehicle,
                               "debug_V" + std::to_string(vehicle.id),
                               1000 + static_cast<int>(i) * 1000,
                               analysisColor(i));
        }
        if (!build.complete) {
            addText(markers, "status", 0, map_param_.field_width * 0.5,
                    map_param_.field_height - 0.12, 0.3, build.reason,
                    color(1.0f, 0.1f, 0.1f, 1.0f), 0.07);
            ROS_WARN_THROTTLE(2.0, "[nlp_recovery] %s", build.reason.c_str());
        }
        addOptimizedResult(markers, latest_debug_solver_result_, "debug");
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
        OptimizationInputBuildResult build;
        NlpInitialGuess initial_guess;
        NlpSolveResult solver_result;
    };

    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber snapshot_sub_;
    MapParam map_param_;
    std::unique_ptr<ForkliftMap> map_;
    CorridorConfig config_;
    std::unique_ptr<CorridorBuilder> builder_;
    std::unique_ptr<OptimizationInputBuilder> input_builder_;
    std::unique_ptr<InitialGuessGenerator> initial_guess_generator_;
    std::unique_ptr<NlpSolver> nlp_solver_;
    InitialGuessConfig initial_guess_config_;
    NlpSolverConfig solver_config_;
    OperatingMode mode_ = OperatingMode::DEBUG;
    std::string mode_name_ = "debug";
    LockedTransaction locked_;
    std::vector<int> debug_vehicle_ids_;
    std::string snapshot_topic_;
    std::string debug_output_file_;
    std::string results_directory_;
    std::string last_debug_case_identity_;
    NlpInitialGuess latest_debug_initial_guess_;
    NlpSolveResult latest_debug_solver_result_;
};

}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ros::init(argc, argv, "forklift_nlp_recovery");
    forklift_nlp_recovery::RecoveryVisualizerNode node;
    ros::spin();
    return 0;
}
