#include "forklift_nlp_recovery/replay_data.h"

#include <forklift_map/forklift_map.h>
#include <forklift_map/map_param.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>

namespace forklift_nlp_recovery {
namespace {

std_msgs::ColorRGBA color(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA value;
    value.r = r; value.g = g; value.b = b; value.a = a;
    return value;
}

std_msgs::ColorRGBA vehicleColor(std::size_t index) {
    static const std_msgs::ColorRGBA colors[] = {
        color(0.10f, 0.65f, 1.00f), color(0.20f, 0.90f, 0.35f),
        color(0.95f, 0.65f, 0.10f), color(0.75f, 0.35f, 0.95f),
        color(0.10f, 0.85f, 0.80f), color(0.95f, 0.35f, 0.65f)};
    return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

geometry_msgs::Point point(double x, double y, double z) {
    geometry_msgs::Point value;
    value.x = x; value.y = y; value.z = z;
    return value;
}

}  // namespace

class NlpTrajectoryReplayNode {
public:
    NlpTrajectoryReplayNode() : private_nh_("~") {
        private_nh_.param("case_dir", case_dir_, std::string());
        private_nh_.param("playback_rate", playback_rate_, 1.0);
        private_nh_.param("refresh_period", refresh_period_, 0.02);
        private_nh_.param("paused", paused_, false);
        int validation_substeps = 10;
        private_nh_.param("solver_validation_substeps", validation_substeps, validation_substeps);
        private_nh_.param("solver_validation_tolerance", validation_tolerance_, 1e-5);
        validation_substeps_ = validation_substeps > 0 ? static_cast<std::size_t>(validation_substeps) : 0;
        if (case_dir_.empty()) throw std::runtime_error("~case_dir is required");
        if (playback_rate_ < 0.0 || refresh_period_ <= 0.0 || validation_substeps_ == 0 || validation_tolerance_ < 0.0) {
            throw std::runtime_error("invalid replay timing or validation parameter");
        }
        std::string error;
        if (!loadReplayCase(case_dir_, replay_case_, error)) throw std::runtime_error(error);
        diagnostics_ = scanReplayCase(replay_case_, validation_substeps_, validation_tolerance_);
        map_param_ = MapParam::fromROSParam(nh_);
        map_ = std::make_unique<ForkliftMap>(map_param_);

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_nlp_recovery/replay_markers", 1, true);
        pause_service_ = private_nh_.advertiseService("set_paused", &NlpTrajectoryReplayNode::setPaused, this);
        restart_service_ = private_nh_.advertiseService("restart", &NlpTrajectoryReplayNode::restart, this);
        last_wall_time_ = ros::WallTime::now();
        timer_ = nh_.createWallTimer(ros::WallDuration(refresh_period_), &NlpTrajectoryReplayNode::tick, this);
        logSummary();
        publish();
    }

private:
    visualization_msgs::Marker marker(const std::string& ns, int type, const ros::Time& stamp) {
        visualization_msgs::Marker value;
        value.header.frame_id = replay_case_.frame_id;
        value.header.stamp = stamp;
        value.ns = "nlp_replay/" + ns;
        value.id = next_marker_id_++;
        value.type = type;
        value.action = visualization_msgs::Marker::ADD;
        value.pose.orientation.w = 1.0;
        return value;
    }

    void addText(visualization_msgs::MarkerArray& array, const std::string& ns,
                 double x, double y, double z, const std::string& text_value,
                 const std_msgs::ColorRGBA& marker_color, double size, const ros::Time& stamp) {
        auto value = marker(ns, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
        value.pose.position = point(x, y, z);
        value.scale.z = size;
        value.color = marker_color;
        value.text = text_value;
        array.markers.push_back(value);
    }

    void addMap(visualization_msgs::MarkerArray& array, const ros::Time& stamp) {
        for (const ShelfBlock& shelf : map_->shelf_blocks()) {
            auto value = marker("map_shelves", visualization_msgs::Marker::CUBE, stamp);
            value.pose.position = point(shelf.cx(), shelf.cy(), 0.04);
            value.scale.x = shelf.w; value.scale.y = shelf.h; value.scale.z = 0.08;
            value.color = color(0.35f, 0.27f, 0.45f, 0.75f);
            array.markers.push_back(value);
        }
        auto boundary = marker("map_boundary", visualization_msgs::Marker::LINE_STRIP, stamp);
        boundary.scale.x = 0.018;
        boundary.color = color(0.75f, 0.75f, 0.80f);
        boundary.points = {point(0, 0, 0.01), point(map_param_.field_width, 0, 0.01),
                           point(map_param_.field_width, map_param_.field_height, 0.01),
                           point(0, map_param_.field_height, 0.01), point(0, 0, 0.01)};
        array.markers.push_back(boundary);
    }

    void addFixedGeometry(visualization_msgs::MarkerArray& array,
                          const std::vector<ReplayViolation>& violations,
                          const ros::Time& stamp) {
        for (std::size_t vehicle_index = 0; vehicle_index < replay_case_.vehicles.size(); ++vehicle_index) {
            const ReplayVehicle& vehicle = replay_case_.vehicles[vehicle_index];
            const auto base_color = vehicleColor(vehicle_index);
            for (std::size_t box_index = 0; box_index < vehicle.corridor.size(); ++box_index) {
                const ReplayAabb& box = vehicle.corridor[box_index];
                auto outline = marker("aabb", visualization_msgs::Marker::LINE_STRIP, stamp);
                outline.scale.x = 0.008;
                outline.color = color(base_color.r, base_color.g, base_color.b, 0.35f);
                outline.points = {point(box.min_x, box.min_y, 0.015), point(box.max_x, box.min_y, 0.015),
                                  point(box.max_x, box.max_y, 0.015), point(box.min_x, box.max_y, 0.015),
                                  point(box.min_x, box.min_y, 0.015)};
                array.markers.push_back(outline);
            }
            auto trajectory = marker("optimized_rear_trajectory", visualization_msgs::Marker::LINE_STRIP, stamp);
            trajectory.scale.x = 0.018;
            trajectory.color = color(base_color.r, base_color.g, base_color.b, 0.75f);
            for (const ReplayNode& node : vehicle.nodes) trajectory.points.push_back(point(node.rear_axle.x, node.rear_axle.y, 0.035));
            array.markers.push_back(trajectory);
        }
        std::set<int> colliding_static_ids;
        for (const ReplayViolation& violation : violations) {
            if (violation.type == ReplayViolationType::OBB &&
                violation.other_is_static) {
                colliding_static_ids.insert(violation.other_id);
            }
        }
        for (const ReplayStaticVehicle& obstacle : replay_case_.static_vehicles) {
            const ReplayPose center = replayBodyCenter(
                obstacle.rear_axle, obstacle.rear_axle_to_center);
            const bool colliding = colliding_static_ids.count(obstacle.id) != 0;
            const auto static_color = colliding ? color(1.0f, 0.0f, 0.0f) :
                                                  color(1.0f, 0.55f, 0.10f);
            auto body = marker("static_vehicle_body", visualization_msgs::Marker::CUBE, stamp);
            body.pose.position = point(center.x, center.y, 0.075);
            body.pose.orientation.z = std::sin(obstacle.rear_axle.yaw * 0.5);
            body.pose.orientation.w = std::cos(obstacle.rear_axle.yaw * 0.5);
            body.scale.x = obstacle.length;
            body.scale.y = obstacle.width;
            body.scale.z = 0.10;
            body.color = color(static_color.r, static_color.g, static_color.b, 0.85f);
            array.markers.push_back(body);
            addText(array, "static_vehicle_label", center.x, center.y, 0.18,
                    "static V" + std::to_string(obstacle.id), static_color, 0.07, stamp);
        }
        addHistoricalViolations(array, stamp);
    }

    void addHistoricalViolations(visualization_msgs::MarkerArray& array, const ros::Time& stamp) {
        for (ReplayViolationType type : {ReplayViolationType::AABB,
                                         ReplayViolationType::OBB}) {
            auto points = marker(std::string("history_") + replayViolationTypeName(type),
                                 visualization_msgs::Marker::SPHERE_LIST, stamp);
            points.scale.x = points.scale.y = points.scale.z = 0.035;
            points.color = type == ReplayViolationType::OBB ?
                color(1.0f, 0.1f, 0.1f, 0.75f) :
                color(1.0f, 0.45f, 0.0f, 0.75f);
            for (const ReplayViolation& violation : diagnostics_.violations) {
                if (violation.type != type) continue;
                points.points.push_back(point(violation.first_center.x, violation.first_center.y, 0.055));
                if (type != ReplayViolationType::AABB) points.points.push_back(point(violation.second_center.x, violation.second_center.y, 0.055));
            }
            if (!points.points.empty()) array.markers.push_back(points);
        }
    }

    void addCurrentVehicles(visualization_msgs::MarkerArray& array,
                            const std::vector<ReplayViolation>& violations, const ros::Time& stamp) {
        std::set<int> violated;
        for (const ReplayViolation& violation : violations) {
            if (violation.type != ReplayViolationType::OBB) continue;
            violated.insert(violation.vehicle_id);
            if (!violation.other_is_static) violated.insert(violation.other_id);
        }
        for (std::size_t i = 0; i < replay_case_.vehicles.size(); ++i) {
            const ReplayVehicle& vehicle = replay_case_.vehicles[i];
            const ReplayNode state = sampleReplayVehicle(vehicle, playback_time_);
            const ReplayPose center = replayBodyCenter(state.rear_axle, vehicle.parameters.rear_axle_to_center);
            const auto base = violated.count(vehicle.id) ? color(1.0f, 0.0f, 0.0f) : vehicleColor(i);
            auto body = marker("current_body", visualization_msgs::Marker::CUBE, stamp);
            body.pose.position = point(center.x, center.y, 0.075);
            body.pose.orientation.z = std::sin(state.rear_axle.yaw * 0.5);
            body.pose.orientation.w = std::cos(state.rear_axle.yaw * 0.5);
            body.scale.x = vehicle.parameters.length; body.scale.y = vehicle.parameters.width; body.scale.z = 0.10;
            body.color = color(base.r, base.g, base.b, 0.9f);
            array.markers.push_back(body);
            std::ostringstream label;
            label << "V" << vehicle.id << "  t=" << std::fixed << std::setprecision(2) << playback_time_;
            addText(array, "current_vehicle_label", center.x, center.y, 0.18, label.str(), base, 0.075, stamp);
        }
    }

    void addCurrentViolations(visualization_msgs::MarkerArray& array,
                              const std::vector<ReplayViolation>& violations, const ros::Time& stamp) {
        for (const ReplayViolation& violation : violations) {
            const bool obb = violation.type == ReplayViolationType::OBB;
            const auto marker_color = obb ? color(1.0f, 0.0f, 0.0f) :
                                            color(1.0f, 0.45f, 0.0f);
            if (obb) {
                auto line = marker("current_obb_sat", visualization_msgs::Marker::LINE_LIST,
                                   stamp);
                line.scale.x = 0.025;
                line.color = marker_color;
                line.points = {
                    point(violation.first_center.x, violation.first_center.y, 0.12),
                    point(violation.second_center.x, violation.second_center.y, 0.12)};
                array.markers.push_back(line);
            } else {
                auto location = marker("current_aabb_violation",
                    visualization_msgs::Marker::SPHERE, stamp);
                location.pose.position = point(violation.first_center.x,
                                               violation.first_center.y, 0.08);
                location.scale.x = location.scale.y = location.scale.z = 0.05;
                location.color = marker_color;
                array.markers.push_back(location);
            }
            std::ostringstream text;
            text << replayViolationTypeName(violation.type) << " V" << violation.vehicle_id;
            if (violation.other_id >= 0) {
                text << "-" << (violation.other_is_static ? "static V" : "V")
                     << violation.other_id;
            }
            text << " t=" << std::fixed << std::setprecision(2) << violation.time;
            if (violation.type == ReplayViolationType::AABB) text << " outside=" << std::setprecision(4) << violation.magnitude;
            else text << " overlap=" << std::setprecision(4) << violation.magnitude;
            addText(array, "current_violation_text",
                    (violation.first_center.x + violation.second_center.x) * 0.5,
                    (violation.first_center.y + violation.second_center.y) * 0.5, 0.24,
                    text.str(), marker_color, 0.065, stamp);
        }
    }

    void publish() {
        visualization_msgs::MarkerArray array;
        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        array.markers.push_back(clear);
        next_marker_id_ = 0;
        const ros::Time stamp = ros::Time::now();
        const auto violations = checkReplayInstant(
            replay_case_, playback_time_, validation_tolerance_);
        addMap(array, stamp);
        addFixedGeometry(array, violations, stamp);
        addCurrentVehicles(array, violations, stamp);
        addCurrentViolations(array, violations, stamp);
        std::ostringstream status;
        status << replay_case_.case_id << "  t=" << std::fixed << std::setprecision(2)
               << playback_time_ << "/" << replay_case_.total_time << "  x" << playback_rate_
               << (paused_ ? "  PAUSED" : "") << "\nIPOPT: " << replay_case_.report.return_status
               << "  saved validation: " << (replay_case_.report.validation_passed ? "PASS" : "FAIL");
        addText(array, "status", map_param_.field_width * 0.5, map_param_.field_height + 0.12,
                0.1, status.str(), color(1.0f, 1.0f, 1.0f), 0.09, stamp);
        marker_pub_.publish(array);
    }

    void tick(const ros::WallTimerEvent& event) {
        const ros::WallTime now = event.current_real;
        if (!paused_) playback_time_ = wrapReplayTime(playback_time_, (now - last_wall_time_).toSec(), playback_rate_, replay_case_.total_time);
        last_wall_time_ = now;
        publish();
    }

    bool setPaused(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
        paused_ = request.data;
        last_wall_time_ = ros::WallTime::now();
        response.success = true;
        response.message = paused_ ? "replay paused" : "replay resumed";
        publish();
        return true;
    }

    bool restart(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
        playback_time_ = 0.0;
        last_wall_time_ = ros::WallTime::now();
        response.success = true;
        response.message = "replay restarted at t=0";
        publish();
        return true;
    }

    void logSummary() const {
        std::ostringstream ids;
        for (std::size_t i = 0; i < replay_case_.participant_ids.size(); ++i) {
            if (i) ids << ',';
            ids << replay_case_.participant_ids[i];
        }
        ROS_INFO("NLP replay case=%s vehicles=[%s] T=%.6f solver=%s "
                 "saved_validation=%s saved_collision_geometry=%s "
                 "replay_min_pair_obb_sat_margin=%.6f "
                 "replay_min_static_obb_sat_margin=%.6f intervals=%zu",
                 replay_case_.case_id.c_str(), ids.str().c_str(), replay_case_.total_time,
                 replay_case_.report.return_status.c_str(), replay_case_.report.validation_passed ? "PASS" : "FAIL",
                 replay_case_.report.collision_geometry.c_str(),
                 diagnostics_.minimum_participant_obb_separation_margin,
                 diagnostics_.minimum_static_obb_separation_margin,
                 diagnostics_.intervals.size());
        for (const ReplayViolationInterval& interval : diagnostics_.intervals) {
            ROS_WARN("replay violation type=%s vehicles=%d,%d time=[%.6f, %.6f] max=%.6f",
                     replayViolationTypeName(interval.type), interval.vehicle_id, interval.other_id,
                     interval.start_time, interval.end_time, interval.max_violation);
        }
        ROS_INFO("Replay collision diagnostics use saved vehicle OBB geometry and SAT only; AABB diagnostics remain separate. Shelf geometry is display-only because the saved input contains no complete shelf geometry.");
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::ServiceServer pause_service_;
    ros::ServiceServer restart_service_;
    ros::WallTimer timer_;
    ros::WallTime last_wall_time_;
    std::string case_dir_;
    ReplayCase replay_case_;
    ReplayDiagnostics diagnostics_;
    MapParam map_param_;
    std::unique_ptr<ForkliftMap> map_;
    double playback_rate_ = 1.0;
    double refresh_period_ = 0.02;
    double validation_tolerance_ = 1e-5;
    double playback_time_ = 0.0;
    std::size_t validation_substeps_ = 10;
    int next_marker_id_ = 0;
    bool paused_ = false;
};

}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ros::init(argc, argv, "nlp_trajectory_replay");
    try {
        forklift_nlp_recovery::NlpTrajectoryReplayNode node;
        ros::spin();
        return 0;
    } catch (const std::exception& exception) {
        ROS_FATAL("Cannot start NLP trajectory replay: %s", exception.what());
        return 1;
    }
}
