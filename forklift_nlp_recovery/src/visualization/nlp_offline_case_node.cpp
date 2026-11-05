#include "forklift_nlp_recovery/case_manager.h"
#include "forklift_nlp_recovery/replay_data.h"

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sandbox_msgs/RecoverySnapshot.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace forklift_nlp_recovery {
namespace {

geometry_msgs::Point point(double x, double y, double z) {
    geometry_msgs::Point value;
    value.x = x; value.y = y; value.z = z;
    return value;
}

std_msgs::ColorRGBA color(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA value;
    value.r = r; value.g = g; value.b = b; value.a = a;
    return value;
}

unsigned int largestNumericDirectory(const std::filesystem::path& root) {
    unsigned int largest = 0;
    std::error_code error;
    if (!std::filesystem::is_directory(root, error)) return 0;
    for (const auto& entry : std::filesystem::directory_iterator(root, error)) {
        if (!entry.is_directory()) continue;
        try {
            const std::string name = entry.path().filename().string();
            if (!name.empty() && std::all_of(name.begin(), name.end(), ::isdigit)) {
                largest = std::max(largest, static_cast<unsigned int>(std::stoul(name)));
            }
        } catch (const std::exception&) {}
    }
    return largest;
}

}  // namespace

class NlpOfflineCaseNode {
public:
    NlpOfflineCaseNode() : private_nh_("~") {
        private_nh_.param<std::string>("case_dir", case_dir_, "");
        private_nh_.param<std::string>("result_case_dir", result_case_dir_, "");
        private_nh_.param("playback_rate", playback_rate_, 1.0);
        if (case_dir_.empty() || result_case_dir_.empty()) {
            throw std::runtime_error("~case_dir and ~result_case_dir are required");
        }
        std::string error;
        if (!loadRecoveryCase(case_dir_, snapshot_, participant_ids_, error)) {
            throw std::runtime_error("cannot load offline case: " + error);
        }
        baseline_run_ = largestNumericDirectory(result_case_dir_);
        snapshot_.header.stamp = ros::Time::now();
        snapshot_pub_ = nh_.advertise<sandbox_msgs::RecoverySnapshot>(
            "/forklift_nlp_recovery/offline_snapshot", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_nlp_recovery/offline_playback_markers", 1, true);
        timer_ = nh_.createWallTimer(ros::WallDuration(0.05),
                                     &NlpOfflineCaseNode::tick, this);
        last_wall_time_ = ros::WallTime::now();
        publishSnapshot();
        ROS_INFO("[NLP_OFFLINE] loaded %s participants=%zu; waiting for run > %u",
                 case_dir_.c_str(), participant_ids_.size(), baseline_run_);
    }

private:
    void publishSnapshot() {
        snapshot_.header.stamp = ros::Time::now();
        snapshot_pub_.publish(snapshot_);
    }

    visualization_msgs::Marker marker(const std::string& ns, int type) {
        visualization_msgs::Marker value;
        value.header.frame_id = replay_.frame_id;
        value.header.stamp = ros::Time::now();
        value.ns = "nlp_offline/" + ns;
        value.id = next_marker_id_++;
        value.type = type;
        value.action = visualization_msgs::Marker::ADD;
        value.pose.orientation.w = 1.0;
        return value;
    }

    void publishReplay() {
        visualization_msgs::MarkerArray array;
        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        array.markers.push_back(clear);
        next_marker_id_ = 0;
        const auto violations = checkReplayInstant(replay_, playback_time_, 1e-5);
        std::set<int> colliding;
        for (const auto& violation : violations) {
            if (violation.type == ReplayViolationType::OBB) {
                colliding.insert(violation.vehicle_id);
                if (!violation.other_is_static) colliding.insert(violation.other_id);
            }
        }
        for (size_t i = 0; i < replay_.vehicles.size(); ++i) {
            const ReplayVehicle& vehicle = replay_.vehicles[i];
            const ReplayNode state = sampleReplayVehicle(vehicle, playback_time_);
            const ReplayPose center = replayBodyCenter(
                state.rear_axle, vehicle.parameters.rear_axle_to_center);
            const auto body_color = colliding.count(vehicle.id) ?
                color(1.0f, 0.0f, 0.0f, 0.95f) :
                color(i == 0 ? 0.1f : 0.2f, i == 1 ? 0.9f : 0.55f,
                      i == 2 ? 0.95f : 0.35f, 0.9f);
            auto body = marker("body", visualization_msgs::Marker::CUBE);
            body.pose.position = point(center.x, center.y, 0.08);
            body.pose.orientation.z = std::sin(state.rear_axle.yaw * 0.5);
            body.pose.orientation.w = std::cos(state.rear_axle.yaw * 0.5);
            body.scale.x = vehicle.parameters.length;
            body.scale.y = vehicle.parameters.width;
            body.scale.z = 0.12;
            body.color = body_color;
            array.markers.push_back(body);
            auto label = marker("label", visualization_msgs::Marker::TEXT_VIEW_FACING);
            label.pose.position = point(center.x, center.y, 0.22);
            label.scale.z = 0.08;
            label.color = body_color;
            label.text = "V" + std::to_string(vehicle.id);
            array.markers.push_back(label);
        }
        marker_pub_.publish(array);
    }

    void detectResult() {
        const unsigned int latest = largestNumericDirectory(result_case_dir_);
        if (latest <= baseline_run_) return;
        const std::filesystem::path directory =
            std::filesystem::path(result_case_dir_) / std::to_string(latest);
        const std::filesystem::path status_path = directory / "run_status.yaml";
        if (!std::filesystem::exists(status_path)) return;
        try {
            const YAML::Node status = YAML::LoadFile(status_path.string());
            if (!status["complete"] || !status["complete"].as<bool>()) return;
            const bool success = status["success"] && status["success"].as<bool>();
            if (!success) {
                ROS_ERROR("[NLP_OFFLINE] run %u failed at %s: %s", latest,
                          status["stage"] ? status["stage"].as<std::string>().c_str() : "unknown",
                          status["reason"] ? status["reason"].as<std::string>().c_str() : "unknown");
                baseline_run_ = latest;
                return;
            }
            std::string error;
            if (!loadReplayCase(directory.string(), replay_, error)) {
                ROS_ERROR("[NLP_OFFLINE] cannot load successful run %u: %s",
                          latest, error.c_str());
                return;
            }
            if (replay_.report.kinematic_integrator != "euler") {
                ROS_WARN("[NLP_OFFLINE] result %u is tagged as legacy RK4; Euler playback "
                         "may differ from its original dynamics", latest);
            }
            replay_ready_ = replay_.report.solver_success && replay_.report.validation_passed;
            if (replay_ready_) {
                result_detected_ = true;
                ROS_INFO("[NLP_OFFLINE] run %u validated; automatic playback started", latest);
            } else {
                baseline_run_ = latest;
            }
        } catch (const std::exception& exception) {
            ROS_ERROR_THROTTLE(1.0, "[NLP_OFFLINE] result status read failed: %s",
                               exception.what());
        }
    }

    void tick(const ros::WallTimerEvent& event) {
        if (!result_detected_) {
            publishSnapshot();
            detectResult();
        }
        if (replay_ready_) {
            playback_time_ = wrapReplayTime(playback_time_,
                (event.current_real - last_wall_time_).toSec(), playback_rate_,
                replay_.total_time);
            publishReplay();
        }
        last_wall_time_ = event.current_real;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher snapshot_pub_;
    ros::Publisher marker_pub_;
    ros::WallTimer timer_;
    sandbox_msgs::RecoverySnapshot snapshot_;
    std::vector<int> participant_ids_;
    ReplayCase replay_;
    std::string case_dir_;
    std::string result_case_dir_;
    unsigned int baseline_run_ = 0;
    bool result_detected_ = false;
    bool replay_ready_ = false;
    double playback_rate_ = 1.0;
    double playback_time_ = 0.0;
    int next_marker_id_ = 0;
    ros::WallTime last_wall_time_;
};

}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ros::init(argc, argv, "nlp_offline_case");
    try {
        forklift_nlp_recovery::NlpOfflineCaseNode node;
        ros::spin();
    } catch (const std::exception& exception) {
        ROS_FATAL("[NLP_OFFLINE] %s", exception.what());
        return 1;
    }
    return 0;
}
