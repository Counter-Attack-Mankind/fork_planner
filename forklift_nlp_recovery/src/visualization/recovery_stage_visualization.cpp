#include "forklift_nlp_recovery/recovery_stage_visualization.h"

#include <cmath>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace forklift_nlp_recovery {
namespace {

std_msgs::ColorRGBA color(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA value;
    value.r = r; value.g = g; value.b = b; value.a = a;
    return value;
}

geometry_msgs::Point point(double x, double y, double z) {
    geometry_msgs::Point value;
    value.x = x; value.y = y; value.z = z;
    return value;
}

visualization_msgs::Marker marker(const std::string& frame_id,
                                  const std::string& ns, int id, int type) {
    visualization_msgs::Marker value;
    value.header.frame_id = frame_id;
    value.header.stamp = ros::Time::now();
    value.ns = ns;
    value.id = id;
    value.type = type;
    value.action = visualization_msgs::Marker::ADD;
    value.pose.orientation.w = 1.0;
    return value;
}

}  // namespace

void appendRecoveryStageMarkers(visualization_msgs::MarkerArray& markers,
    const std::string& frame_id, double field_width, double field_height,
    double containment_tolerance, const NlpOptimizationInput& input,
    const NlpInitialGuess& original, const NlpInitialGuess& actual,
    const std::string& prefix) {
    (void)containment_tolerance;
    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size(); ++vehicle_index) {
        if (vehicle_index >= original.vehicles.size() || vehicle_index >= actual.vehicles.size()) continue;
        const NlpVehicleInput& source = input.vehicles[vehicle_index];
        const VehicleInitialGuess& before = original.vehicles[vehicle_index];
        const VehicleInitialGuess& used = actual.vehicles[vehicle_index];
        const std::string vehicle_ns = prefix + "/V" + std::to_string(source.id);
        for (size_t node = 0; node < source.corridor.size(); ++node) {
            const Aabb& box = source.corridor[node];
            auto rectangle = marker(frame_id, vehicle_ns + "/node_aabbs",
                                    static_cast<int>(node), visualization_msgs::Marker::CUBE);
            rectangle.pose.position = point(0.5 * (box.min_x + box.max_x),
                                            0.5 * (box.min_y + box.max_y), 0.035);
            rectangle.scale.x = std::max(0.002, box.width());
            rectangle.scale.y = std::max(0.002, box.height());
            rectangle.scale.z = 0.012;
            rectangle.color = color(0.0f, 0.85f, 0.90f, 0.12f);
            markers.markers.push_back(rectangle);
        }
        auto original_path = marker(frame_id, vehicle_ns + "/pre_offset_initial_guess",
                                    source.id, visualization_msgs::Marker::LINE_STRIP);
        original_path.scale.x = 0.010;
        original_path.color = color(0.05f, 0.35f, 1.0f, 1.0f);
        for (const InitialGuessNode& node : before.nodes)
            original_path.points.push_back(point(node.rear_axle_pose.x, node.rear_axle_pose.y, 0.23));
        markers.markers.push_back(original_path);

        auto actual_path = marker(frame_id, vehicle_ns + "/nlp_initial_guess",
                                  source.id, visualization_msgs::Marker::LINE_STRIP);
        actual_path.scale.x = 0.012;
        actual_path.color = color(0.05f, 0.95f, 0.25f, 1.0f);
        auto shifted = marker(frame_id, vehicle_ns + "/shifted_seed_nodes",
                              source.id, visualization_msgs::Marker::SPHERE_LIST);
        shifted.scale.x = shifted.scale.y = shifted.scale.z = 0.055;
        shifted.color = color(1.0f, 0.48f, 0.0f, 1.0f);
        for (const InitialGuessNode& node : used.nodes) {
            actual_path.points.push_back(point(node.rear_axle_pose.x, node.rear_axle_pose.y, 0.25));
            if (std::hypot(node.seed_offset_x, node.seed_offset_y) > 0.0)
                shifted.points.push_back(point(node.disc_center.x, node.disc_center.y, 0.30));
        }
        markers.markers.push_back(actual_path);
        if (!shifted.points.empty()) markers.markers.push_back(shifted);
    }
    auto status = marker(frame_id, prefix + "/initial_guess_status", 0,
                         visualization_msgs::Marker::TEXT_VIEW_FACING);
    status.pose.position = point(field_width * 0.5, field_height - 0.32, 0.34);
    status.scale.z = 0.06;
    status.color = actual.generated ? color(0.05f, 0.95f, 0.25f, 1.0f) :
                                      color(1.0f, 0.15f, 0.05f, 1.0f);
    status.text = "initial guess: " + actual.reason;
    markers.markers.push_back(status);
}

}  // namespace forklift_nlp_recovery
