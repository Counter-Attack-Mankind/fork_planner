#include "forklift_nlp_recovery/recovery_stage_visualization.h"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

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

void appendRecoveryStageMarkers(
    visualization_msgs::MarkerArray& markers,
    const std::string& frame_id,
    double field_width,
    double field_height,
    double containment_tolerance,
    const NlpOptimizationInput& input,
    const NlpInitialGuess& original,
    const InitialGuessGeometryRepairResult& repair,
    const std::string& prefix) {
    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size(); ++vehicle_index) {
        if (vehicle_index >= original.vehicles.size() ||
            vehicle_index >= repair.repaired_guess.vehicles.size()) continue;
        const NlpVehicleInput& source = input.vehicles[vehicle_index];
        const VehicleInitialGuess& original_vehicle = original.vehicles[vehicle_index];
        const VehicleInitialGuess& repaired_vehicle = repair.repaired_guess.vehicles[vehicle_index];
        const std::string vehicle_ns = prefix + "/V" + std::to_string(source.id);
        const bool show_comparison = vehicle_index < repair.vehicles.size() &&
                                     repair.vehicles[vehicle_index].moved_nodes > 0;

        for (size_t box_index = 0; box_index < source.corridor.size(); ++box_index) {
            const Aabb& box = source.corridor[box_index];
            auto rectangle = marker(frame_id, vehicle_ns + "/aabb_corridor",
                static_cast<int>(box_index), visualization_msgs::Marker::CUBE);
            rectangle.pose.position = point((box.min_x + box.max_x) * 0.5,
                                            (box.min_y + box.max_y) * 0.5, 0.035);
            rectangle.scale.x = box.width();
            rectangle.scale.y = box.height();
            rectangle.scale.z = 0.012;
            rectangle.color = color(0.0f, 0.85f, 0.90f, 0.12f);
            markers.markers.push_back(rectangle);
        }

        auto original_path = marker(frame_id, vehicle_ns + "/original_initial_guess",
            source.id, visualization_msgs::Marker::LINE_STRIP);
        original_path.scale.x = 0.010;
        original_path.color = color(0.05f, 0.35f, 1.0f, 1.0f);
        auto uncovered = marker(frame_id, vehicle_ns + "/original_uncovered_nodes",
            source.id, visualization_msgs::Marker::SPHERE_LIST);
        uncovered.scale.x = uncovered.scale.y = uncovered.scale.z = 0.065;
        uncovered.color = color(1.0f, 0.0f, 0.0f, 1.0f);
        for (const InitialGuessNode& node : original_vehicle.nodes) {
            if (show_comparison) {
                original_path.points.push_back(point(node.rear_axle_pose.x,
                                                     node.rear_axle_pose.y, 0.23));
            }
            const bool covered = std::any_of(source.corridor.begin(),
                source.corridor.end(), [&](const Aabb& box) {
                    return node.disc_center.x >= box.min_x - containment_tolerance &&
                           node.disc_center.x <= box.max_x + containment_tolerance &&
                           node.disc_center.y >= box.min_y - containment_tolerance &&
                           node.disc_center.y <= box.max_y + containment_tolerance;
                });
            if (!covered) uncovered.points.push_back(point(
                node.disc_center.x, node.disc_center.y, 0.28));
        }
        if (show_comparison) markers.markers.push_back(original_path);
        if (!uncovered.points.empty()) markers.markers.push_back(uncovered);

        if (show_comparison) {
            auto repaired_path = marker(frame_id, vehicle_ns + "/repaired_initial_guess",
                source.id, visualization_msgs::Marker::LINE_LIST);
            repaired_path.scale.x = 0.010;
            repaired_path.color = color(0.05f, 0.95f, 0.25f, 1.0f);
            for (size_t node = 0; node + 1 < repaired_vehicle.nodes.size(); node += 2) {
                repaired_path.points.push_back(point(repaired_vehicle.nodes[node].rear_axle_pose.x,
                    repaired_vehicle.nodes[node].rear_axle_pose.y, 0.25));
                repaired_path.points.push_back(point(repaired_vehicle.nodes[node + 1].rear_axle_pose.x,
                    repaired_vehicle.nodes[node + 1].rear_axle_pose.y, 0.25));
            }
            if (!repaired_path.points.empty()) markers.markers.push_back(repaired_path);
        }

        auto moved = marker(frame_id, vehicle_ns + "/repaired_moved_nodes",
            source.id, visualization_msgs::Marker::SPHERE_LIST);
        moved.scale.x = moved.scale.y = moved.scale.z = 0.055;
        moved.color = color(1.0f, 0.48f, 0.0f, 1.0f);
        if (vehicle_index < repair.vehicles.size()) {
            for (const RepairedNodeDiagnostic& diagnostic : repair.vehicles[vehicle_index].nodes) {
                if (diagnostic.node_index >= repaired_vehicle.nodes.size()) continue;
                const Pose2d& center = repaired_vehicle.nodes[diagnostic.node_index].disc_center;
                moved.points.push_back(point(center.x, center.y, 0.30));
            }
        }
        if (!moved.points.empty()) markers.markers.push_back(moved);

        if (vehicle_index < repair.vehicles.size() &&
            repair.vehicles[vehicle_index].repair_attempted && !show_comparison &&
            !original_vehicle.nodes.empty()) {
            auto diagnostic = marker(frame_id, vehicle_ns + "/repair_diagnostic",
                source.id, visualization_msgs::Marker::TEXT_VIEW_FACING);
            diagnostic.pose.position = point(original_vehicle.nodes.front().rear_axle_pose.x,
                original_vehicle.nodes.front().rear_axle_pose.y, 0.34);
            diagnostic.scale.z = 0.045;
            diagnostic.color = color(1.0f, 0.48f, 0.0f, 1.0f);
            diagnostic.text = "V" + std::to_string(source.id) +
                              " repair attempted, no displacement";
            markers.markers.push_back(diagnostic);
        }
    }

    auto status = marker(frame_id, prefix + "/geometry_repair_status", 0,
                         visualization_msgs::Marker::TEXT_VIEW_FACING);
    status.pose.position = point(field_width * 0.5, field_height - 0.32, 0.34);
    status.scale.z = 0.06;
    status.color = repair.generated && repair.aabb_assignment_valid ?
        color(0.05f, 0.95f, 0.25f, 1.0f) : color(1.0f, 0.15f, 0.05f, 1.0f);
    status.text = "initial guess: " + repair.reason;
    markers.markers.push_back(status);
}

}  // namespace forklift_nlp_recovery
