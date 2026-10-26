#include "forklift_nlp_recovery/optimization_input.h"

#include <sandbox_msgs/RecoverySnapshot.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

namespace forklift_nlp_recovery {
namespace {

constexpr double kEpsilon = 1e-9;

std::string quoted(const std::string& value) {
    std::string out = "\"";
    for (const char ch : value) {
        if (ch == '\\' || ch == '\"') out.push_back('\\');
        if (ch == '\n') out += "\\n";
        else if (ch != '\r') out.push_back(ch);
    }
    out.push_back('\"');
    return out;
}

void writePose(std::ostream& out, const Pose2d& pose) {
    out << "{x: " << pose.x << ", y: " << pose.y << ", yaw: " << pose.yaw << "}";
}

void writeIds(std::ostream& out, const std::vector<int>& ids) {
    out << "[";
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i != 0) out << ", ";
        out << ids[i];
    }
    out << "]";
}

}  // namespace

OptimizationInputBuilder::OptimizationInputBuilder(
    const CorridorBuilder& corridor_builder,
    const VehicleParameters& vehicle_parameters)
    : corridor_builder_(corridor_builder),
      vehicle_parameters_(vehicle_parameters) {}

const sandbox_msgs::RecoveryVehicleState* OptimizationInputBuilder::findVehicle(
    const sandbox_msgs::RecoverySnapshot& snapshot, int id) const {
    const auto found = std::find_if(
        snapshot.vehicles.begin(), snapshot.vehicles.end(),
        [id](const auto& vehicle) { return vehicle.id == id; });
    return found == snapshot.vehicles.end() ? nullptr : &*found;
}

std::vector<PathPoint> OptimizationInputBuilder::pathFromMessage(
    const sandbox_msgs::RecoveryVehicleState& vehicle) const {
    std::vector<Pose2d> poses;
    poses.reserve(vehicle.path.size());
    for (const auto& pose : vehicle.path) {
        poses.push_back({pose.x, pose.y, pose.theta});
    }
    return CorridorBuilder::parameterizePath(poses, vehicle.path_types);
}

double OptimizationInputBuilder::pointToAabbDistance(const Pose2d& point,
                                                       const Aabb& box) {
    const double dx = std::max({box.min_x - point.x, 0.0, point.x - box.max_x});
    const double dy = std::max({box.min_y - point.y, 0.0, point.y - box.max_y});
    return std::hypot(dx, dy);
}

bool OptimizationInputBuilder::discMayAffectCorridor(
    const Pose2d& center, double static_radius,
    const NlpVehicleInput& participant) {
    const double threshold = std::max(0.0, static_radius) +
                             std::max(0.0, participant.parameters.disc_radius);
    return std::any_of(
        participant.corridor.begin(), participant.corridor.end(),
        [&](const Aabb& box) {
            return pointToAabbDistance(center, box) <= threshold + kEpsilon;
        });
}

void OptimizationInputBuilder::collectStaticVehicleCandidates(
    const sandbox_msgs::RecoverySnapshot& snapshot,
    NlpOptimizationInput& input) const {
    const std::set<int> participants(input.participant_ids.begin(),
                                     input.participant_ids.end());
    for (const auto& vehicle : snapshot.vehicles) {
        if (participants.count(vehicle.id) != 0 || !vehicle.pose_valid) continue;

        StaticVehicleObstacle obstacle;
        obstacle.id = vehicle.id;
        obstacle.rear_axle_pose = {vehicle.pose.x, vehicle.pose.y,
                                   vehicle.pose.theta};
        obstacle.disc_center = corridor_builder_.bodyCenter(
            obstacle.rear_axle_pose);
        obstacle.disc_radius = vehicle_parameters_.disc_radius;
        obstacle.length = vehicle_parameters_.length;
        obstacle.width = vehicle_parameters_.width;
        obstacle.rear_axle_to_center = vehicle_parameters_.rear_axle_to_center;
        obstacle.min_distance_to_corridor = std::numeric_limits<double>::infinity();
        input.static_vehicles.push_back(std::move(obstacle));
    }
}

OptimizationInputBuildResult OptimizationInputBuilder::build(
    const sandbox_msgs::RecoverySnapshot& snapshot,
    const std::vector<int>& participant_ids,
    const std::string& source_mode) const {
    OptimizationInputBuildResult result;
    result.input.source_mode = source_mode;
    result.input.trigger_reason = snapshot.reason;
    result.input.frame_id = snapshot.header.frame_id;
    result.input.stamp_sec = snapshot.header.stamp.sec;
    result.input.stamp_nsec = snapshot.header.stamp.nsec;
    result.input.recovery_id = snapshot.recovery_id;
    result.input.participant_ids = participant_ids;

    if (participant_ids.empty()) {
        result.reason = "participant vehicle list is empty";
        result.input.reason = result.reason;
        return result;
    }

    std::set<int> unique_ids;
    std::vector<std::string> structural_errors;
    bool terminals_valid = true;
    for (const int id : participant_ids) {
        if (!unique_ids.insert(id).second) {
            structural_errors.push_back("duplicate participant V" +
                                        std::to_string(id));
            continue;
        }
        const auto* vehicle = findVehicle(snapshot, id);
        if (vehicle == nullptr) {
            structural_errors.push_back("missing participant V" +
                                        std::to_string(id));
            continue;
        }
        if (!vehicle->pose_valid) {
            structural_errors.push_back("participant V" + std::to_string(id) +
                                        " has no valid rear-axle pose");
            continue;
        }
        NlpVehicleInput input_vehicle;
        input_vehicle.id = id;
        input_vehicle.initial_state.rear_axle_pose = {
            vehicle->pose.x, vehicle->pose.y, vehicle->pose.theta};
        input_vehicle.initial_state.longitudinal_speed =
            vehicle->current_speed;
        input_vehicle.initial_state.steering_valid = vehicle->steering_valid;
        input_vehicle.initial_state.steering_estimated = vehicle->steering_estimated;
        input_vehicle.initial_state.steering_angle = vehicle->steering_angle;
        input_vehicle.path_s = vehicle->path_s;
        input_vehicle.path_gen = vehicle->path_gen;
        input_vehicle.original_path = pathFromMessage(*vehicle);
        input_vehicle.parameters = vehicle_parameters_;

        const CorridorResult corridor = corridor_builder_.build(
            input_vehicle.initial_state.rear_axle_pose,
            input_vehicle.original_path, input_vehicle.path_s,
            input_vehicle.path_gen, id);
        input_vehicle.corridor_valid = corridor.success;
        input_vehicle.corridor_reason = corridor.reason;
        input_vehicle.raw_aabb_valid = corridor.raw_aabb_valid;
        input_vehicle.raw_aabb = corridor.raw_aabb;
        input_vehicle.stc_valid = corridor.stc_valid;
        input_vehicle.safe_stc = corridor.safe_stc;
        input_vehicle.terminal_valid = corridor.terminal_valid;
        input_vehicle.terminal = corridor.terminal;
        input_vehicle.corridor = corridor.boxes;
        input_vehicle.corridor_seeds = corridor.seeds;
        terminals_valid = terminals_valid && corridor.success;
        result.input.vehicles.push_back(std::move(input_vehicle));
    }

    if (!structural_errors.empty()) {
        std::ostringstream reason;
        for (size_t i = 0; i < structural_errors.size(); ++i) {
            if (i != 0) reason << "; ";
            reason << structural_errors[i];
        }
        result.reason = reason.str();
        result.input.reason = result.reason;
        return result;
    }

    result.complete = true;
    collectStaticVehicleCandidates(snapshot, result.input);
    result.input.valid = terminals_valid;
    if (terminals_valid) {
        result.reason = "ok";
    } else {
        std::ostringstream reason;
        reason << "rejoin terminal construction failed for";
        for (const NlpVehicleInput& vehicle : result.input.vehicles) {
            if (!vehicle.corridor_valid) reason << " V" << vehicle.id;
        }
        result.reason = reason.str();
    }
    result.input.reason = result.reason;
    return result;
}

bool writeOptimizationInputDebugFile(const OptimizationInputBuildResult& result,
                                     const std::string& path,
                                     std::string& error) {
    if (path.empty()) {
        error = "debug output path is empty";
        return false;
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        error = "cannot open " + path;
        return false;
    }

    const NlpOptimizationInput& input = result.input;
    out << std::boolalpha << std::fixed << std::setprecision(9);
    out << "complete: " << result.complete << "\n";
    out << "valid: " << input.valid << "\n";
    out << "source_mode: " << quoted(input.source_mode) << "\n";
    out << "frame_id: " << quoted(input.frame_id) << "\n";
    out << "stamp: {sec: " << input.stamp_sec << ", nsec: "
        << input.stamp_nsec << "}\n";
    out << "recovery_id: " << input.recovery_id << "\n";
    out << "reason: " << quoted(result.reason) << "\n";
    out << "trigger_reason: " << quoted(input.trigger_reason) << "\n";
    out << "participant_ids: ";
    writeIds(out, input.participant_ids);
    if (input.vehicles.empty()) out << "\nvehicles: []\n";
    else out << "\nvehicles:\n";
    for (const NlpVehicleInput& vehicle : input.vehicles) {
        out << "  - id: " << vehicle.id << "\n";
        out << "    initial_rear_axle: ";
        writePose(out, vehicle.initial_state.rear_axle_pose);
        out << "\n    initial_speed: "
            << vehicle.initial_state.longitudinal_speed << "\n";
        out << "    path_s: " << vehicle.path_s << "\n";
        out << "    path_gen: " << vehicle.path_gen << "\n";
        out << "    corridor_valid: " << vehicle.corridor_valid << "\n";
        out << "    corridor_reason: " << quoted(vehicle.corridor_reason) << "\n";
        out << "    raw_aabb_valid: " << vehicle.raw_aabb_valid << "\n";
        out << "    stc_valid: " << vehicle.stc_valid << "\n";
        out << "    terminal_valid: " << vehicle.terminal_valid << "\n";
        if (vehicle.raw_aabb_valid) {
            out << "    raw_aabb: {min_x: " << vehicle.raw_aabb.min_x
                << ", max_x: " << vehicle.raw_aabb.max_x
                << ", min_y: " << vehicle.raw_aabb.min_y
                << ", max_y: " << vehicle.raw_aabb.max_y << "}\n";
        }
        if (vehicle.stc_valid) {
            out << "    safe_stc: {min_x: " << vehicle.safe_stc.min_x
                << ", max_x: " << vehicle.safe_stc.max_x
                << ", min_y: " << vehicle.safe_stc.min_y
                << ", max_y: " << vehicle.safe_stc.max_y << "}\n";
        }
        if (vehicle.terminal_valid) {
            out << "    terminal_rear_axle: ";
            writePose(out, vehicle.terminal.pose);
            out << "\n    terminal_path_s: " << vehicle.terminal.path_s << "\n";
        }
        out << "    terminal_path_gen: " << vehicle.terminal.path_gen << "\n";
        out << "    terminal_path_type: "
            << static_cast<int>(vehicle.terminal.type) << "\n";
        out << "    aabb_count: " << vehicle.corridor.size() << "\n";
        out << "    node_aabb_one_to_one: "
            << (vehicle.corridor.size() == vehicle.corridor_seeds.size()) << "\n";
        const auto& p = vehicle.parameters;
        out << "    parameters: {length: " << p.length
            << ", width: " << p.width
            << ", rear_axle_to_center: " << p.rear_axle_to_center
            << ", disc_radius: " << p.disc_radius
            << ", wheel_base: " << p.wheel_base
            << ", max_steer_angle: " << p.max_steer_angle
            << ", max_steer_rate: " << p.max_steer_rate
            << ", nominal_speed: " << p.nominal_speed
            << ", max_speed: " << p.max_speed
            << ", max_accel: " << p.max_accel
            << ", max_decel: " << p.max_decel
            << ", fixed_retreat_speed: " << p.fixed_retreat_speed << "}\n";
        out << "    original_path:\n";
        for (const PathPoint& point : vehicle.original_path) {
            out << "      - {x: " << point.pose.x << ", y: " << point.pose.y
                << ", yaw: " << point.pose.yaw << ", s: " << point.s
                << ", type: " << static_cast<int>(point.type) << "}\n";
        }
        if (vehicle.corridor.empty()) {
            out << "    aabbs: []\n";
        } else {
            out << "    aabbs:\n";
            for (size_t node = 0; node < vehicle.corridor.size(); ++node) {
                const Aabb& box = vehicle.corridor[node];
                out << "      - {node: " << node << ", min_x: " << box.min_x
                    << ", max_x: " << box.max_x
                    << ", min_y: " << box.min_y
                    << ", max_y: " << box.max_y << "}\n";
            }
        }
        if (vehicle.corridor_seeds.empty()) {
            out << "    aabb_seeds: []\n";
        } else {
            out << "    aabb_seeds:\n";
            for (size_t node = 0; node < vehicle.corridor_seeds.size(); ++node) {
                out << "      - {node: " << node << ", x: "
                    << vehicle.corridor_seeds[node].x << ", y: "
                    << vehicle.corridor_seeds[node].y << "}\n";
            }
        }
    }
    if (input.static_vehicles.empty()) out << "static_vehicles: []\n";
    else out << "static_vehicles:\n";
    for (const StaticVehicleObstacle& obstacle : input.static_vehicles) {
        out << "  - id: " << obstacle.id << "\n";
        out << "    rear_axle_pose: ";
        writePose(out, obstacle.rear_axle_pose);
        out << "\n    disc_center: ";
        writePose(out, obstacle.disc_center);
        out << "\n    disc_radius: " << obstacle.disc_radius << "\n";
        out << "    length: " << obstacle.length << "\n";
        out << "    width: " << obstacle.width << "\n";
        out << "    rear_axle_to_center: " << obstacle.rear_axle_to_center << "\n";
        out << "    min_distance_to_corridor: "
            << obstacle.min_distance_to_corridor << "\n";
        out << "    influence_distance: " << obstacle.influence_distance << "\n";
        out << "    influencing_vehicle_ids: ";
        writeIds(out, obstacle.influencing_vehicle_ids);
        out << "\n";
    }
    if (!out) {
        error = "failed while writing " + path;
        return false;
    }
    error.clear();
    return true;
}

}  // namespace forklift_nlp_recovery
