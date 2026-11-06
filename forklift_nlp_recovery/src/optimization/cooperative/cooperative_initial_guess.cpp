#include "forklift_nlp_recovery/cooperative/cooperative_initial_guess.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace forklift_nlp_recovery {

CooperativeInitialGuessGenerator::CooperativeInitialGuessGenerator(
    const InitialGuessConfig& config) : config_(config) {}

NlpInitialGuess CooperativeInitialGuessGenerator::generate(
    const NlpOptimizationInput& input) const {
    NlpInitialGuess guess;
    if (!input.valid || input.vehicles.size() != 3 || config_.node_count < 2 ||
        config_.time_margin <= 0.0) {
        guess.reason = "cooperative initial guess requires valid input for exactly three vehicles";
        return guess;
    }

    std::set<int> ids;
    double estimated_time = 0.0;
    for (const NlpVehicleInput& vehicle : input.vehicles) {
        if (!ids.insert(vehicle.id).second || !vehicle.stc_valid ||
            !vehicle.terminal_valid || vehicle.initial_state_corrected ||
            vehicle.parameters.max_speed <= 0.0) {
            guess.reason = "cooperative input has duplicate IDs, invalid STC/terminal, or a corrected start";
            return guess;
        }
        const Pose2d& start = vehicle.frozen_initial_state.rear_axle_pose;
        const double distance = std::hypot(vehicle.terminal.pose.x - start.x,
                                           vehicle.terminal.pose.y - start.y);
        const double speed = vehicle.parameters.nominal_speed > 0.0 ?
            std::min(vehicle.parameters.nominal_speed, vehicle.parameters.max_speed) :
            vehicle.parameters.max_speed;
        estimated_time = std::max(estimated_time, distance / std::max(1e-3, speed));
    }
    guess.node_count = config_.node_count;
    guess.total_time = std::max(1.0, estimated_time * config_.time_margin);
    guess.average_dt = guess.total_time / static_cast<double>(guess.node_count - 1);
    guess.time_grid.resize(guess.node_count);
    for (size_t node = 0; node < guess.node_count; ++node) {
        guess.time_grid[node] = guess.average_dt * static_cast<double>(node);
    }

    // 对每辆车的初始解生成，
    for (const NlpVehicleInput& vehicle : input.vehicles) {
        VehicleInitialGuess vehicle_guess;
        vehicle_guess.id = vehicle.id;
        vehicle_guess.valid = true;
        vehicle_guess.reason = "stationary motion guess";
        vehicle_guess.nodes.resize(guess.node_count);
        const Pose2d start = vehicle.frozen_initial_state.rear_axle_pose;
        const double steer = vehicle.frozen_initial_state.steering_valid ?
            vehicle.frozen_initial_state.steering_angle : 0.0;
        for (size_t node = 0; node < guess.node_count; ++node) {
            InitialGuessNode& value = vehicle_guess.nodes[node];
            value.index = node;
            value.time = guess.time_grid[node];
            value.path_s = vehicle.frozen_path_s;
            value.rear_axle_pose = start;
            value.disc_center = {start.x + vehicle.parameters.rear_axle_to_center * std::cos(start.yaw),
                                 start.y + vehicle.parameters.rear_axle_to_center * std::sin(start.yaw),
                                 start.yaw};
            value.front_steer = steer;
            value.aabb_index = -1;
        }
        guess.vehicles.push_back(std::move(vehicle_guess));
    }
    guess.generated = true;
    guess.constraints_satisfied = false;
    guess.reason = "stationary three-vehicle IPOPT initial guess";
    return guess;
}

}  // namespace forklift_nlp_recovery
