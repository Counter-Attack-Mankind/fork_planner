#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sandbox_msgs/NlpBacktrackCommand.h>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

struct NlpBacktrackExecutorConfig {
    double path_validation_step = 0.01;
    double max_distance = 0.50;
    double max_accel = 0.20;
    double max_decel = 0.30;
};

class NlpBacktrackExecutor {
public:
    enum Status : uint8_t {
        NONE = 0,
        MOVING = 1,
        SUCCEEDED = 2,
        SAFETY_FAILED = 3,
        NO_EXECUTABLE = 4,
    };

    NlpBacktrackExecutor(const MapParam& map_param,
                         const std::vector<ShelfBlock>& shelves,
                         const NlpBacktrackExecutorConfig& config);

    bool begin(const sandbox_msgs::NlpBacktrackCommand& command,
               uint64_t recovery_id, uint32_t solve_round,
               const std::vector<int>& deadlock_cycle_ids,
               std::vector<VehicleAgent>& vehicles, std::string& rejection_reason);
    void step(double dt, uint64_t recovery_id, std::vector<VehicleAgent>& vehicles);

    bool active() const { return active_; }
    uint8_t status() const { return status_; }
    uint64_t recoveryId() const { return recovery_id_; }
    uint32_t solveRound() const { return solve_round_; }
    int vehicleId() const { return vehicle_id_; }
    int pathGen() const { return path_gen_; }
    double targetS() const { return target_s_; }
    const std::string& reason() const { return reason_; }

private:
    struct CollisionInfo {
        int blocker_id = -1;
        double path_s = 0.0;
        RoughWp pose;
    };

    struct Request {
        int vehicle_id = -1;
        int path_gen = -1;
        double target_s = 0.0;
        bool hold_at_source_b = false;
        double start_s = 0.0;
    };

    VehicleAgent* findVehicle(std::vector<VehicleAgent>& vehicles, int id) const;
    const VehicleAgent* findVehicle(const std::vector<VehicleAgent>& vehicles, int id) const;
    bool poseSafe(const VehicleAgent& moving, const RoughWp& pose,
                  const std::vector<VehicleAgent>& vehicles,
                  int* blocker_id = nullptr) const;
    bool sweepClear(const VehicleAgent& moving, double target_s,
                    const std::vector<VehicleAgent>& vehicles,
                    CollisionInfo& collision) const;
    bool startCurrent(std::vector<VehicleAgent>& vehicles);
    void finishCurrentBlocked(VehicleAgent& vehicle, const CollisionInfo& collision,
                              std::vector<VehicleAgent>& vehicles);
    void finishBatch();
    void fail(const std::string& reason);

    MapParam map_param_;
    NlpBacktrackExecutorConfig config_;
    std::vector<Request> requests_;
    size_t request_index_ = 0;
    bool active_ = false;
    uint8_t status_ = NONE;
    uint64_t recovery_id_ = 0;
    uint32_t solve_round_ = 0;
    double speed_ = 0.0;
    int vehicle_id_ = -1;
    int path_gen_ = -1;
    double target_s_ = 0.0;
    size_t completed_count_ = 0;
    size_t skipped_count_ = 0;
    size_t moved_count_ = 0;
    std::string reason_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
