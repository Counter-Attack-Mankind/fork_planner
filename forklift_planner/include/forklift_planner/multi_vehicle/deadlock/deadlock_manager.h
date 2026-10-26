#pragma once

#include <functional>
#include <cstdint>
#include <string>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

//死锁恢复状态机的阶段,也就是车辆进入死锁，可能会出现的阶段
enum class RecoveryPhase {
    NONE,
    RETREAT,
    RETREAT_HOLD,
    PASS,
    CLEAR,
    UNRESOLVED,
    ABORT,
    NLP_PREP,
};

//车辆在死锁阶段所要采取的动作
enum class RecoveryMotion {
    NORMAL,
    HOLD,
    RETREAT,
};

//DeadlockManager 对外发布的“当前死锁恢复指令”
struct RecoveryDirective {
    RecoveryPhase phase = RecoveryPhase::NONE;
    uint64_t recovery_id = 0;
    int retreat_attempt = 0;   // 当前第几次固定退让，目前设定容忍退让三次
    int retreat_vehicle_id = -1;
    int pass_vehicle_id = -1;
    int retreat_path_gen = -1;
    int pass_path_gen = -1;
    double retreat_target_s = 0.0;
    double retreat_distance = 0.0;
    std::vector<int> deadlock_cycle_ids;
    std::vector<int> frozen_vehicle_ids;
    std::vector<int> nlp_vehicle_ids;
    std::string reason;

    bool active() const {
        return phase != RecoveryPhase::NONE && phase != RecoveryPhase::CLEAR;
    }

    bool nlpTakeover() const { return phase == RecoveryPhase::NLP_PREP; }

    RecoveryMotion motionFor(int vehicle_id) const;
};

struct DeadlockPairGeometry {
    int vehicle_a = -1;
    int vehicle_b = -1;
    int path_gen_a = -1;
    int path_gen_b = -1;
    int preferred_priority_vehicle_id = -1;
};


class DeadlockManager {
public:
    struct CandidateState {
        bool valid = false;
        std::vector<int> vehicle_ids;
        std::vector<int> path_gens;
        std::vector<double> anchor_s;
        double duration = 0.0;
    };

    //DeadlockManager 内部真正执行中的恢复事务状态。
    struct TransactionState {
        RecoveryPhase phase = RecoveryPhase::NONE;
        uint64_t recovery_id = 0;
        int retreat_attempt = 0;   // 当前 recovery 已执行到第几次退让（真实操作）
        int retreat_vehicle_id = -1;
        int pass_vehicle_id = -1;
        int retreat_path_gen = -1;
        int pass_path_gen = -1;
        double retreat_target_s = 0.0;
        double retreat_distance = 0.0;
        double retreat_hold_elapsed = 0.0;
        double retreat_clear_elapsed = 0.0; //两车独立计时器，判断2周期内是否有车没有失活，则判断死锁解除
        double pass_clear_elapsed = 0.0;
        std::vector<int> deadlock_cycle_ids;
        std::vector<int> frozen_vehicle_ids;
        std::vector<int> nlp_vehicle_ids;
        std::string reason;
    };


    struct Snapshot {
        CandidateState candidate;
        TransactionState transaction;
        RecoveryDirective directive;
    };

    DeadlockManager(const MapParam& map_param,
                    const MultiVehicleConfig& config);

    void setLogSink(const std::function<void(const std::string&)>& sink) {
        log_sink_ = sink;
    }

    void update(const std::vector<VehicleAgent>& vehicles,
                const std::vector<DeadlockPairGeometry>& pair_geometry,
                double dt, bool emit_logs);

    const RecoveryDirective& directive() const { return directive_; }

    bool retreatSweepClear(const VehicleAgent& retreat,
                           const VehicleAgent& passer,
                           const std::vector<VehicleAgent>& vehicles,
                           double target_s) const;

    Snapshot snapshot() const;
    void restore(const Snapshot& snapshot);
    bool completeNlpRecovery(uint64_t recovery_id);
    bool restartNlpRecovery(uint64_t recovery_id,
                            const std::vector<VehicleAgent>& vehicles,
                            const std::string& reason);

private:


    const VehicleAgent* vehicleById(const std::vector<VehicleAgent>& vehicles,
                                    int id) const;
    const DeadlockPairGeometry* geometryFor(
        const std::vector<DeadlockPairGeometry>& geometry,
        int vehicle_a, int vehicle_b) const;
    void refreshDirective();
    void emit(const char* event, const std::string& details, bool enabled) const;
    void abort(const std::string& reason, bool emit_logs);
    void clearSuccessfulRecovery(const VehicleAgent* retreat,
                                 const VehicleAgent* passer,
                                 const std::string& reason,
                                 bool emit_logs);
    std::vector<int> nlpVehicleIds(
        const std::vector<VehicleAgent>& vehicles,
        const std::vector<int>& deadlock_vehicle_ids) const;
    void enterNlpRecovery(const std::vector<int>& deadlock_cycle_ids,
                          const std::vector<int>& frozen_vehicle_ids,
                          const std::string& reason, bool emit_logs);

    const MapParam& map_param_;
    const MultiVehicleConfig& config_;
    CandidateState candidate_;
    TransactionState transaction_;
    RecoveryDirective directive_;
    uint64_t next_recovery_id_ = 1;
    std::function<void(const std::string&)> log_sink_;
};

const char* recoveryPhaseName(RecoveryPhase phase);

}  // namespace multi_vehicle
}  // namespace forklift_planner
