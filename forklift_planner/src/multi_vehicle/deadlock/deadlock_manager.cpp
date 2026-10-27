#include "forklift_planner/multi_vehicle/deadlock/deadlock_manager.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>

#include <ros/ros.h>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

//得到一辆车当前用于几何检测的路径纵向位置s，若休眠则证明在库位，走完路径。
double vehiclePoseS(const VehicleAgent& vehicle) {
    return vehicle.mode == VehicleMode::DWELL
               ? vehicle.track.length()
               : vehicle.path_s;
}


template <typename Callback>
//在一段路径 s 区间上按照指定步长采样，并对每个采样点执行回调
bool sampleInterval(double begin, double end, double step,
                    const Callback& callback) {
    const double direction = end >= begin ? 1.0 : -1.0;     //路径方向
    const double distance = std::abs(end - begin);          //路径长度
    const int count = std::max(1, static_cast<int>(std::ceil(distance / step)));
    for (int i = 0; i <= count; ++i) {
        const double ratio = static_cast<double>(i) / count;
        if (!callback(begin + direction * distance * ratio)) return false;
    }
    return true;
}

std::vector<const VehicleAgent*> findWaitingCycle(
    const std::vector<VehicleAgent>& vehicles) {
    std::map<int, const VehicleAgent*> eligible;
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.mode == VehicleMode::ACTIVE &&
            vehicle.action == VehicleAction::STOP &&
            vehicle.blocker_id >= 0) {
            eligible[vehicle.id] = &vehicle;
        }
    }

    for (const auto& start : eligible) {
        std::vector<const VehicleAgent*> chain;
        std::map<int, size_t> seen;
        const VehicleAgent* current = start.second;
        while (current != nullptr) {
            const auto repeated = seen.find(current->id);
            if (repeated != seen.end()) {
                std::vector<const VehicleAgent*> cycle(
                    chain.begin() + repeated->second, chain.end());
                if (cycle.size() < 2) break;
                const auto first = std::min_element(
                    cycle.begin(), cycle.end(),
                    [](const auto* a, const auto* b) { return a->id < b->id; });
                std::rotate(cycle.begin(), first, cycle.end());
                return cycle;
            }
            seen[current->id] = chain.size();
            chain.push_back(current);
            const auto next = eligible.find(current->blocker_id);
            current = next == eligible.end() ? nullptr : next->second;
        }
    }
    return {};
}

std::string vehicleListText(const std::vector<int>& vehicle_ids) {
    std::ostringstream out;
    for (size_t i = 0; i < vehicle_ids.size(); ++i) {
        if (i != 0) out << "->";
        out << "V" << vehicle_ids[i];
    }
    return out.str();
}

}  // namespace

//Recovery 状态与动作解释，用于日志调试
const char* recoveryPhaseName(RecoveryPhase phase) {
    switch (phase) {
        case RecoveryPhase::NONE: return "NONE";
        case RecoveryPhase::RETREAT: return "RETREAT";
        case RecoveryPhase::RETREAT_HOLD: return "RETREAT_HOLD";
        case RecoveryPhase::PASS: return "PASS";
        case RecoveryPhase::CLEAR: return "CLEAR";
        case RecoveryPhase::UNRESOLVED: return "UNRESOLVED";
        case RecoveryPhase::ABORT: return "ABORT";
        case RecoveryPhase::NLP_PREP: return "NLP_PREP";
    }
    return "UNKNOWN";
}

//寻找当前死锁恢复状态下，某一辆车到底应该执行RETREAT、HOLD 还是 NORMAL
RecoveryMotion RecoveryDirective::motionFor(int vehicle_id) const {

    if (phase == RecoveryPhase::RETREAT)
    {
        if (vehicle_id == retreat_vehicle_id)
            return RecoveryMotion::RETREAT;

        if (vehicle_id == pass_vehicle_id)
            return RecoveryMotion::HOLD;
    }
    else if (phase == RecoveryPhase::RETREAT_HOLD)
    {
        if (vehicle_id == retreat_vehicle_id)
            return RecoveryMotion::HOLD;
    }
    else if (phase == RecoveryPhase::PASS && hold_retreat_at_b_start)
    {
        if (vehicle_id == retreat_vehicle_id)
            return RecoveryMotion::HOLD;
    }
    else if (phase == RecoveryPhase::UNRESOLVED)
    {
        if (vehicle_id == retreat_vehicle_id ||
            vehicle_id == pass_vehicle_id) {
            return RecoveryMotion::HOLD;
        }
    }
    else if (phase == RecoveryPhase::NLP_PREP)
    {
        if (std::find(frozen_vehicle_ids.begin(), frozen_vehicle_ids.end(),
                      vehicle_id) != frozen_vehicle_ids.end()) {
            return RecoveryMotion::HOLD;
        }
    }
    //若PASS阶段都会返回NORMAL，交给普通冲突处理，是否再会造成双停
    return RecoveryMotion::NORMAL;
}


//构造函数。创建一个 DeadlockManager 对象时，这个构造函数会执行一次
DeadlockManager::DeadlockManager(const MapParam& map_param,
                                 const MultiVehicleConfig& config)
    //读取传入的 map_param 和 config，并分别保存到自己的 map_param_ 和 config_ 成员中。
    : map_param_(map_param), config_(config) {}  //成员初始化列表

//查询函数，输入id，在vehicles容器中查找对应车辆；找到就返回该车辆的只读指针，找不到就返回 nullptr
const VehicleAgent* DeadlockManager::vehicleById(
    const std::vector<VehicleAgent>& vehicles, int id) const {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.id == id) return &vehicle;
    }
    return nullptr;
}

//根据两个车辆 ID，在死锁车辆对几何信息表中查找对应的组队
//车辆顺序不敏感；找到返回指针，找不到返回空指针
const DeadlockPairGeometry* DeadlockManager::geometryFor(
    const std::vector<DeadlockPairGeometry>& geometry,
    int vehicle_a, int vehicle_b) const {
    for (const DeadlockPairGeometry& item : geometry) {
        if ((item.vehicle_a == vehicle_a && item.vehicle_b == vehicle_b) ||
            (item.vehicle_a == vehicle_b && item.vehicle_b == vehicle_a)) {
            return &item;
        }
    }
    return nullptr;
}

// 检查车辆从当前位置沿原路径到 target_s 的整个车身扫掠区域。
bool DeadlockManager::pathSweepClear(
    const VehicleAgent& moving, const VehicleAgent& protected_vehicle,
    const std::vector<VehicleAgent>& vehicles, double target_s) const {
    //确定采样步长，在0.005m和0.01m之间
    const double sweep_step = std::max(
        0.005, std::min(0.01, config_.path_validation_step));
    // sampleInterval 同时支持 s- 和 s+，沿途任一 OBB 重叠即不可行。
    return sampleInterval(moving.path_s, target_s, sweep_step,
                          [&](double moving_s) {
        const OBB body = makeBody(moving.track.poseAtS(moving_s),
                                  map_param_, 0.0);
        for (const VehicleAgent& other : vehicles) {
            if (other.id == moving.id || other.track.empty() ||
                other.mode == VehicleMode::NEED_TASK) {
                continue;
            }
            const double other_s = other.id == protected_vehicle.id
                                       ? protected_vehicle.path_s
                                       : vehiclePoseS(other);
            const OBB obstacle = makeBody(other.track.poseAtS(other_s),
                                          map_param_, 0.0);
            if (overlaps(body, obstacle)) return false;
        }
        return true;
    });
}



//把 DeadlockManager 内部的恢复事务状态 transaction_（脑子想执行的事情），同步成对外可读的执行指令 directive_（实际执行的动作）
void DeadlockManager::refreshDirective() {
    directive_ = {};
    directive_.phase = transaction_.phase;
    directive_.recovery_id = transaction_.recovery_id;
    directive_.retreat_attempt = transaction_.retreat_attempt;   //死锁最多回退次数
    directive_.retreat_vehicle_id = transaction_.retreat_vehicle_id;
    directive_.pass_vehicle_id = transaction_.pass_vehicle_id;
    directive_.retreat_path_gen = transaction_.retreat_path_gen;
    directive_.pass_path_gen = transaction_.pass_path_gen;
    directive_.retreat_target_s = transaction_.retreat_target_s;    //死锁恢复每次需要的回退的距离

   
    directive_.retreat_distance = transaction_.retreat_distance;
    directive_.hold_retreat_at_b_start =
        transaction_.hold_retreat_at_b_start;
    directive_.deadlock_cycle_ids = transaction_.deadlock_cycle_ids;
    directive_.frozen_vehicle_ids = transaction_.frozen_vehicle_ids;
    directive_.nlp_vehicle_ids = transaction_.nlp_vehicle_ids;
    directive_.reason = transaction_.reason;
}

void DeadlockManager::emit(const char* event, const std::string& details,
                           bool enabled) const {
    if (!enabled) return;
    const std::string line =
        std::string("[DEADLOCK] event=") + event + " " + details;
    if (log_sink_) log_sink_(line);
    const std::string name(event);
    if (name == "CONFIRMED" || name == "NLP_PREP" || name == "ABORT") {
        ROS_WARN_STREAM(line);
    }
}

void DeadlockManager::abort(const std::string& reason, bool emit_logs) {
    const int retreat_id = transaction_.retreat_vehicle_id;
    const int pass_id = transaction_.pass_vehicle_id;
    emit("ABORT", "pair=V" + std::to_string(retreat_id) +
                      "-V" + std::to_string(pass_id) + " reason=" + reason,
         emit_logs);
    candidate_ = {};
    transaction_ = {};
    refreshDirective();
}

void DeadlockManager::clearSuccessfulRecovery(
    const VehicleAgent* retreat, const VehicleAgent* passer,
    const std::string& reason, bool emit_logs) {
    const int retreat_id = transaction_.retreat_vehicle_id;
    const int pass_id = transaction_.pass_vehicle_id;
    const double pass_s = passer != nullptr ? passer->path_s : 0.0;
    std::ostringstream details;
    details << "pair=V" << retreat_id << "-V" << pass_id
            << " pass_s=" << pass_s
            << " reason=" << reason;
    emit("CLEAR", details.str(), emit_logs);

    candidate_ = {};
    transaction_ = {};
    refreshDirective();
}

std::vector<int> DeadlockManager::nlpVehicleIds(
    const std::vector<VehicleAgent>& vehicles,
    const std::vector<int>& deadlock_vehicle_ids) const {
    std::set<int> related(deadlock_vehicle_ids.begin(),
                          deadlock_vehicle_ids.end());
    bool changed = true;
    while (changed) {
        changed = false;
        for (const VehicleAgent& vehicle : vehicles) {
            if (vehicle.mode == VehicleMode::ACTIVE &&
                vehicle.action == VehicleAction::STOP &&
                related.count(vehicle.blocker_id) != 0) {
                changed = related.insert(vehicle.id).second || changed;
            }
        }
    }

    std::vector<int> participant_ids;
    for (const int id : related) {
        const VehicleAgent* vehicle = vehicleById(vehicles, id);
        if (vehicle == nullptr) continue;
        participant_ids.push_back(id);
    }
    return participant_ids;
}

void DeadlockManager::enterNlpRecovery(
                                       const std::vector<int>& deadlock_cycle_ids,
                                       const std::vector<int>& frozen_vehicle_ids,
                                       const std::string& reason,
                                       bool emit_logs) {
    candidate_ = {};
    transaction_ = {};
    transaction_.phase = RecoveryPhase::NLP_PREP;
    transaction_.recovery_id = next_recovery_id_++;
    transaction_.deadlock_cycle_ids = deadlock_cycle_ids;
    transaction_.frozen_vehicle_ids = frozen_vehicle_ids;
    transaction_.nlp_vehicle_ids = frozen_vehicle_ids;
    transaction_.reason = reason;
    refreshDirective();
    emit("NLP_PREP", "cycle=" + vehicleListText(deadlock_cycle_ids) +
                         " frozen=" + vehicleListText(frozen_vehicle_ids) +
                         " reason=" + reason, emit_logs);
}

bool DeadlockManager::restartNlpRecovery(
    uint64_t recovery_id, const std::vector<VehicleAgent>& vehicles,
    const std::string& reason) {
    if (transaction_.phase != RecoveryPhase::NLP_PREP || recovery_id == 0 ||
        transaction_.recovery_id != recovery_id) return false;
    const auto cycle = findWaitingCycle(vehicles);
    if (cycle.size() < 2) return false;
    std::vector<int> cycle_ids;
    cycle_ids.reserve(cycle.size());
    for (const VehicleAgent* vehicle : cycle) cycle_ids.push_back(vehicle->id);
    enterNlpRecovery(cycle_ids, nlpVehicleIds(vehicles, cycle_ids), reason, true);
    return true;
}

bool DeadlockManager::completeNlpRecovery(uint64_t recovery_id) {
    if (transaction_.phase != RecoveryPhase::NLP_PREP || recovery_id == 0 ||
        transaction_.recovery_id != recovery_id) {
        return false;
    }
    emit("NLP_FINISH", "recovery_id=" + std::to_string(recovery_id), true);
    candidate_ = {};
    transaction_ = {};
    refreshDirective();
    return true;
}

void DeadlockManager::update(const std::vector<VehicleAgent>& vehicles, const std::vector<DeadlockPairGeometry>& pair_geometry,double dt, bool emit_logs) 
{
    if (!config_.deadlock_enabled) {
        candidate_ = {};
        transaction_ = {};
        directive_ = {};
        return;
    }



    if (transaction_.phase != RecoveryPhase::NONE)
    {
        if (transaction_.phase == RecoveryPhase::NLP_PREP) {
            refreshDirective();
            return;
        }
        const VehicleAgent* retreat = vehicleById(vehicles, transaction_.retreat_vehicle_id);
        const VehicleAgent* passer = vehicleById(vehicles, transaction_.pass_vehicle_id);

        if (retreat == nullptr || passer == nullptr)
        {
            abort("recovery_vehicle_missing", emit_logs);
            return;
        }
        if (retreat->mode != VehicleMode::ACTIVE ||
            passer->mode != VehicleMode::ACTIVE ||
            retreat->path_gen != transaction_.retreat_path_gen ||
            passer->path_gen != transaction_.pass_path_gen) {
            abort("vehicle_or_path_identity_changed", emit_logs);
            return;
        }
        if (transaction_.phase == RecoveryPhase::RETREAT_HOLD)
        {
            transaction_.retreat_hold_elapsed += std::max(0.0, dt);
            if (transaction_.retreat_hold_elapsed + 1e-9 >=
                config_.rolling_refresh_period) {
                transaction_.phase = RecoveryPhase::PASS;
                transaction_.retreat_clear_elapsed = 0.0;
                transaction_.pass_clear_elapsed = 0.0;
                transaction_.reason = "retreat_hold_done_recheck";
                refreshDirective();

                std::ostringstream details;
                details << "pair=V" << retreat->id << "-V" << passer->id
                        << " retreat=V" << retreat->id
                        << " pass=V" << passer->id
                        << " attempt=" << transaction_.retreat_attempt
                        << " hold=" << config_.rolling_refresh_period
                        << " hold_at_b_start="
                        << (transaction_.hold_retreat_at_b_start ? 1 : 0)
                        << " pass_conflict_exit_s="
                        << transaction_.pass_conflict_exit_s;
                emit("RETREAT_HOLD_DONE", details.str(), emit_logs);
                emit("PASS_START", details.str(), emit_logs);
            } else {
                refreshDirective();
            }
            return;
        }
        if (transaction_.phase == RecoveryPhase::PASS)
        {
            
            const double tolerance =std::max(0.005, config_.path_validation_step);
            //判断车辆是否失活的标志位
            const bool retreat_active = retreat->action != VehicleAction::STOP;
            const bool passer_active =  passer->action != VehicleAction::STOP;
            if (transaction_.hold_retreat_at_b_start) {
                const OBB retreat_body = makeBody(
                    retreat->track.poseAtS(retreat->path_s), map_param_, 0.0);
                const OBB passer_body = makeBody(
                    passer->track.poseAtS(passer->path_s), map_param_, 0.0);
                const double passer_rear_s = std::max(
                    0.0, passer->path_s - map_param_.body_rear_ext());
                if (passer_rear_s >
                        transaction_.pass_conflict_exit_s + tolerance &&
                    !overlaps(retreat_body, passer_body)) {
                    clearSuccessfulRecovery(
                        retreat, passer, "passer_cleared_b_start_conflict",
                        emit_logs);
                    return;
                }
                // 特殊 PASS：优先车连续停止满 4 秒才进入 NLP
                if (!passer_active)
                    transaction_.pass_clear_elapsed += std::max(0.0, dt);
                else
                    transaction_.pass_clear_elapsed = 0.0;
                if (transaction_.pass_clear_elapsed + 1e-9 >=
                    2.0 * config_.rolling_refresh_period) {
                    const std::vector<int> cycle_ids{retreat->id, passer->id};
                    enterNlpRecovery(
                        cycle_ids, nlpVehicleIds(vehicles, cycle_ids),
                        "b_start_hold_passer_stopped", emit_logs);
                    return;
                }
                refreshDirective();
                return;
            }
            //分别统计两辆车连续保持非 STOP 的时间，某车持续2个决策周期没被停下，则证明死锁解除
            if (retreat_active) 
                transaction_.retreat_clear_elapsed += std::max(0.0, dt);
            else 
                transaction_.retreat_clear_elapsed = 0.0;

            if (passer_active) 
                transaction_.pass_clear_elapsed += std::max(0.0, dt);
            else 
                transaction_.pass_clear_elapsed = 0.0;
    

        // 任意一辆车连续两个 rolling period 保持活动，即认为死锁已经解除。
            const double required_clear_time = 2.0 * config_.rolling_refresh_period;
            if (transaction_.retreat_clear_elapsed + 1e-9 >=   required_clear_time || transaction_.pass_clear_elapsed + 1e-9 >=required_clear_time)
            {
                clearSuccessfulRecovery(retreat,passer,"one_vehicle_active_for_two_periods",emit_logs);
                return;
            }

            // 两车再次同时 STOP：说明本次退让没有解除死锁。
            const bool pair_stopped_again =!retreat_active && !passer_active;
            if (pair_stopped_again)
            {
                // 已经无 s- 退让空间。
                if (retreat->path_s <= tolerance)
                {
                    const std::vector<int> cycle_ids{retreat->id, passer->id};
                    enterNlpRecovery(cycle_ids, nlpVehicleIds(vehicles, cycle_ids),
                                     "no_retreat_space_and_both_stopped",
                                     emit_logs);
                    return;
                }

                // 已经完成最大退让次数。
                if (transaction_.retreat_attempt >=config_.deadlock_retreat_max_attempts)
                {
                    const std::vector<int> cycle_ids{retreat->id, passer->id};
                    enterNlpRecovery(cycle_ids, nlpVehicleIds(vehicles, cycle_ids),
                                     "max_fixed_retreat_attempts_reached",
                                     emit_logs);
                    return;
                }
                // 计算下一次后退到的s
                const double next_target_s = std::max(0.0,retreat->path_s -config_.deadlock_retreat_distance);

                if (!pathSweepClear(*retreat,*passer,vehicles,next_target_s))
                {
                    const std::vector<int> cycle_ids{retreat->id, passer->id};
                    enterNlpRecovery(cycle_ids, nlpVehicleIds(vehicles, cycle_ids),
                                     "next_retreat_sweep_blocked", emit_logs);
                    return;
                }

                ++transaction_.retreat_attempt;

                transaction_.retreat_target_s = next_target_s;
                transaction_.retreat_distance =
                retreat->path_s - next_target_s;

                transaction_.phase = RecoveryPhase::RETREAT;

                transaction_.retreat_clear_elapsed = 0.0;
                transaction_.pass_clear_elapsed = 0.0;

                transaction_.reason ="both_stopped_retry_retreat";

                refreshDirective();
                std::ostringstream retry;
                retry << "pair=V" << retreat->id << "-V" << passer->id
                      << " retreat=V" << retreat->id
                      << " pass=V" << passer->id
                      << " attempt=" << transaction_.retreat_attempt
                      << " start_s=" << retreat->path_s
                      << " target_s=" << next_target_s
                      << " reason=both_stopped_retry_retreat";
                emit("RETREAT_RETRY", retry.str(), emit_logs);
                return;
            }

            refreshDirective();
            return;
        }

        //如果处在死锁恢复阶段---真正执行的状态机
        if (transaction_.phase == RecoveryPhase::RETREAT) {
            // 每一拍都重新检查：从当前实际位置继续退到本次 target 是否仍然安全，不安全则直接退出
            if (!pathSweepClear(*retreat, *passer, vehicles,
                                transaction_.retreat_target_s))
            {   
                const std::vector<int> cycle_ids{retreat->id, passer->id};
                enterNlpRecovery(cycle_ids, nlpVehicleIds(vehicles, cycle_ids),
                                 "retreat_sweep_invalidated", emit_logs);
                return;
            }
            const double tolerance = std::max(0.005, config_.path_validation_step);
            
            // 已经到达本次固定退让目标，则低优先级车强制静止 2 秒。
            if (retreat->path_s <=transaction_.retreat_target_s + tolerance) 
            {
                const bool returned_to_b_start =
                    retreat->slot_departure_clear_s > 0.0 &&
                    std::abs(transaction_.retreat_target_s) <= tolerance &&
                    transaction_.pass_conflict_exit_s >= 0.0;
                if (returned_to_b_start) {
                    VehicleAgent source_retreat = *retreat;
                    source_retreat.path_s = 0.0;
                    const double pass_check_s = std::min(
                        passer->track.length(),
                        std::max(passer->path_s,
                                 transaction_.pass_conflict_exit_s +
                                     map_param_.body_rear_ext() + tolerance));
                    const std::vector<VehicleAgent> pair{
                        source_retreat, *passer};
                    transaction_.hold_retreat_at_b_start =
                        pathSweepClear(*passer, source_retreat, pair,
                                       pass_check_s);
                }
                transaction_.phase = RecoveryPhase::RETREAT_HOLD;
                transaction_.retreat_hold_elapsed = 0.0;
                transaction_.retreat_clear_elapsed = 0.0;
                transaction_.pass_clear_elapsed = 0.0;
                transaction_.reason = "retreat_step_done_hold";
                refreshDirective();

                std::ostringstream details;
                details << "pair=V" << retreat->id
                << "-V" << passer->id
                << " retreat=V" << retreat->id
                << " pass=V" << passer->id
                << " attempt=" << transaction_.retreat_attempt
                << " target_s=" << transaction_.retreat_target_s
                << " actual_s=" << retreat->path_s
                << " hold_at_b_start="
                << (transaction_.hold_retreat_at_b_start ? 1 : 0);
                emit("RETREAT_DONE", details.str(), emit_logs);
                emit("RETREAT_HOLD_START", details.str(), emit_logs);
            }
            return;
        }
        return;
    }


    const std::vector<const VehicleAgent*> cycle = findWaitingCycle(vehicles);
    if (cycle.empty()) {
        candidate_ = {};
        refreshDirective();
        return;
    }

    const DeadlockPairGeometry* geometry = cycle.size() == 2
        ? geometryFor(pair_geometry, cycle[0]->id, cycle[1]->id) : nullptr;
    if (cycle.size() == 2 && geometry == nullptr) {
        candidate_ = {};
        refreshDirective();
        return;
    }

    std::vector<int> cycle_ids;
    std::vector<int> cycle_path_gens;
    std::vector<double> cycle_s;
    cycle_ids.reserve(cycle.size());
    cycle_path_gens.reserve(cycle.size());
    cycle_s.reserve(cycle.size());
    for (const VehicleAgent* vehicle : cycle) {
        cycle_ids.push_back(vehicle->id);
        cycle_path_gens.push_back(vehicle->path_gen);
        cycle_s.push_back(vehicle->path_s);
    }

    const double progress_epsilon = std::max(0.005, config_.path_validation_step);
    const bool same_candidate = candidate_.valid &&
        candidate_.vehicle_ids == cycle_ids &&
        candidate_.path_gens == cycle_path_gens;
    bool progressed = !same_candidate ||
        candidate_.anchor_s.size() != cycle_s.size();
    if (!progressed) {
        for (size_t i = 0; i < cycle_s.size(); ++i) {
            if (std::abs(cycle_s[i] - candidate_.anchor_s[i]) >
                progress_epsilon) {
                progressed = true;
                break;
            }
        }
    }
    if (progressed) {
        candidate_ = {};
        candidate_.valid = true;
        candidate_.vehicle_ids = cycle_ids;
        candidate_.path_gens = cycle_path_gens;
        candidate_.anchor_s = cycle_s;
        candidate_.duration = dt;
        std::ostringstream details;
        details << "cycle=" << vehicleListText(cycle_ids);
        for (const VehicleAgent* vehicle : cycle) {
            details << " V" << vehicle->id << "_gen=" << vehicle->path_gen
                    << "_s=" << vehicle->path_s
                    << "_reason=" << vehicle->reason
                    << "_blocker=V" << vehicle->blocker_id;
        }
        emit("CANDIDATE", details.str(), emit_logs);
        return;
    }

    candidate_.duration += dt;
    if (candidate_.duration + 1e-9 < config_.deadlock_confirm_time) return;

    std::ostringstream confirmed;
    confirmed << "cycle=" << vehicleListText(cycle_ids)
              << " duration=" << candidate_.duration;
    emit("CONFIRMED", confirmed.str(), emit_logs);

    if (cycle.size() >= 3) {
        enterNlpRecovery(cycle_ids, nlpVehicleIds(vehicles, cycle_ids),
                         "multi_vehicle_wait_cycle", emit_logs);
        return;
    }

    const VehicleAgent* candidate_a = cycle[0];
    const VehicleAgent* candidate_b = cycle[1];

    //==============新的死锁更新恢复逻辑==============
    const int preferred_priority_id = geometry->preferred_priority_vehicle_id;
    //冻结死锁恢复角色，一定是低优先级退让，高优先级清出，这样才不会出现TTC打架
    const VehicleAgent* passer = nullptr;
    const VehicleAgent* retreat = nullptr;
    
    if (preferred_priority_id == candidate_a->id)
    {
        passer = candidate_a;
        retreat = candidate_b;
    } 
    else
    {
        passer = candidate_b;
        retreat = candidate_a;
    } 

    candidate_ = {};
    transaction_ = {};
    transaction_.retreat_vehicle_id = retreat->id;
    transaction_.pass_vehicle_id = passer->id;
    transaction_.retreat_path_gen = retreat->path_gen;
    transaction_.pass_path_gen = passer->path_gen;
    transaction_.pass_conflict_exit_s = geometry->priority_conflict_exit_s;

    const double tolerance = std::max(0.005, config_.path_validation_step);
    // 若已经在路径起点，不再退；B 起点仍执行一个完整滚动周期的静止观察。
    if (retreat->path_s <= tolerance) 
    {
        const bool starts_at_b_slot = retreat->slot_departure_clear_s > 0.0;
        transaction_.phase = starts_at_b_slot
            ? RecoveryPhase::RETREAT_HOLD : RecoveryPhase::PASS;
        transaction_.retreat_attempt = 0;
        transaction_.retreat_target_s = 0.0;
        transaction_.retreat_hold_elapsed = 0.0;
        transaction_.retreat_clear_elapsed = 0.0;
        transaction_.pass_clear_elapsed = 0.0;
        transaction_.reason = starts_at_b_slot
            ? "retreater_at_b_start_hold"
            : "retreater_already_at_path_start";
        refreshDirective();
        emit(starts_at_b_slot ? "RETREAT_HOLD_START" : "PASS_START",
             "pair=V" + std::to_string(retreat->id) + "-V" +
                 std::to_string(passer->id) + " retreat=V" +
                 std::to_string(retreat->id) + " pass=V" +
                 std::to_string(passer->id) +
                 " reason=" + transaction_.reason,
             emit_logs);
        return;
    }

    // B 起点在最大三次退让范围内时，首次直接安全扫掠回 s=0；其余路径仍固定退 0.5 m。
    const double max_total_retreat = config_.deadlock_retreat_distance *
        static_cast<double>(config_.deadlock_retreat_max_attempts);
    bool return_to_b_start = retreat->slot_departure_clear_s > 0.0 &&
        retreat->path_s <= max_total_retreat + tolerance;
    double target_s = return_to_b_start
        ? 0.0
        : std::max(0.0, retreat->path_s - config_.deadlock_retreat_distance);
    bool sweep_clear = pathSweepClear(*retreat, *passer, vehicles, target_s);
    if (return_to_b_start && !sweep_clear) {
        return_to_b_start = false;
        target_s = std::max(
            0.0, retreat->path_s - config_.deadlock_retreat_distance);
        sweep_clear = pathSweepClear(*retreat, *passer, vehicles, target_s);
    }
    if (!sweep_clear)
    {
        std::ostringstream details;
        details << "pair=V" << retreat->id << "-V" << passer->id
                << " retreat=V" << retreat->id << " pass=V" << passer->id
                << " start_s=" << retreat->path_s << " target_s=" << target_s
                << " attempt=1 sweep_clear=0 reason=first_retreat_sweep_blocked";
        emit("FIXED_RETREAT_FAILED", details.str(), emit_logs);
        const std::vector<int> cycle_ids{retreat->id, passer->id};
        enterNlpRecovery(cycle_ids, nlpVehicleIds(vehicles, cycle_ids),
                         "first_retreat_sweep_blocked", emit_logs);
        return;
    }

    transaction_.phase = RecoveryPhase::RETREAT;
    transaction_.retreat_attempt = 1;
    transaction_.retreat_target_s = target_s;
    transaction_.retreat_distance =
    retreat->path_s - target_s;

    transaction_.reason = return_to_b_start
        ? "priority_yielding_return_to_b_start"
        : "priority_yielding_fixed_retreat";
    refreshDirective();

    std::ostringstream selection;
    selection << "pair=V" << candidate_a->id << "-V" << candidate_b->id
          << " priority=V" << passer->id
          << " retreat=V" << retreat->id
          << " attempt=1"
          << " start_s=" << retreat->path_s 
          << " target_s=" << target_s
          << " distance=" << transaction_.retreat_distance
          << " return_to_b_start=" << (return_to_b_start ? 1 : 0)
          << " sweep_clear=1 reason=" << transaction_.reason;

    emit("SELECT", selection.str(), emit_logs);
}

DeadlockManager::Snapshot DeadlockManager::snapshot() const {
    return Snapshot{candidate_, transaction_, directive_};
}

void DeadlockManager::restore(const Snapshot& snapshot) {
    candidate_ = snapshot.candidate;
    transaction_ = snapshot.transaction;
    directive_ = snapshot.directive;
}



}  // namespace multi_vehicle
}  // namespace forklift_planner
