#include "forklift_planner/multi_vehicle/deadlock/deadlock_manager.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <limits>
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

}  // namespace

//Recovery 状态与动作解释，用于日志调试
const char* recoveryPhaseName(RecoveryPhase phase) {
    switch (phase) {
        case RecoveryPhase::NONE: return "NONE";
        case RecoveryPhase::RETREAT: return "RETREAT";
        case RecoveryPhase::PASS: return "PASS";
        case RecoveryPhase::CLEAR: return "CLEAR";
        case RecoveryPhase::UNRESOLVED: return "UNRESOLVED";
        case RecoveryPhase::ABORT: return "ABORT";
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
    else if (phase == RecoveryPhase::UNRESOLVED)
    {
        if (vehicle_id == retreat_vehicle_id ||
            vehicle_id == pass_vehicle_id) {
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

//检查 retreat 车辆从当前位置一路倒到 target_s 的整个车身扫掠区域，会不会撞到任何其他车辆。
//判断向后退，是否会撞到其他车
bool DeadlockManager::retreatSweepClear(
    const VehicleAgent& retreat, const VehicleAgent& passer,
    const std::vector<VehicleAgent>& vehicles, double target_s) const {
    //确定采样步长，在0.005m和0.01m之间
    const double sweep_step = std::max(
        0.005, std::min(0.01, config_.path_validation_step));
    //创建匿名函数，从当前 path_s 一直采样到 target_s，检查后退车辆沿途每一个采样位置是否会和其他车辆车身重叠；
    //只要有一个位置发生碰撞，就返回 false；全部位置都安全，返回 true
    return sampleInterval(retreat.path_s, target_s, sweep_step,
                          [&](double retreat_s) {
        const OBB body = makeBody(retreat.track.poseAtS(retreat_s),
                                  map_param_, 0.0);
        for (const VehicleAgent& other : vehicles) {
            if (other.id == retreat.id || other.track.empty() ||
                other.mode == VehicleMode::NEED_TASK) {
                continue;
            }
            const double other_s = other.id == passer.id
                                       ? passer.path_s
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
    directive_.retreat_attempt = transaction_.retreat_attempt;   //死锁最多回退次数
    directive_.retreat_vehicle_id = transaction_.retreat_vehicle_id;
    directive_.pass_vehicle_id = transaction_.pass_vehicle_id;
    directive_.retreat_path_gen = transaction_.retreat_path_gen;
    directive_.pass_path_gen = transaction_.pass_path_gen;
    directive_.retreat_target_s = transaction_.retreat_target_s;    //死锁恢复每次需要的回退的距离

   
    directive_.retreat_distance = transaction_.retreat_distance;
    directive_.reason = transaction_.reason;
}

void DeadlockManager::emit(const char* event, const std::string& details,
                           bool enabled) const {
    if (!enabled) return;
    const std::string line =
        std::string("[DEADLOCK] event=") + event + " " + details;
    if (log_sink_) log_sink_(line);
    const std::string name(event);
    if (name == "CONFIRMED" || name == "SELECT" ||
        name == "RETREAT_DONE" || name == "PASS_START" ||
        name == "CLEAR" || name == "UNRESOLVED" || name == "ABORT") {
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
        const VehicleAgent* retreat = vehicleById(vehicles, transaction_.retreat_vehicle_id);
        const VehicleAgent* passer = vehicleById(vehicles, transaction_.pass_vehicle_id);

        if (retreat == nullptr || passer == nullptr)
        {
            abort("recovery_vehicle_missing", emit_logs);
            return;
        }
        if (transaction_.phase == RecoveryPhase::PASS)
        {
            
            const double tolerance =std::max(0.005, config_.path_validation_step);
            //判断车辆是否失活的标志位
            const bool retreat_active = retreat->action != VehicleAction::STOP;
            const bool passer_active =  passer->action != VehicleAction::STOP;
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
                    transaction_.phase = RecoveryPhase::UNRESOLVED;
                    transaction_.reason ="no_retreat_space_and_both_stopped";
                    refreshDirective();
                    emit("UNRESOLVED","pair=V" + std::to_string(retreat->id) +"-V" + std::to_string(passer->id) +" reason=no_retreat_space_and_both_stopped",emit_logs);
                    return;
                }

                // 已经完成最大退让次数。
                if (transaction_.retreat_attempt >=config_.deadlock_retreat_max_attempts)
                {
                    transaction_.phase = RecoveryPhase::UNRESOLVED;
                    transaction_.reason ="max_fixed_retreat_attempts_reached";
                    refreshDirective();
                    emit("UNRESOLVED","pair=V" + std::to_string(retreat->id) +"-V" + std::to_string(passer->id) +" reason=max_fixed_retreat_attempts_reached",emit_logs);
                    return;
                }
                // 计算下一次后退到的s
                const double next_target_s = std::max(0.0,retreat->path_s -config_.deadlock_retreat_distance);

                if (!retreatSweepClear(*retreat,*passer,vehicles,next_target_s))
                {
                    transaction_.phase = RecoveryPhase::UNRESOLVED;
                    transaction_.reason ="next_retreat_sweep_blocked";
                    refreshDirective();
                    emit("UNRESOLVED","pair=V" + std::to_string(retreat->id) +"-V" + std::to_string(passer->id) +" reason=next_retreat_sweep_blocked",emit_logs);
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
                return;
            }

            refreshDirective();
            return;
        }

        if (
            retreat->mode != VehicleMode::ACTIVE ||
            passer->mode != VehicleMode::ACTIVE ||
            retreat->path_gen != transaction_.retreat_path_gen ||
            passer->path_gen != transaction_.pass_path_gen) {
            abort("vehicle_or_path_identity_changed", emit_logs);
            return;
        }

        //如果处在死锁恢复阶段---真正执行的状态机
        if (transaction_.phase == RecoveryPhase::RETREAT) {
            // 每一拍都重新检查：从当前实际位置继续退到本次 target 是否仍然安全，不安全则直接退出
            if (!retreatSweepClear(*retreat, *passer, vehicles,transaction_.retreat_target_s)) 
            {   
                abort("retreat_sweep_invalidated", emit_logs);
                return;
            }
            const double tolerance = std::max(0.005, config_.path_validation_step);
            
            // 已经到达本次固定退让目标，则低优先级车停住，优先车恢复 NORMAL，进入观察阶段。
            if (retreat->path_s <=transaction_.retreat_target_s + tolerance) 
            {
                transaction_.phase = RecoveryPhase::PASS;
                transaction_.retreat_clear_elapsed = 0.0;
                transaction_.pass_clear_elapsed = 0.0;
                transaction_.reason = "retreat_step_done_recheck";
                refreshDirective();

                std::ostringstream details;
                details << "pair=V" << retreat->id
                << "-V" << passer->id
                << " retreat=V" << retreat->id
                << " pass=V" << passer->id
                << " attempt=" << transaction_.retreat_attempt
                << " target_s=" << transaction_.retreat_target_s
                << " actual_s=" << retreat->path_s;
                emit("RETREAT_DONE", details.str(), emit_logs);
                emit("PASS_START", details.str(), emit_logs);
            }
            return;
        }
        return;
    }


    const VehicleAgent* candidate_a = nullptr;
    const VehicleAgent* candidate_b = nullptr;
    for (const VehicleAgent& a : vehicles) {
        if (a.mode != VehicleMode::ACTIVE ||
            a.action != VehicleAction::STOP || a.blocker_id < 0) {
            continue;
        }

        const VehicleAgent* b = vehicleById(vehicles, a.blocker_id);
        if (b == nullptr || b->mode != VehicleMode::ACTIVE ||
            b->action != VehicleAction::STOP || b->blocker_id != a.id) {
            continue;
        }

        if (a.id < b->id) {
            candidate_a = &a;
            candidate_b = b;
            break;
        }
    }

    if (candidate_a == nullptr) {
        candidate_ = {};
        refreshDirective();
        return;
    }

    const DeadlockPairGeometry* observed_geometry = geometryFor(
        pair_geometry, candidate_a->id, candidate_b->id);
    if (observed_geometry == nullptr) {
        candidate_ = {};
        directive_ = {};
        return;
    }

    const double progress_epsilon = std::max(0.005, config_.path_validation_step);
    const bool same_candidate = candidate_.valid &&
        candidate_.vehicle_a == candidate_a->id &&
        candidate_.vehicle_b == candidate_b->id &&
        candidate_.path_gen_a == candidate_a->path_gen &&
        candidate_.path_gen_b == candidate_b->path_gen;
    if (!same_candidate ||
        std::abs(candidate_a->path_s - candidate_.anchor_s_a) >
            progress_epsilon ||
        std::abs(candidate_b->path_s - candidate_.anchor_s_b) >
            progress_epsilon) {
        candidate_ = {};
        candidate_.valid = true;
        candidate_.vehicle_a = candidate_a->id;
        candidate_.vehicle_b = candidate_b->id;
        candidate_.path_gen_a = candidate_a->path_gen;
        candidate_.path_gen_b = candidate_b->path_gen;
        candidate_.anchor_s_a = candidate_a->path_s;
        candidate_.anchor_s_b = candidate_b->path_s;
        candidate_.duration = dt;
        std::ostringstream details;
        details << "pair=V" << candidate_a->id << "-V" << candidate_b->id
                << " path_gen=" << candidate_a->path_gen << "/"
                << candidate_b->path_gen << " s=" << candidate_a->path_s
                << "/" << candidate_b->path_s;
        emit("CANDIDATE", details.str(), emit_logs);
        return;
    }

    candidate_.duration += dt;
    if (candidate_.duration + 1e-9 < config_.deadlock_confirm_time) return;

    const DeadlockPairGeometry* geometry = observed_geometry;
    std::ostringstream confirmed;
    confirmed << "pair=V" << candidate_a->id << "-V" << candidate_b->id
              << " duration=" << candidate_.duration
              << " path_gen=" << candidate_a->path_gen << "/"
              << candidate_b->path_gen;
    emit("CONFIRMED", confirmed.str(), emit_logs);

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

    const double tolerance = std::max(0.005, config_.path_validation_step);
    // 若已经在路径起点，不再退
    if (retreat->path_s <= tolerance) 
    {
        transaction_.phase = RecoveryPhase::PASS;
        transaction_.retreat_attempt = 0;
        transaction_.retreat_target_s = 0.0;
        transaction_.retreat_clear_elapsed = 0.0;
        transaction_.pass_clear_elapsed = 0.0;
        transaction_.reason = "retreater_already_at_path_start";
        refreshDirective();
        return;
    }

    // 否则第一次退 0.5 m，不足则退到 s=0，如果后退会碰到车，证明后退失效，直接无解----后续可以考虑多车联动，目前双车先这样
    const double target_s = std::max(0.0,   retreat->path_s - config_.deadlock_retreat_distance);
    if (!retreatSweepClear(*retreat, *passer, vehicles,target_s)) 
    {
        transaction_.phase = RecoveryPhase::UNRESOLVED;
        transaction_.reason = "first_retreat_sweep_blocked";
        refreshDirective();
        emit("UNRESOLVED","pair=V" + std::to_string(retreat->id) + "-V" + std::to_string(passer->id) + " reason=first_retreat_sweep_blocked",emit_logs);
        return;
    }

    transaction_.phase = RecoveryPhase::RETREAT;
    transaction_.retreat_attempt = 1;
    transaction_.retreat_target_s = target_s;
    transaction_.retreat_distance =
    retreat->path_s - target_s;

    transaction_.reason ="priority_yielding_fixed_retreat";
    refreshDirective();

    std::ostringstream selection;
    selection << "pair=V" << candidate_a->id << "-V" << candidate_b->id
          << " priority=V" << passer->id
          << " retreat=V" << retreat->id
          << " attempt=1"
          << " start_s=" << retreat->path_s 
          << " target_s=" << target_s
          << " distance=" << transaction_.retreat_distance;

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
