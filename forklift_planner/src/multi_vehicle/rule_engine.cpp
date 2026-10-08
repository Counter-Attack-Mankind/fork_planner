#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/conflict_zone_closure.h"
#include "forklift_planner/multi_vehicle/bridge_ttc_correction.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

RuleEngine::RuleEngine(const MapParam& mp, const MultiVehicleConfig& cfg)
    : mp_(mp), cfg_(cfg),
      a1_coordinator_(
          mp, cfg,
          A1Coordinator::Dependencies{
              [this](const VehicleAgent& a, const VehicleAgent& b) {
                  return computeConflictZonesFull(a, b);
              },
              [this](const VehicleAgent& a, const VehicleAgent& b) {
                  return findConflictZones(a, b);
              },
              [this](const VehicleAgent& a, const VehicleAgent& b) {
                  return unifiedPriority(a, b);
              }}),
      deadlock_manager_(mp, cfg) {}

bool RuleEngine::shouldLogA1Decision(const VehicleAgent& vehicle,
                                     int blocker_id) {
    return a1_coordinator_.shouldLogA1Decision(vehicle, blocker_id);
}

std::string RuleEngine::debugLogPrefix() const {
    std::ostringstream out;
    out << "[SOURCE=" << debug_log_source_ << "]"
        << " [plan=" << debug_log_plan_id_ << "]"
        << " [frame=" << debug_log_frame_id_ << "]"
        << " [rollout_step=" << debug_log_rollout_step_ << "]";
    return out.str();
}

namespace {

std::vector<ConflictMarker::TimedOverlap> decimateTimedOverlaps(
    const std::vector<ConflictMarker::TimedOverlap>& input) {
    constexpr size_t kMaxDisplayedSamples = 32;
    if (input.size() <= kMaxDisplayedSamples) return input;
    std::vector<ConflictMarker::TimedOverlap> output;
    output.reserve(kMaxDisplayedSamples);
    for (size_t i = 0; i < kMaxDisplayedSamples; ++i) {
        const size_t index = i * (input.size() - 1) /
                             (kMaxDisplayedSamples - 1);
        output.push_back(input[index]);
    }
    return output;
}

struct OverlapSample {
    double s_self = 0.0;
    double s_other = 0.0;
    double x = 0.0;
    double y = 0.0;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    bool bounds_valid = false;
};

const char* missionPhaseName(MissionPhase phase) {
    switch (phase) {
        case MissionPhase::DIRECT_TO_B: return "DIRECT_TO_B";
        case MissionPhase::TO_A1: return "TO_A1";
        case MissionPhase::PICKUP_DWELL: return "PICKUP_DWELL";
        case MissionPhase::WAIT_DROPOFF_TASK: return "WAIT_DROPOFF_TASK";
        case MissionPhase::TO_B: return "TO_B";
        case MissionPhase::UNLOAD_DWELL: return "UNLOAD_DWELL";
    }
    return "UNKNOWN";
}

void logConflictReservation(
    const std::function<void(const std::string&)>& sink,
    const std::pair<int, int>& key, const char* op,
    const RuleEngine::ConflictReservation& reservation) {
    if (!sink) return;
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[CONFLICT_RESERVATION] component=PAIRWISE pair=V" << key.first
         << "-V" << key.second << " event=" << op
         << " holder=V" << reservation.owner_id
         << " waiter=V"
         << (reservation.owner_id == key.first ? key.second : key.first)
         << " conflict_type=CROSSING/OPPOSING"
         << " t=" << reservation.first_conflict_t
         << " reason=" << (reservation.create_reason.empty()
                                ? "unknown" : reservation.create_reason)
         << " zone=(" << reservation.x << "," << reservation.y << ")";
    sink(line.str());
}

bool sameReservation(const RuleEngine::ConflictReservation& a,
                     const RuleEngine::ConflictReservation& b) {
    return a.owner_id == b.owner_id &&
           a.gen_lo == b.gen_lo && a.gen_hi == b.gen_hi &&
           a.enter_lo == b.enter_lo && a.exit_lo == b.exit_lo &&
           a.enter_hi == b.enter_hi && a.exit_hi == b.exit_hi &&
           a.x == b.x && a.y == b.y &&
           a.first_conflict_t == b.first_conflict_t &&
           a.create_reason == b.create_reason &&
           a.raw_zone_index == b.raw_zone_index &&
           a.aabb_min_x == b.aabb_min_x && a.aabb_min_y == b.aabb_min_y &&
           a.aabb_max_x == b.aabb_max_x && a.aabb_max_y == b.aabb_max_y &&
           a.aabb_valid == b.aabb_valid;
}

void logA1Decision(const std::function<void(const std::string&)>& sink,
                   const MultiVehicleConfig& cfg, const VehicleAgent& vehicle,
                   const VehicleAgent* blocker, int blocker_id) {
    if (!sink || vehicle.track.empty()) return;
    const RoughWp pose = vehicle.track.poseAtS(vehicle.path_s);
    const double dist_to_a1 = std::hypot(
        pose.x - cfg.a1_pickup_center_x,
        pose.y - cfg.a1_pickup_center_y);
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[A1_DECISION] V" << vehicle.id
         << " action=" << actionName(vehicle.requested_action)
         << " reason=" << vehicle.reason
         << " blocker=V" << blocker_id
         << " phase=" << missionPhaseName(vehicle.mission_phase)
         << " dist_to_A1=" << dist_to_a1
         << " blocker_phase="
         << (blocker ? missionPhaseName(blocker->mission_phase) : "UNKNOWN");
    sink(line.str());
}

}  // namespace

RuleEngine::SimSnapshot RuleEngine::snapshot() const {
    return SimSnapshot{conflict_reservations_, a1_coordinator_.snapshot(),
                       following_pairs_, tokens_, conflicts_,
                       deadlock_manager_.snapshot(), now_};
}

void RuleEngine::restore(const SimSnapshot& s, bool restore_deadlock) {
    for (const auto& current : conflict_reservations_) {
        if (s.reservations.count(current.first) == 0) {
            logConflictReservation(coord_log_sink_, current.first, "delete",
                                   current.second);
        }
    }
    for (const auto& incoming : s.reservations) {
        const auto current = conflict_reservations_.find(incoming.first);
        if (current == conflict_reservations_.end()) {
            logConflictReservation(coord_log_sink_, incoming.first, "create",
                                   incoming.second);
        } else if (!sameReservation(current->second, incoming.second)) {
            logConflictReservation(coord_log_sink_, incoming.first, "update",
                                   incoming.second);
        }
    }
    conflict_reservations_ = s.reservations;
    a1_coordinator_.restore(s.a1, restore_deadlock);
    following_pairs_ = s.following_pairs;
    tokens_ = s.tokens;
    conflicts_ = s.conflicts;
    if (restore_deadlock) deadlock_manager_.restore(s.deadlock);
    now_ = s.now;
}

double RuleEngine::speedForAction(VehicleAction action) const {
    switch (action) {
        case VehicleAction::STOP:
            return 0.0;
        case VehicleAction::CREEP:
            return cfg_.nominal_speed * cfg_.creep_ratio;
        case VehicleAction::YIELD:
            return cfg_.nominal_speed * cfg_.yield_ratio;
        case VehicleAction::NOMINAL:
            return cfg_.nominal_speed;
        case VehicleAction::BOOST:
            return cfg_.enable_boost
                       ? std::min(cfg_.max_speed,
                                  cfg_.nominal_speed * cfg_.boost_ratio)
                       : cfg_.nominal_speed;
    }
    return 0.0;
}

int RuleEngine::priorityWinner(const VehicleAgent& a,
                               const VehicleAgent& b) const {
    if (!cfg_.enable_priority_tiebreak) return -1;

    // 资源前置约束(非任意优先级,§4/§6/§7):若一车要去的目标库位正被另一车占着
    // (a.target==b.current),占用者必须先清出该位、入库者让行——否则入库者抢先开到
    // 库位口却进不去(位被占),占用者又被它让停在口内,直接死锁。这是 slot 资源依赖,
    // 当任务只指向空/即将空的库位时无环;故置于严格全序之上作 override(若与全序冲突成环
    // =任务分配层的循环占位问题,由第5步环检测断言抓出)。
    {
        const bool a_wants_b_slot = (a.target_slot == b.current_slot);
        const bool b_wants_a_slot = (b.target_slot == a.current_slot);
        if (a_wants_b_slot && !b_wants_a_slot) return b.id;  // b 占用者,先清出
        if (b_wants_a_slot && !a_wants_b_slot) return a.id;  // a 占用者,先清出
    }

    // The existing continuous A1 service commitment is the second-level
    // ordinary-road authority. Its creation, lock and release remain wholly
    // owned by A1Coordinator; pair priority only consumes the current owner.
    const FutureA1Commitment& a1 = a1_coordinator_.futureA1Commitment();
    if (a1.valid()) {
        const bool a_is_owner = a.id == a1.owner_id;
        const bool b_is_owner = b.id == a1.owner_id;
        if (a_is_owner != b_is_owner) return a_is_owner ? a.id : b.id;
    }

    // Ordinary-road authority is the existing deterministic total order.
    // Local interaction geometry (including same-direction front/rear order)
    // is not a second priority system.
    return unifiedPriority(a, b);
}

int RuleEngine::unifiedPriority(const VehicleAgent& a,
                                const VehicleAgent& b) const {
    // 协调图第2步:严格全序 ≺。按可证「反对称+传递+完全」的字典序键比较,id 作唯一终裁。
    // 任意时刻"谁让谁"关系由全序导出 ⇒ 必无环 ⇒ 无死锁(见 草履虫规则_协调图统一架构
    // 设计 §2.3/§5)。
    // 普通道路基础键(越小越优先 / 越先走):
    //   1) 载货优先;2) 已完成任务少者优先(摊平工作量);3) id 最小(确定性终裁)。
    // wait_time 仍用于日志、统计和死锁/饥饿诊断，但不得让本次
    // YIELD/CREEP/STOP 自己积累出的等待时间反向翻转下一滚动周期的优先级。
    // 注:原"占用者先清出"的成对 slot 规则会破坏传递性(成对、非全序),已移出本函数,
    //     在 priorityWinner 中作显式**资源前置约束** override(见那里)。
    auto key = [&](const VehicleAgent& v) {
        return std::make_tuple(v.loaded ? 0 : 1, // 载货优先
                               v.task_count,     // 任务少优先
                               v.id);            // 唯一终裁
    };
    return key(a) < key(b) ? a.id : b.id;
}

double RuleEngine::predictedTravelDistance(const VehicleAgent& v,
                                           VehicleAction action,
                                           double t) const {
    if (t <= 0.0) return 0.0;

    const double desired = speedForAction(action);
    const double current = std::max(0.0, v.current_speed);
    if (std::abs(desired - current) < 1e-9) {
        return current * t;
    }

    const bool accelerating = desired > current;
    const double limit = accelerating ? cfg_.max_accel : cfg_.max_decel;
    if (limit <= 1e-9) {
        return current * t;
    }

    const double signed_accel = accelerating ? limit : -limit;
    const double ramp_time = std::abs(desired - current) / limit;
    const double ramp_used = std::min(t, ramp_time);
    const double ramp_dist =
        current * ramp_used + 0.5 * signed_accel * ramp_used * ramp_used;
    if (t <= ramp_time) {
        return std::max(0.0, ramp_dist);
    }
    return std::max(0.0, ramp_dist + desired * (t - ramp_time));
}


std::vector<RuleEngine::ConflictZone> RuleEngine::computeConflictZonesFull(
    const VehicleAgent& self, const VehicleAgent& other) const {
    constexpr double kStep = 0.025;
    constexpr double kMergeGap = kStep * 2.25;
    std::vector<ConflictZone> zones;

    // 两条路径都是「完全固定、已知」的;它们之间的冲突集 C_ij 是静态几何量,与时间/
    // 速度/朝向/当前位置都无关 → 在整段路径 [0,L]×[0,L] 上一次算定即可(由
    // conflictBlocksCanonical 按 path_gen 缓存)。当前位置相关的裁剪(裁掉已清出的块、
    // 把入口夹到车尾起点)在 findConflictZones 里按调用时的 path_s 施加。
    // (历史教训:早先按 current_speed×prediction_horizon 裁剪扫描范围,会在让行车停在
    // 停止线 speed→0 时让冲突区凭空消失 → 原子门预约被释放 → 让行车蹭过停止线挤进区
    // → 三车旋转楔死。改扫整段固定路径根治;此处进一步缓存,几何恒定可见且不每拍重算。)
    const double s_self_end = self.track.length();
    const double s_other_end = other.track.length();
    const double s_self_begin = 0.0;
    const double s_other_begin = 0.0;

    // 广相剪枝(扫全程后的性能护栏):两车「剩余路径」的轴对齐包围盒各按「车身对角线
    // 半径 + 冲突余量」外胀;若两盒分离,则任意位姿下车身 OBB 绝无可能重叠 → 直接返回空,
    // 免去对八竿子打不着的远车做整段精扫。这只省算力,绝不改变任何冲突判定结果。
    {
        const double inf = std::numeric_limits<double>::infinity();
        const double infl =
            0.5 * std::hypot(mp_.vehicle_length, mp_.vehicle_width) +
            cfg_.conflict_margin;
        constexpr double kCoarse = 0.15;  // 粗采样建包围盒
        double ax0 = inf, ay0 = inf, ax1 = -inf, ay1 = -inf;
        for (double s = s_self_begin; s <= s_self_end + 1e-9; s += kCoarse) {
            const RoughWp p = self.track.poseAtS(std::min(s, s_self_end));
            ax0 = std::min(ax0, p.x); ay0 = std::min(ay0, p.y);
            ax1 = std::max(ax1, p.x); ay1 = std::max(ay1, p.y);
        }
        double bx0 = inf, by0 = inf, bx1 = -inf, by1 = -inf;
        for (double s = s_other_begin; s <= s_other_end + 1e-9; s += kCoarse) {
            const RoughWp p = other.track.poseAtS(std::min(s, s_other_end));
            bx0 = std::min(bx0, p.x); by0 = std::min(by0, p.y);
            bx1 = std::max(bx1, p.x); by1 = std::max(by1, p.y);
        }
        if (ax1 + infl < bx0 - infl || bx1 + infl < ax0 - infl ||
            ay1 + infl < by0 - infl || by1 + infl < ay0 - infl) {
            return zones;  // 包围盒分离 → 无冲突
        }
    }

    for (double ss = s_self_begin; ss <= s_self_end + 1e-9; ss += kStep) {
        const double ss_clamped = std::min(ss, s_self_end);
        // Static ConflictZone uses the same bare-body geometry as the timed
        // prediction layer and the simulation hard collision guard.
        const OBB obb_s = makeBody(
            self.track.poseAtS(ss_clamped), mp_, 0.0);
        std::vector<OverlapSample> row;

        for (double so = s_other_begin; so <= s_other_end + 1e-9; so += kStep) {
            const double so_clamped = std::min(so, s_other_end);
            const OBB obb_o = makeBody(
                other.track.poseAtS(so_clamped), mp_, 0.0);

            if (!overlaps(obb_s, obb_o)) continue;

            const RoughWp ps = self.track.poseAtS(ss_clamped);
            const RoughWp po = other.track.poseAtS(so_clamped);
            OverlapSample sample;
            sample.s_self = ss_clamped;
            sample.s_other = so_clamped;
            sample.x = 0.5 * (ps.x + po.x);
            sample.y = 0.5 * (ps.y + po.y);
            const auto polygon = intersectObbs(obb_s, obb_o);
            if (polygon.size() >= 3) {
                sample.min_x = sample.max_x = polygon.front().x;
                sample.min_y = sample.max_y = polygon.front().y;
                for (const auto& point : polygon) {
                    sample.min_x = std::min(sample.min_x, point.x);
                    sample.min_y = std::min(sample.min_y, point.y);
                    sample.max_x = std::max(sample.max_x, point.x);
                    sample.max_y = std::max(sample.max_y, point.y);
                }
                sample.bounds_valid = true;
            }
            row.push_back(sample);
        }

        if (row.empty()) continue;

        std::vector<ConflictZone> row_zones;
        for (const OverlapSample& sample : row) {
            if (row_zones.empty() ||
                sample.s_other > row_zones.back().s_other_exit + kMergeGap) {
                ConflictZone z;
                z.s_self_enter = sample.s_self;
                z.s_self_exit = sample.s_self;
                z.s_other_enter = sample.s_other;
                z.s_other_exit = sample.s_other;
                z.x = sample.x;
                z.y = sample.y;
                z.aabb_min_x = sample.min_x;
                z.aabb_min_y = sample.min_y;
                z.aabb_max_x = sample.max_x;
                z.aabb_max_y = sample.max_y;
                z.aabb_valid = sample.bounds_valid;
                row_zones.push_back(z);
            } else {
                ConflictZone& z = row_zones.back();
                z.s_other_exit = sample.s_other;
                z.x = 0.5 * (z.x + sample.x);
                z.y = 0.5 * (z.y + sample.y);
                if (sample.bounds_valid) {
                    if (!z.aabb_valid) {
                        z.aabb_min_x = sample.min_x;
                        z.aabb_min_y = sample.min_y;
                        z.aabb_max_x = sample.max_x;
                        z.aabb_max_y = sample.max_y;
                    } else {
                        z.aabb_min_x = std::min(z.aabb_min_x, sample.min_x);
                        z.aabb_min_y = std::min(z.aabb_min_y, sample.min_y);
                        z.aabb_max_x = std::max(z.aabb_max_x, sample.max_x);
                        z.aabb_max_y = std::max(z.aabb_max_y, sample.max_y);
                    }
                    z.aabb_valid = true;
                }
            }
        }

        const auto touches = [kMergeGap](const ConflictZone& lhs,
                                         const ConflictZone& rhs) {
            const bool self_touch =
                lhs.s_self_enter <= rhs.s_self_exit + kMergeGap &&
                lhs.s_self_exit + kMergeGap >= rhs.s_self_enter;
            const bool other_touch =
                lhs.s_other_enter <= rhs.s_other_exit + kMergeGap &&
                lhs.s_other_exit + kMergeGap >= rhs.s_other_enter;
            return self_touch && other_touch;
        };
        const auto merge = [](ConflictZone& destination,
                              const ConflictZone& source) {
            destination.s_self_enter =
                std::min(destination.s_self_enter, source.s_self_enter);
            destination.s_self_exit =
                std::max(destination.s_self_exit, source.s_self_exit);
            destination.s_other_enter =
                std::min(destination.s_other_enter, source.s_other_enter);
            destination.s_other_exit =
                std::max(destination.s_other_exit, source.s_other_exit);
            destination.x = 0.5 * (destination.x + source.x);
            destination.y = 0.5 * (destination.y + source.y);
            if (source.aabb_valid) {
                if (!destination.aabb_valid) {
                    destination.aabb_min_x = source.aabb_min_x;
                    destination.aabb_min_y = source.aabb_min_y;
                    destination.aabb_max_x = source.aabb_max_x;
                    destination.aabb_max_y = source.aabb_max_y;
                } else {
                    destination.aabb_min_x =
                        std::min(destination.aabb_min_x, source.aabb_min_x);
                    destination.aabb_min_y =
                        std::min(destination.aabb_min_y, source.aabb_min_y);
                    destination.aabb_max_x =
                        std::max(destination.aabb_max_x, source.aabb_max_x);
                    destination.aabb_max_y =
                        std::max(destination.aabb_max_y, source.aabb_max_y);
                }
                destination.aabb_valid = true;
            }
        };

        for (const ConflictZone& row_zone : row_zones) {
            insertConflictZoneWithClosure(row_zone, zones, touches, merge);
        }
    }

    // 为每个块算「同向」标志(静态):在块中点测两路径行进朝向(REVERSE 段切线+π),
    // 同向(dot>0.7)=正对角带=同车道跟车;否则交叉/对向。供 resolveFollowing 与 pairwise
    // 共用作稳定判据——不再用随当前位姿闪烁的瞬时朝向(交叉/汇入处会瞬时对齐而误判)。
    constexpr double kPi = 3.14159265358979323846;
    auto pathHeadingAtS = [&](const VehicleAgent& v, double s) {
        double h = v.track.poseAtS(s).theta;
        if (v.track.typeAtS(s) == WpType::REVERSE) h += kPi;
        return h;
    };
    for (size_t raw_index = 0; raw_index < zones.size(); ++raw_index) {
        ConflictZone& z = zones[raw_index];
        z.raw_index = static_cast<int>(raw_index);
        const double hs =
            pathHeadingAtS(self, 0.5 * (z.s_self_enter + z.s_self_exit));
        const double ho =
            pathHeadingAtS(other, 0.5 * (z.s_other_enter + z.s_other_exit));
        z.same_dir =
            (std::cos(hs) * std::cos(ho) + std::sin(hs) * std::sin(ho)) > 0.7;
    }

    return zones;
}

const std::vector<RuleEngine::ConflictZone>& RuleEngine::conflictBlocksCanonical(
    const VehicleAgent& lo, const VehicleAgent& hi) const {
    // lo.id < hi.id(调用方保证)。按 path_gen 缓存:任一方换了固定路径才重算。
    const std::pair<int, int> key{lo.id, hi.id};
    ConflictCacheEntry& e = conflict_cache_[key];
    if (e.gen_lo != lo.path_gen || e.gen_hi != hi.path_gen) {
        e.blocks = computeConflictZonesFull(lo, hi);  // self=lo, other=hi
        e.display_overlap_polygons.clear();
        e.gen_lo = lo.path_gen;
        e.gen_hi = hi.path_gen;
    }
    return e.blocks;
}

std::vector<RuleEngine::ConflictZone> RuleEngine::findConflictZones(
    const VehicleAgent& self, const VehicleAgent& other) const {
    // 取静态 C_ij(缓存),按 self/other 朝向取用,再按当前位置裁剪——产物与历史
    // 「逐拍沿剩余路径全程精扫」在同一离散精度下等价(窗口只截低端、不会拆分连通块)。
    const bool self_is_lo = self.id < other.id;
    const VehicleAgent& lo = self_is_lo ? self : other;
    const VehicleAgent& hi = self_is_lo ? other : self;
    const std::vector<ConflictZone>& canon = conflictBlocksCanonical(lo, hi);

    // 车尾参考:车身向后伸 rear_ext。某车「已完全清出某块」= 车尾(s-rear_ext)越过该块
    // 在其路径上的出口 → 该块对它不再是冲突,丢弃。
    // 注:块的入口/出口保持「静态」(不再夹到 path_s-rear_ext)——夹紧会让上报的 se 随车
    // 前移,导致让行方的停止线(se-front)随它一起漂、永远追不上、最终蹭进区(实测 V1↔V5
    // 蹭撞的一半根因)。停止线必须是固定弧长,让行方才能稳稳停在区外。committed/cleared 仍按
    // 静态 se/sx 与当前 path_s 比较,语义不变。
    const double rear_ext = mp_.body_rear_ext();
    const double self_begin = std::max(0.0, self.path_s - rear_ext);
    const double other_begin = std::max(0.0, other.path_s - rear_ext);

    std::vector<ConflictZone> out;
    out.reserve(canon.size());
    for (const ConflictZone& cz : canon) {
        ConflictZone z = cz;
        if (!self_is_lo) {  // 朝向反转:canonical 以 lo 为 self,这里 self 是 hi
            std::swap(z.s_self_enter, z.s_other_enter);
            std::swap(z.s_self_exit, z.s_other_exit);
        }
        // 任一方已完全清出该块 → 不再冲突,丢弃。(入口不夹紧,保持静态)
        if (z.s_self_exit < self_begin || z.s_other_exit < other_begin) continue;
        out.push_back(z);
    }
    return out;
}

std::vector<ConflictMarker> RuleEngine::conflictResourceMarkers(
    const std::vector<VehicleAgent>& vehicles) const {
    std::vector<ConflictMarker> markers;
    if (!cfg_.show_prediction_conflicts) return markers;

    constexpr double kDisplayStep = 0.025;
    constexpr size_t kMaxPolygonsPerZone = 256;
    auto buildPolygons = [&](const VehicleAgent& self,
                             const VehicleAgent& other,
                             const ConflictZone& zone) {
        // ConflictZone stores compressed arc intervals and the diagnostic
        // union AABB, but not every source polygon. Re-run the same OBB
        // predicate inside the arc rectangle for the blue display geometry;
        // non-overlapping combinations inside the compressed rectangle are
        // explicitly discarded and never rendered.
        std::map<std::pair<long long, long long>,
                 std::vector<ConflictMarker::Point>> spatial_cells;
        for (double ss = zone.s_self_enter;
             ss <= zone.s_self_exit + 1e-9; ss += kDisplayStep) {
            const double self_s = std::min(ss, zone.s_self_exit);
            const OBB self_body = makeBody(
                self.track.poseAtS(self_s), mp_, 0.0);
            for (double so = zone.s_other_enter;
                 so <= zone.s_other_exit + 1e-9; so += kDisplayStep) {
                const double other_s = std::min(so, zone.s_other_exit);
                const OBB other_body = makeBody(
                    other.track.poseAtS(other_s), mp_, 0.0);
                if (!overlaps(self_body, other_body)) continue;
                auto polygon = intersectObbs(self_body, other_body);
                if (polygon.size() < 3) continue;
                double cx = 0.0;
                double cy = 0.0;
                for (const auto& p : polygon) {
                    cx += p.x;
                    cy += p.y;
                }
                cx /= static_cast<double>(polygon.size());
                cy /= static_cast<double>(polygon.size());
                const std::pair<long long, long long> cell{
                    static_cast<long long>(std::llround(cx / kDisplayStep)),
                    static_cast<long long>(std::llround(cy / kDisplayStep))};
                spatial_cells.emplace(cell, std::move(polygon));
            }
        }

        std::vector<std::vector<ConflictMarker::Point>> polygons;
        if (spatial_cells.empty()) return polygons;
        const size_t stride = std::max<size_t>(
            1, (spatial_cells.size() + kMaxPolygonsPerZone - 1) /
                   kMaxPolygonsPerZone);
        polygons.reserve(std::min(spatial_cells.size(),
                                  kMaxPolygonsPerZone));
        size_t index = 0;
        for (auto& item : spatial_cells) {
            if (index % stride == 0 &&
                polygons.size() < kMaxPolygonsPerZone) {
                polygons.push_back(std::move(item.second));
            }
            ++index;
        }
        return polygons;
    };

    auto polygonsForZone = [&](const VehicleAgent& self,
                               const VehicleAgent& other,
                               const ConflictZone& oriented_zone)
        -> const std::vector<std::vector<ConflictMarker::Point>>& {
        const bool self_is_lo = self.id < other.id;
        const VehicleAgent& lo = self_is_lo ? self : other;
        const VehicleAgent& hi = self_is_lo ? other : self;
        const auto& canonical = conflictBlocksCanonical(lo, hi);
        ConflictCacheEntry& cache = conflict_cache_[{lo.id, hi.id}];
        if (cache.display_overlap_polygons.size() != canonical.size()) {
            cache.display_overlap_polygons.clear();
            cache.display_overlap_polygons.reserve(canonical.size());
            for (const ConflictZone& zone : canonical) {
                cache.display_overlap_polygons.push_back(
                    buildPolygons(lo, hi, zone));
            }
        }

        const double self_enter = self_is_lo
            ? oriented_zone.s_self_enter : oriented_zone.s_other_enter;
        const double self_exit = self_is_lo
            ? oriented_zone.s_self_exit : oriented_zone.s_other_exit;
        const double other_enter = self_is_lo
            ? oriented_zone.s_other_enter : oriented_zone.s_self_enter;
        const double other_exit = self_is_lo
            ? oriented_zone.s_other_exit : oriented_zone.s_self_exit;
        for (size_t index = 0; index < canonical.size(); ++index) {
            const ConflictZone& zone = canonical[index];
            if (std::abs(zone.s_self_enter - self_enter) <= 1e-9 &&
                std::abs(zone.s_self_exit - self_exit) <= 1e-9 &&
                std::abs(zone.s_other_enter - other_enter) <= 1e-9 &&
                std::abs(zone.s_other_exit - other_exit) <= 1e-9) {
                return cache.display_overlap_polygons[index];
            }
        }
        static const std::vector<std::vector<ConflictMarker::Point>> empty;
        return empty;
    };

    auto makeMarker = [&](const VehicleAgent& a, const VehicleAgent& b,
                          const ConflictZone& zone, int zone_index,
                          ConflictMarkerKind kind, int holder_id,
                          int waiter_id) {
        ConflictMarker marker;
        marker.vehicle_a = a.id;
        marker.vehicle_b = b.id;
        marker.raw_zone_index = zone.raw_index;
        marker.active_zone_index = zone_index;
        marker.path_gen_a = a.path_gen;
        marker.path_gen_b = b.path_gen;
        marker.s_a_enter = zone.s_self_enter;
        marker.s_a_exit = zone.s_self_exit;
        marker.s_b_enter = zone.s_other_enter;
        marker.s_b_exit = zone.s_other_exit;
        marker.kind = kind;
        marker.holder_id = holder_id;
        marker.waiter_id = waiter_id;
        marker.spatial_overlap_polygons = polygonsForZone(a, b, zone);

        if (zone.aabb_valid) {
            marker.x = 0.5 * (zone.aabb_min_x + zone.aabb_max_x);
            marker.y = 0.5 * (zone.aabb_min_y + zone.aabb_max_y);
            marker.scale_x = std::max(0.01, zone.aabb_max_x - zone.aabb_min_x);
            marker.scale_y = std::max(0.01, zone.aabb_max_y - zone.aabb_min_y);
            marker.zone_aabb_valid = true;
        } else {
            marker.x = zone.x;
            marker.y = zone.y;
        }
        return marker;
    };

    for (size_t i = 0; i < vehicles.size(); ++i) {
        const VehicleAgent& a = vehicles[i];
        if (!a.active() || a.track.empty()) continue;
        for (size_t j = i + 1; j < vehicles.size(); ++j) {
            const VehicleAgent& b = vehicles[j];
            if (!b.active() || b.track.empty()) continue;
            const std::vector<ConflictZone> zones = findConflictZones(a, b);
            for (size_t zone_index = 0; zone_index < zones.size();
                 ++zone_index) {
                markers.push_back(makeMarker(
                    a, b, zones[zone_index],
                    static_cast<int>(zone_index),
                    ConflictMarkerKind::POTENTIAL_CONFLICT_ZONE, -1, -1));
            }

        }
    }
    return markers;
}

void RuleEngine::recordConflictZones(
    const VehicleAgent& self, const VehicleAgent& other,
    const std::vector<ConflictZone>& zones, ConflictMarkerKind kind,
    double first_conflict_t, int follower_id, int leader_id,
    double following_gap, VehicleAction following_action,
    int holder_id, int waiter_id,
    const std::vector<ConflictMarker::TimedOverlap>& timed_overlaps,
    PairInteractionType interaction_type, double last_conflict_t) {
    constexpr double kDisplayStep = 0.025;
    const double following_pad =
        std::max(0.03, 0.5 * mp_.vehicle_width +
                           0.5 * cfg_.conflict_margin);

    for (const ConflictZone& z : zones) {
        double x_min = std::numeric_limits<double>::infinity();
        double y_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();

        auto includePathSpan = [&](const VehicleAgent& v,
                                   double s_enter, double s_exit) {
            const double begin = std::max(0.0, s_enter);
            const double end = std::min(v.track.length(), s_exit);
            if (end < begin) return;
            for (double s = begin; s <= end + 1e-9;
                 s += kDisplayStep) {
                const RoughWp p = v.track.poseAtS(std::min(s, end));
                x_min = std::min(x_min, p.x);
                y_min = std::min(y_min, p.y);
                x_max = std::max(x_max, p.x);
                y_max = std::max(y_max, p.y);
            }
            const RoughWp p = v.track.poseAtS(end);
            x_min = std::min(x_min, p.x);
            y_min = std::min(y_min, p.y);
            x_max = std::max(x_max, p.x);
            y_max = std::max(y_max, p.y);
        };

        if (interaction_type == PairInteractionType::OPPOSING &&
            z.aabb_valid) {
            x_min = z.aabb_min_x;
            y_min = z.aabb_min_y;
            x_max = z.aabb_max_x;
            y_max = z.aabb_max_y;
        } else if (!timed_overlaps.empty()) {
            for (const ConflictMarker::TimedOverlap& overlap : timed_overlaps) {
                for (const ConflictMarker::Point& p : overlap.polygon) {
                    x_min = std::min(x_min, p.x);
                    y_min = std::min(y_min, p.y);
                    x_max = std::max(x_max, p.x);
                    y_max = std::max(y_max, p.y);
                }
            }
        } else if (kind == ConflictMarkerKind::SAME_DIRECTION) {
            includePathSpan(self, z.s_self_enter, z.s_self_exit);
            includePathSpan(other, z.s_other_enter, z.s_other_exit);
        }
        if (!std::isfinite(x_min) || !std::isfinite(y_min) ||
            !std::isfinite(x_max) || !std::isfinite(y_max)) {
            continue;
        }

        ConflictMarker marker;
        marker.x = 0.5 * (x_min + x_max);
        marker.y = 0.5 * (y_min + y_max);
        const double pad = kind == ConflictMarkerKind::SAME_DIRECTION
            ? following_pad : 0.0;
        marker.scale_x = std::max(0.01, x_max - x_min + 2.0 * pad);
        marker.scale_y = std::max(0.01, y_max - y_min + 2.0 * pad);
        marker.vehicle_a = self.id;
        marker.vehicle_b = other.id;
        marker.path_gen_a = self.path_gen;
        marker.path_gen_b = other.path_gen;
        marker.raw_zone_index = z.raw_index;
        marker.s_a_enter = z.s_self_enter;
        marker.s_a_exit = z.s_self_exit;
        marker.s_b_enter = z.s_other_enter;
        marker.s_b_exit = z.s_other_exit;
        if (z.raw_index >= 0) {
            const auto active_zones = findConflictZones(self, other);
            for (size_t active = 0; active < active_zones.size(); ++active) {
                if (active_zones[active].raw_index == z.raw_index) {
                    marker.active_zone_index = static_cast<int>(active);
                    break;
                }
            }
        }
        marker.follower_id = follower_id;
        marker.leader_id = leader_id;
        marker.holder_id = holder_id;
        marker.waiter_id = waiter_id;
        marker.following_gap = following_gap;
        marker.following_action = following_action;
        marker.interaction_type = interaction_type;
        marker.timed_overlaps = timed_overlaps;
        if (follower_id >= 0 && leader_id >= 0) {
            const RoughWp follower_pose = self.track.poseAtS(self.path_s);
            const RoughWp leader_pose = other.track.poseAtS(other.path_s);
            marker.follower_x = follower_pose.x;
            marker.follower_y = follower_pose.y;
            marker.leader_x = leader_pose.x;
            marker.leader_y = leader_pose.y;
        }
        marker.kind = kind;
        marker.t = first_conflict_t;
        marker.last_t = last_conflict_t >= 0.0
            ? last_conflict_t : first_conflict_t;
        conflicts_.push_back(marker);
    }
}

void RuleEngine::debugDumpConflict(const VehicleAgent& a,
                                   const VehicleAgent& b) const {
    const std::vector<ConflictZone> zones = findConflictZones(a, b);
    const std::pair<int, int> pkey{std::min(a.id, b.id), std::max(a.id, b.id)};
    int owner = -1;
    auto it = conflict_reservations_.find(pkey);
    if (it != conflict_reservations_.end()) owner = it->second.owner_id;
    const bool following = following_pairs_.count(pkey) > 0;
    const double f = mp_.body_front_ext();
    bool all_same = true;
    for (const ConflictZone& z : zones)
        if (!z.same_dir) { all_same = false; break; }
    ROS_WARN("%s [CONFLICT] V%d(s=%.3f rem=%.3f act=%d blk=%d) vs "
             "V%d(s=%.3f rem=%.3f act=%d blk=%d) | owner=V%d following=%d "
             "nzones=%zu all_same_dir=%d",
             debugLogPrefix().c_str(),
             a.id, a.path_s, a.remainingS(), (int)a.action, a.blocker_id,
             b.id, b.path_s, b.remainingS(), (int)b.action, b.blocker_id,
             owner, (int)following, zones.size(), (int)all_same);
    for (size_t i = 0; i < zones.size(); ++i) {
        const ConflictZone& z = zones[i];
        ROS_WARN("%s [CONFLICT]   zone%zu same_dir=%d | A[%.3f,%.3f] stopA=%.3f "
                 "committedA=%d | B[%.3f,%.3f] stopB=%.3f committedB=%d | @(%.2f,%.2f)",
                 debugLogPrefix().c_str(), i, (int)z.same_dir,
                 z.s_self_enter, z.s_self_exit,
                 z.s_self_enter - f, (int)(a.path_s > z.s_self_enter),
                 z.s_other_enter, z.s_other_exit, z.s_other_enter - f,
                 (int)(b.path_s > z.s_other_enter), z.x, z.y);
    }
}


double RuleEngine::timeToReachS(const VehicleAgent& v, VehicleAction action,
                                 double target_s) const {
    if (target_s <= v.path_s + 1e-9) return 0.0;

    const double dist    = target_s - v.path_s;
    const double v_des   = speedForAction(action);
    const double v_cur   = std::max(0.0, v.current_speed);

    if (v_des < 1e-9) return std::numeric_limits<double>::infinity();

    // Binary search: invert predictedTravelDistance
    double lo = 0.0;
    double hi = dist / v_des * 3.0 + 2.0;
    for (int i = 0; i < 64; ++i) {
        const double mid = (lo + hi) * 0.5;
        (predictedTravelDistance(v, action, mid) < dist ? lo : hi) = mid;
    }
    return hi;
}

void RuleEngine::applyActionRequest(VehicleAgent& v, VehicleAction action,
                                    const std::string& reason,
                                    int blocker_id) {
    if (moreRestrictive(action, v.requested_action)) {
        v.requested_action = action;
        v.reason = reason;
        v.blocker_id = blocker_id;
    }
}

void RuleEngine::refreshDepartureClusterCommitments(
    std::vector<VehicleAgent>& vehicles) {
    a1_coordinator_.refreshDepartureClusterCommitments(vehicles);
}

int RuleEngine::departureClusterOwnerForPair(const VehicleAgent& a,
                                             const VehicleAgent& b) const {
    return a1_coordinator_.authorityForPair(a, b).departure_owner_id;
}

RuleEngine::A1LaunchAdmission RuleEngine::checkA1LaunchAdmission(
    const VehicleAgent& service_owner,
    const VehicleAgent& launch_candidate) const {
    return a1_coordinator_.checkA1LaunchAdmission(service_owner,
                                                   launch_candidate);
}

RuleEngine::SlotDepartureAdmission RuleEngine::checkSlotDepartureAdmission(
    const VehicleAgent* service_owner,
    const VehicleAgent& launch_candidate,
    const std::vector<VehicleAgent>& vehicles,
    double prediction_horizon) const {
    SlotDepartureAdmission result;
    if (!launch_candidate.active() || launch_candidate.track.empty() ||
        launch_candidate.mission_phase != MissionPhase::TO_A1) {
        return result;
    }

    if (service_owner != nullptr &&
        service_owner->id != launch_candidate.id) {
        result.a1 = checkA1LaunchAdmission(*service_owner,
                                           launch_candidate);
        if (result.a1.departure_resource_conflict) {
            result.clear = false;
            result.a1_departure_conflict = true;
            result.blocker_id = service_owner->id;
        }
    }

    const double horizon = std::max(cfg_.prediction_step,
                                    prediction_horizon);
    const auto candidate_prediction = predictTrajectory(
        launch_candidate, mp_, cfg_, VehicleAction::NOMINAL, horizon);
    if (candidate_prediction.empty()) return result;

    auto sampleAt = [](const auto& prediction, double time)
        -> const PredictedKinematicSample& {
        auto it = std::lower_bound(
            prediction.begin(), prediction.end(), time,
            [](const PredictedKinematicSample& sample, double value) {
                return sample.t < value;
            });
        return it == prediction.end() ? prediction.back() : *it;
    };
    constexpr double kPi = 3.14159265358979323846;
    auto motionHeading = [&](const VehicleAgent& vehicle, double s) {
        double heading = vehicle.track.poseAtS(s).theta;
        if (vehicle.track.typeAtS(s) == WpType::REVERSE) heading += kPi;
        return heading;
    };

    for (const VehicleAgent& occupant : vehicles) {
        if (occupant.id == launch_candidate.id || !occupant.active() ||
            occupant.track.empty()) {
            continue;
        }
        const auto occupant_prediction = predictTrajectory(
            occupant, mp_, cfg_, VehicleAction::NOMINAL, horizon);
        const PairInteractionResult physical =
            detectPairInteractionFromPredictions(
                launch_candidate, occupant, {}, candidate_prediction,
                occupant_prediction);
        if (!physical.event.valid) continue;

        const PredictedKinematicSample& candidate_sample = sampleAt(
            candidate_prediction, physical.event.first_overlap_t);
        if (candidate_sample.s >
            launch_candidate.slot_departure_clear_s + 1e-9) {
            continue;
        }
        if (result.ordinary_road_conflict &&
            physical.event.first_overlap_t >= result.first_conflict_t - 1e-9) {
            continue;
        }

        const PredictedKinematicSample& occupant_sample = sampleAt(
            occupant_prediction, physical.event.first_overlap_t);
        const double direction_dot = std::cos(
            motionHeading(launch_candidate, candidate_sample.s) -
            motionHeading(occupant, occupant_sample.s));
        result.interaction_type = direction_dot < -0.5
            ? PairInteractionType::OPPOSING
            : direction_dot > 0.7
                ? PairInteractionType::SAME_DIRECTION
                : PairInteractionType::CROSSING;
        result.clear = false;
        result.ordinary_road_conflict = true;
        result.blocker_id = occupant.id;
        result.first_conflict_t = physical.event.first_overlap_t;
        result.candidate_conflict_s = candidate_sample.s;
    }
    return result;
}

void RuleEngine::enforceDepartureClusterCommitments(
    std::vector<VehicleAgent>& vehicles, double dt) {
    a1_coordinator_.enforceDepartureClusterCommitments(
        vehicles, dt,
        [this](VehicleAgent& vehicle, VehicleAction action,
               const std::string& reason, int blocker_id) {
            if (a1_coordinator_.intrusionCorrectionFor(vehicle.id) != nullptr &&
                !motionOverrideFor(vehicle.id).a1_intrusion) {
                return;
            }
            applyActionRequest(vehicle, action, reason, blocker_id);
        });
}

PairInteractionResult RuleEngine::detectPairInteraction(
    const VehicleAgent& a, const VehicleAgent& b,
    double prediction_horizon) const {
    const std::vector<ConflictZone> zones = findConflictZones(a, b);
    const auto prediction_a =
        predictTrajectory(a, mp_, cfg_, VehicleAction::NOMINAL,
                          prediction_horizon);
    const auto prediction_b =
        predictTrajectory(b, mp_, cfg_, VehicleAction::NOMINAL,
                          prediction_horizon);
    PairInteractionResult result = detectPairInteractionFromPredictions(
        a, b, {}, prediction_a, prediction_b);
    // Static path geometry remains diagnostic only for this public query; it
    // is no longer a gate or identity source for a crossing TimedConflict.
    result.potential_zones = zones;
    return result;
}

void RuleEngine::resolvePairwiseConflicts(std::vector<VehicleAgent>& vehicles,
                                          double dt,
                                          double prediction_horizon,
                                          bool reuse_ordinary_coordination) {
    pairwise_managed_pairs_.clear();
    ordinary_dynamic_pairs_.clear();
    following_pairs_.clear();
    // Predict each vehicle once. Pair detection below is pure and consumes
    // these shared samples without changing coordination state.
    const double horizon =
        std::max(cfg_.prediction_step, prediction_horizon);
    const double prediction_step = std::max(0.02, cfg_.prediction_step);
    std::vector<std::vector<PredictedKinematicSample>> predictions(
        vehicles.size());
    for (size_t i = 0; i < vehicles.size(); ++i) {
        if (vehicles[i].active() && !vehicles[i].track.empty()) {
            const RecoveryMotion recovery_motion =
                deadlock_manager_.directive().motionFor(vehicles[i].id);
            const VehicleAction prediction_action =
                recovery_motion == RecoveryMotion::HOLD ||
                        vehicles[i].ttc_stop_hold_remaining > 1e-9
                    ? VehicleAction::STOP : VehicleAction::NOMINAL;
            predictions[i] =
                predictTrajectory(vehicles[i], mp_, cfg_,
                                  prediction_action, horizon);
        }
    }

    // Every active pair, including pairs in a 3+ vehicle scene, is handled by
    // the same rolling motion coordinator. Pair outputs are merged later by
    // applyActionRequest() using the existing restrictive-action ordering.
    const bool dynamic_speed_enabled = vehicles.size() >= 2;
    auto logNominalRecovery = [&](const std::pair<int, int>& key) {
        if (!dynamic_speed_enabled || reuse_ordinary_coordination) return;
        const auto previous = previous_dynamic_actions_.find(key);
        if (previous == previous_dynamic_actions_.end()) return;
        ++dynamic_speed_metrics_.nominal_recoveries;
        if (!coord_log_sink_) return;
        std::ostringstream line;
        line << "[DYN-SPEED] pair=V" << key.first << "-V" << key.second
             << " baseline=NOMINAL/NOMINAL baseline_conflict=false"
             << " previous_action=" << actionName(previous->second)
             << " selected=NOMINAL/NOMINAL"
             << " reason=rolling_recovery";
        coord_log_sink_(line.str());
    };

    // Frozen A1 departure closures are enforced by A1Coordinator after the
    // ordinary rolling decision. ConflictReservation must not retain pair-wide
    // ownership for either A1 or ordinary-road events.
    for (auto it = conflict_reservations_.begin();
         it != conflict_reservations_.end();) {
        const ConflictReservation& r = it->second;
        logConflictReservation(coord_log_sink_, it->first, "delete", r);
        ++dynamic_speed_metrics_.reservation_deletes;
        it = conflict_reservations_.erase(it);
    }

    std::vector<std::optional<A1Coordinator::WaiterStopConstraint>>
        a1_waiter_constraints(vehicles.size());
    for (size_t index = 0; index < vehicles.size(); ++index) {
        a1_waiter_constraints[index] =
            a1_coordinator_.waiterStopConstraint(vehicles[index]);
    }

    auto eventZone = [&](const PairInteractionResult& interaction,
                         const std::vector<PredictedKinematicSample>& pa,
                         const std::vector<PredictedKinematicSample>& pb) {
        ConflictZone zone;
        auto sampleAt = [](const auto& prediction, double time)
            -> const PredictedKinematicSample& {
            auto it = std::lower_bound(
                prediction.begin(), prediction.end(), time,
                [](const PredictedKinematicSample& sample, double value) {
                    return sample.t < value;
                });
            return it == prediction.end() ? prediction.back() : *it;
        };
        const auto& first_a = sampleAt(pa, interaction.event.first_overlap_t);
        const auto& last_a = sampleAt(pa, interaction.event.last_t);
        const auto& first_b = sampleAt(pb, interaction.event.first_overlap_t);
        const auto& last_b = sampleAt(pb, interaction.event.last_t);
        zone.s_self_enter = first_a.s;
        zone.s_self_exit = last_a.s;
        zone.s_other_enter = first_b.s;
        zone.s_other_exit = last_b.s;
        for (const TimedOverlapGeometry& overlap :
             interaction.event.timed_overlaps) {
            for (const InteractionPoint& point : overlap.polygon) {
                if (!zone.aabb_valid) {
                    zone.aabb_min_x = zone.aabb_max_x = point.x;
                    zone.aabb_min_y = zone.aabb_max_y = point.y;
                    zone.aabb_valid = true;
                } else {
                    zone.aabb_min_x = std::min(zone.aabb_min_x, point.x);
                    zone.aabb_min_y = std::min(zone.aabb_min_y, point.y);
                    zone.aabb_max_x = std::max(zone.aabb_max_x, point.x);
                    zone.aabb_max_y = std::max(zone.aabb_max_y, point.y);
                }
            }
        }
        if (zone.aabb_valid) {
            zone.x = 0.5 * (zone.aabb_min_x + zone.aabb_max_x);
            zone.y = 0.5 * (zone.aabb_min_y + zone.aabb_max_y);
        }
        return zone;
    };

    for (size_t i = 0; i < vehicles.size(); ++i) {
        VehicleAgent& a = vehicles[i];
        if (!a.active() || predictions[i].empty()) continue;
        for (size_t j = i + 1; j < vehicles.size(); ++j) {
            VehicleAgent& b = vehicles[j];
            if (!b.active() || predictions[j].empty()) continue;
            const std::pair<int, int> key{std::min(a.id, b.id),
                                          std::max(a.id, b.id)};
            // Crossing truth is generated directly from synchronized OBBs.
            // Static path geometry is intentionally absent from this call.
            PairInteractionResult direct_interaction =
                detectPairInteractionFromPredictions(
                    a, b, {}, predictions[i], predictions[j]);
            const bool a_clipped_by_a1_stop = direct_interaction.event.valid &&
                a1_waiter_constraints[i] &&
                direct_interaction.event.collision_s_a >
                    a1_waiter_constraints[i]->waiter_stop_s + 1e-9;
            const bool b_clipped_by_a1_stop = direct_interaction.event.valid &&
                a1_waiter_constraints[j] &&
                direct_interaction.event.collision_s_b >
                    a1_waiter_constraints[j]->waiter_stop_s + 1e-9;
            if (a_clipped_by_a1_stop || b_clipped_by_a1_stop) {
                if (coord_log_sink_) {
                    std::ostringstream line;
                    line << std::fixed << std::setprecision(3)
                         << "[A1-STOP-CLIP] pair=V" << key.first << "-V"
                         << key.second << " clipped_waiter=";
                    if (a_clipped_by_a1_stop) {
                        line << "V" << a.id
                             << " collision_s="
                             << direct_interaction.event.collision_s_a
                             << " stop_s="
                             << a1_waiter_constraints[i]->waiter_stop_s;
                    }
                    if (b_clipped_by_a1_stop) {
                        if (a_clipped_by_a1_stop) line << ",";
                        line << "V" << b.id
                             << " collision_s="
                             << direct_interaction.event.collision_s_b
                             << " stop_s="
                             << a1_waiter_constraints[j]->waiter_stop_s;
                    }
                    line << " result=ordinary_overlap_unreachable";
                    coord_log_sink_(line.str());
                }
                direct_interaction.event = TimedConflictEvent{};
                direct_interaction.type = PairInteractionType::NONE;
            }
            // Current-path conflicts remain ordinary even when the same pair
            // owns an active frozen A1 departure transaction. A1Coordinator
            // independently enforces only its frozen protected intervals.
            const bool ordinary = true;
            PairInteractionResult interaction = direct_interaction;

            int ordinary_priority_id = -1;
            PriorityPhysicalTtcEvaluation priority_physical;
            if (ordinary && !reuse_ordinary_coordination) {
                ordinary_priority_id = priorityWinner(a, b);
            }

            const TimedConflictEvent& event = interaction.event;
            if (!event.valid) {
                bool priority_physical_stop = false;
                if (ordinary && !reuse_ordinary_coordination &&
                    (ordinary_priority_id == a.id ||
                     ordinary_priority_id == b.id)) {
                    const bool a_is_priority = ordinary_priority_id == a.id;
                    const VehicleAgent& priority_vehicle =
                        a_is_priority ? a : b;
                    const VehicleAgent& other_vehicle =
                        a_is_priority ? b : a;
                    const auto& priority_prediction =
                        a_is_priority ? predictions[i] : predictions[j];
                    if (priority_vehicle.ttc_stop_hold_remaining <= 1e-9) {
                        priority_physical = evaluatePriorityPhysicalTtc(
                            priority_vehicle, other_vehicle,
                            priority_prediction, mp_, cfg_);
                    }
                }
                if (ordinary && priority_physical.valid) {
                    const TtcStopBoundary boundary = evaluateTtcStopBoundary(
                        priority_physical.safety_ttc,
                        VehicleAction::NOMINAL, cfg_);
                    if (boundary.stop_required) {
                        VehicleAgent& priority_vehicle =
                            ordinary_priority_id == a.id ? a : b;
                        const VehicleAgent& other_vehicle =
                            ordinary_priority_id == a.id ? b : a;
                        priority_vehicle.ttc_stop_hold_remaining = std::max(
                            priority_vehicle.ttc_stop_hold_remaining,
                            cfg_.rolling_refresh_period);
                        applyActionRequest(
                            priority_vehicle, VehicleAction::STOP,
                            "dynamic_speed_STOP_V" +
                                std::to_string(ordinary_priority_id),
                            other_vehicle.id);
                        priority_physical_stop = true;
                        last_rolling_dynamic_decision_.valid = true;
                        last_rolling_dynamic_decision_.baseline_evaluated = true;
                        last_rolling_dynamic_decision_.emergency_stop = true;
                        last_rolling_dynamic_decision_.selected_action_a =
                            ordinary_priority_id == a.id
                                ? VehicleAction::STOP : VehicleAction::NOMINAL;
                        last_rolling_dynamic_decision_.selected_action_b =
                            ordinary_priority_id == b.id
                                ? VehicleAction::STOP : VehicleAction::NOMINAL;
                        last_rolling_dynamic_decision_.targets.push_back(
                            RollingDynamicDecision::Target{
                                priority_vehicle.id,
                                priority_vehicle.path_gen,
                                VehicleAction::STOP,
                                priority_vehicle.action,
                                other_vehicle.id,
                                priority_vehicle.reason});
                        last_rolling_dynamic_decision_.vehicle_ttc_diagnostics.
                            push_back(
                                RollingDynamicDecision::VehicleTtcDiagnostic{
                                    priority_vehicle.id,
                                    priority_vehicle.path_gen,
                                    priority_physical.safety_ttc,
                                    priority_physical.bridge_related
                                        ? "bridge_physical_safety_stop"
                                        : "physical_safety_stop"});
                        if (coord_log_sink_) {
                            std::ostringstream line;
                            line << "[DYN-PHYSICAL] pair=V" << key.first
                                 << "-V" << key.second
                                 << " priority=V" << priority_vehicle.id
                                 << " other_current=V" << other_vehicle.id
                                 << " collision_t="
                                 << priority_physical.collision_t
                                 << " collision_s="
                                 << priority_physical.collision_s
                                 << " safety_ttc="
                                 << priority_physical.safety_ttc
                                 << " boundary_s="
                                 << priority_physical.safety_boundary_s
                                 << " bridge="
                                 << (priority_physical.bridge_related
                                         ? "true" : "false")
                                 << " selected=STOP/NOMINAL"
                                 << " reason=priority_physical_safety_stop";
                            coord_log_sink_(line.str());
                        }
                    }
                }
                if (dynamic_speed_enabled) {
                    last_rolling_dynamic_decision_.baseline_evaluated = true;
                }
                if (!priority_physical_stop) logNominalRecovery(key);
                continue;
            }

            //桥式冲突段的检测，与TTC修正，详细的内容见bridge_ttc_correction.cpp文件
            PairBridgeTtcCorrection bridge_correction;      //创建一个桥式冲突修正结果对象
            if (ordinary) {     //若存在15sOBB的时空冲突
                bridge_correction = evaluateBridgeTtcCorrection(a, b, predictions[i], predictions[j], interaction, mp_, cfg_);  //检测是否存在桥式冲突段
                interaction.type = bridge_correction.a.bridge_related || bridge_correction.b.bridge_related
                        ? PairInteractionType::OPPOSING     //做三目运算，若一辆车识别到是桥，则冲突段类别改为oppsoing
                        : PairInteractionType::CROSSING;
            }   
            //定义一个匿名函数，把刚才算出来的 bridge 信息写进最近创建的 ConflictMarker，用于绘制与日志
            auto annotateBridgeMarker = [&]() {
                if (conflicts_.empty()) return;
                ConflictMarker& marker = conflicts_.back();
                
                if (marker.vehicle_a != a.id || marker.vehicle_b != b.id) {
                    return;
                }
                marker.bridge_a_related = bridge_correction.a.bridge_related;
                marker.bridge_b_related = bridge_correction.b.bridge_related;
                const RoughWp boundary_a = a.track.poseAtS(bridge_correction.a.near_boundary_s);        //将s转为实际的（x,y,theta）
                const RoughWp boundary_b = b.track.poseAtS(bridge_correction.b.near_boundary_s);
                marker.bridge_boundary_a_x = boundary_a.x;
                marker.bridge_boundary_a_y = boundary_a.y;
                marker.bridge_boundary_b_x = boundary_b.x;
                marker.bridge_boundary_b_y = boundary_b.y;
                marker.bridge_corrected_ttc_a = bridge_correction.a.corrected_ttc;
                marker.bridge_corrected_ttc_b = bridge_correction.b.corrected_ttc;
            };

            // 把这次 timed collision 的起始碰撞位置写进最近一个 ConflictMarker，用于日志/RViz 标记。
            auto annotateTimedCollisionStartMarker = [&]() {
                if (conflicts_.empty()) return;
                ConflictMarker& marker = conflicts_.back();
                if (marker.vehicle_a != a.id || marker.vehicle_b != b.id ||
                    marker.kind !=
                        ConflictMarkerKind::CROSSING_OR_OPPOSING) {
                    return;
                }
                const RoughWp collision_a = a.track.poseAtS(event.collision_s_a);
                const RoughWp collision_b = b.track.poseAtS(event.collision_s_b);
                marker.timed_collision_start_valid = true;
                marker.collision_s_a = event.collision_s_a;
                marker.collision_s_b = event.collision_s_b;
                marker.collision_a_x = collision_a.x;
                marker.collision_a_y = collision_a.y;
                marker.collision_b_x = collision_b.x;
                marker.collision_b_y = collision_b.y;
            };

            pairwise_managed_pairs_.insert(key);        //将车辆对信息登记到（冲突事务中）
            ordinary_dynamic_pairs_.insert(key);        //将车辆对信息登记到（动态调速事务中）

            //<输入>1.已检测出的冲突事件，2.a 的预测轨迹。3.b 的预测轨迹----<输出> a的冲突s区间 + b的冲突s区间+ 冲突区域 AABB + 冲突区域中心点
            const ConflictZone zone = eventZone(interaction, predictions[i], predictions[j]);

            //若本轮有冲突，且本轮要求复用已经确定好的协调结果，则不会再做协调决策
            if (ordinary && reuse_ordinary_coordination) {
                recordConflictZones(
                    a, b, std::vector<ConflictZone>{zone},
                    ConflictMarkerKind::CROSSING_OR_OPPOSING,
                    event.first_overlap_t, -1, -1, 0.0,
                    VehicleAction::NOMINAL, -1, -1,
                    decimateTimedOverlaps(event.timed_overlaps),
                    interaction.type, event.last_t);        //记录冲突
                annotateTimedCollisionStartMarker();        //补充碰撞起点
                annotateBridgeMarker();     //补充桥式修正点
                continue;           //然后返回下一对车，继续resolvePairwiseConflicts
            }
            
            //===============（优先级的确定与修正）====================================
            //下面则进行本轮冲突的优先车辆裁定，默认为普通规则决断出的优先级
            int preferred_winner = ordinary_priority_id;

            //桥式局部优先级：一车已进入桥式冲突段、一车仍在桥外时，桥内车辆优先
            const double eps = 1e-6;
            // A车在桥上---（1.必须是桥式冲突段 2.A车已经在桥式冲突段里 ）
            const bool a_in_bridge = bridge_correction.a.bridge_related && a.path_s + eps >= bridge_correction.a.near_boundary_s ;
            const bool b_in_bridge = bridge_correction.b.bridge_related && b.path_s + eps >= bridge_correction.b.near_boundary_s ;

            if (a_in_bridge != b_in_bridge) 
                preferred_winner = a_in_bridge ? a.id : b.id;

            //========(死锁部分)========================================

            const RecoveryDirective& recovery = deadlock_manager_.directive();

            if (ordinary &&(preferred_winner == a.id || preferred_winner == b.id)) 
            {
                const bool a_is_priority = preferred_winner == a.id;
                const VehicleAgent& priority_vehicle = a_is_priority ? a : b;
                const VehicleAgent& other_vehicle = a_is_priority ? b : a;
                const auto& priority_prediction = a_is_priority ? predictions[i] : predictions[j];

                if (priority_vehicle.ttc_stop_hold_remaining <= 1e-9) 
                    priority_physical = evaluatePriorityPhysicalTtc(priority_vehicle, other_vehicle, priority_prediction,mp_, cfg_);
                
            }

            if (dynamic_speed_enabled) {
                ++dynamic_speed_metrics_.baseline_conflicts;
                if (ordinary) {
                    ++dynamic_speed_metrics_.bridge_checked_pairs;
                    dynamic_speed_metrics_.bridge_nearest_evaluations +=
                        bridge_correction.a.nearest_search_evaluations +
                        bridge_correction.b.nearest_search_evaluations;
                    dynamic_speed_metrics_.bridge_backtrack_samples +=
                        bridge_correction.a.backtrack_samples +
                        bridge_correction.b.backtrack_samples;
                    dynamic_speed_metrics_.bridge_max_backtrack_samples =
                        std::max(
                            dynamic_speed_metrics_.bridge_max_backtrack_samples,
                            static_cast<unsigned long long>(std::max(
                                bridge_correction.a.backtrack_samples,
                                bridge_correction.b.backtrack_samples)));
                    if (bridge_correction.a.bridge_related) {
                        ++dynamic_speed_metrics_.bridge_related_a;
                    }
                    if (bridge_correction.b.bridge_related) {
                        ++dynamic_speed_metrics_.bridge_related_b;
                    }
                    if (bridge_correction.a.corrected_ttc + 1e-9 <
                            event.ttc_a ||
                        bridge_correction.b.corrected_ttc + 1e-9 <
                            event.ttc_b) {
                        ++dynamic_speed_metrics_.bridge_corrected_pairs;
                    }
                    if (interaction.type == PairInteractionType::OPPOSING) {
                        ++dynamic_speed_metrics_.opposing_conflicts;
                    } else {
                        ++dynamic_speed_metrics_.crossing_conflicts;
                    }
                }
                PairSpeedCoordinationResult speed_result;
                if (ordinary) {
                    const bool a_is_priority = preferred_winner == a.id;
                    const double effective_ttc_a = std::min(
                        event.ttc_a, bridge_correction.a.corrected_ttc);
                    const double effective_ttc_b = std::min(
                        event.ttc_b, bridge_correction.b.corrected_ttc);
                    PairInteractionResult effective_interaction = interaction;
                    effective_interaction.event.danger_s_a =
                        bridge_correction.a.near_boundary_s;
                    effective_interaction.event.danger_s_b =
                        bridge_correction.b.near_boundary_s;
                    effective_interaction.event.ttc_a =
                        effective_ttc_a;
                    effective_interaction.event.ttc_b =
                        effective_ttc_b;
                    speed_result = evaluatePairSpeedCoordination(
                        a, b, effective_interaction, priority_physical,
                        cfg_, preferred_winner);
                    preferred_winner = speed_result.selected_winner_id;
                    const DynamicInterventionBand intervention_band =
                        speed_result.yielding_band;
                    last_rolling_dynamic_decision_.valid = true;
                    last_rolling_dynamic_decision_.baseline_evaluated = true;
                    last_rolling_dynamic_decision_.band = intervention_band;
                    last_rolling_dynamic_decision_.legacy_fallback = false;
                    last_rolling_dynamic_decision_.emergency_stop =
                        speed_result.emergency_stop;
                    last_rolling_dynamic_decision_.selected_action_a =
                        speed_result.selected_action_a;
                    last_rolling_dynamic_decision_.selected_action_b =
                        speed_result.selected_action_b;
                    last_rolling_dynamic_decision_.baseline_first_overlap_t =
                        event.first_overlap_t;
                    auto recordVehicleTtc = [&](
                            const VehicleAgent& vehicle,
                            const std::optional<double>& ttc,
                            const std::string& reason) {
                        auto& diagnostics = last_rolling_dynamic_decision_.
                            vehicle_ttc_diagnostics;
                        auto existing = std::find_if(
                            diagnostics.begin(), diagnostics.end(),
                            [&](const RollingDynamicDecision::
                                    VehicleTtcDiagnostic& diagnostic) {
                                return diagnostic.vehicle_id == vehicle.id &&
                                       diagnostic.path_gen == vehicle.path_gen;
                            });
                        const RollingDynamicDecision::VehicleTtcDiagnostic
                            incoming{vehicle.id, vehicle.path_gen, ttc, reason};
                        if (existing == diagnostics.end()) {
                            diagnostics.push_back(incoming);
                        } else if (ttc &&
                                   (!existing->ttc ||
                                    *ttc < *existing->ttc)) {
                            *existing = incoming;
                        }
                    };
                    const std::optional<double> priority_ttc =
                        speed_result.priority_physical_ttc;
                    const std::string priority_reason =
                        speed_result.priority_safety_stop
                            ? "safety_stop"
                            : priority_ttc ? "physical_safe" : "clear";
                    recordVehicleTtc(
                        a, a_is_priority
                               ? priority_ttc
                               : speed_result.yielding_effective_ttc,
                        a_is_priority ? priority_reason
                                      : speed_result.reason);
                    recordVehicleTtc(
                        b, a_is_priority
                               ? speed_result.yielding_effective_ttc
                               : priority_ttc,
                        a_is_priority ? speed_result.reason
                                      : priority_reason);
                    if (intervention_band == DynamicInterventionBand::FAR) {
                        ++dynamic_speed_metrics_.far_decisions;
                    } else if (intervention_band ==
                               DynamicInterventionBand::MID) {
                        ++dynamic_speed_metrics_.mid_decisions;
                    } else {
                        ++dynamic_speed_metrics_.near_decisions;
                    }
                    if (speed_result.emergency_stop) {
                        ++dynamic_speed_metrics_.emergency_stop_decisions;
                    }

                    const VehicleAction loser_action =
                        preferred_winner == a.id
                            ? speed_result.selected_action_b
                            : preferred_winner == b.id
                                ? speed_result.selected_action_a
                                : VehicleAction::STOP;
                    if (loser_action == VehicleAction::YIELD) {
                        ++dynamic_speed_metrics_.yield_evaluations;
                    } else if (loser_action == VehicleAction::CREEP) {
                        ++dynamic_speed_metrics_.creep_evaluations;
                    }

                    const VehicleAction previous_a = a.action;
                    const VehicleAction previous_b = b.action;
                    if (speed_result.priority_safety_stop) {
                        VehicleAgent& priority_vehicle =
                            a_is_priority ? a : b;
                        if (priority_vehicle.ttc_stop_hold_remaining <= 1e-9) {
                            priority_vehicle.ttc_stop_hold_remaining =
                                cfg_.rolling_refresh_period;
                        }
                    }
                    if (speed_result.yielding_safety_stop) {
                        VehicleAgent& yielding_vehicle =
                            a_is_priority ? b : a;
                        if (yielding_vehicle.ttc_stop_hold_remaining <= 1e-9) {
                            yielding_vehicle.ttc_stop_hold_remaining =
                                cfg_.rolling_refresh_period;
                        }
                    }
                    const std::string suffix =
                        "_V" + std::to_string(preferred_winner);
                    if (speed_result.selected_action_a !=
                        VehicleAction::NOMINAL) {
                        applyActionRequest(
                            a, speed_result.selected_action_a,
                            "dynamic_speed_" + std::string(actionName(
                                speed_result.selected_action_a)) + suffix,
                            b.id);
                    }
                    if (speed_result.selected_action_b !=
                        VehicleAction::NOMINAL) {
                        applyActionRequest(
                            b, speed_result.selected_action_b,
                            "dynamic_speed_" + std::string(actionName(
                                speed_result.selected_action_b)) + suffix,
                            a.id);
                    }
                    for (const auto& item :
                         {std::make_pair(&a, previous_a),
                          std::make_pair(&b, previous_b)}) {
                        const VehicleAgent* vehicle = item.first;
                        if (vehicle->requested_action ==
                                VehicleAction::NOMINAL ||
                            vehicle->reason.rfind("dynamic_speed_", 0) != 0) {
                            continue;
                        }
                        const RollingDynamicDecision::Target aggregate{
                            vehicle->id, vehicle->path_gen,
                            vehicle->requested_action, item.second,
                            vehicle->blocker_id, vehicle->reason};
                        auto existing_target = std::find_if(
                            last_rolling_dynamic_decision_.targets.begin(),
                            last_rolling_dynamic_decision_.targets.end(),
                            [&](const RollingDynamicDecision::Target& target) {
                                return target.vehicle_id == vehicle->id &&
                                       target.path_gen == vehicle->path_gen;
                            });
                        if (existing_target ==
                            last_rolling_dynamic_decision_.targets.end()) {
                            last_rolling_dynamic_decision_.targets.push_back(
                                aggregate);
                        } else {
                            *existing_target = aggregate;
                        }
                    }

                    if (coord_log_sink_) {
                        const VehicleAction priority_action = a_is_priority
                            ? speed_result.selected_action_a
                            : speed_result.selected_action_b;
                        const VehicleAction yielding_action = a_is_priority
                            ? speed_result.selected_action_b
                            : speed_result.selected_action_a;
                        std::ostringstream ttc_line;
                        ttc_line << std::fixed << std::setprecision(3)
                                 << "[DYN-TTC] pair=V" << key.first << "-V"
                                 << key.second
                                 << " first_overlap_t="
                                 << event.first_overlap_t
                                 << " collision_s_a=" << event.collision_s_a
                                 << " collision_s_b=" << event.collision_s_b
                                 << " danger_s_a="
                                 << bridge_correction.a.near_boundary_s
                                 << " danger_s_b="
                                 << bridge_correction.b.near_boundary_s
                                 << " original_ttc_a=" << event.ttc_a
                                 << " original_ttc_b=" << event.ttc_b
                                 << " effective_ttc_a="
                                 << effective_ttc_a
                                 << " effective_ttc_b="
                                 << effective_ttc_b
                                 << " priority=V" << preferred_winner
                                 << " yield=V"
                                 << (a_is_priority ? b.id : a.id)
                                 << " priority_physical_ttc=";
                        if (speed_result.priority_physical_ttc) {
                            ttc_line << *speed_result.priority_physical_ttc;
                        } else {
                            ttc_line << "CLEAR";
                        }
                        ttc_line << " priority_physical_bridge="
                                 << (speed_result.priority_physical_bridge
                                         ? "true" : "false")
                                 << " yield_effective_ttc="
                                 << *speed_result.yielding_effective_ttc;
                        coord_log_sink_(ttc_line.str());

                        std::ostringstream line;
                        line << std::fixed << std::setprecision(3)
                             << "[DYN-SPEED] pair=V" << key.first << "-V"
                             << key.second
                             << " first_overlap_t=" << event.first_overlap_t
                             << " interaction=GENERIC_TIMED_CONFLICT"
                             << " priority_vehicle=V" << preferred_winner
                             << " yielding_vehicle=V"
                             << (preferred_winner == a.id ? b.id : a.id)
                             << " s_a=" << a.path_s
                             << " s_b=" << b.path_s
                             << " wait_a=" << a.wait_time
                             << " wait_b=" << b.wait_time
                             << " speed_a=" << a.current_speed
                             << " speed_b=" << b.current_speed
                             << " event_exit_a=" << zone.s_self_exit
                             << " event_exit_b=" << zone.s_other_exit
                             << " band="
                             << dynamicInterventionBandName(intervention_band)
                             << " priority_physical_ttc=";
                        if (speed_result.priority_physical_ttc) {
                            line << *speed_result.priority_physical_ttc;
                        } else {
                            line << "CLEAR";
                        }
                        line << " priority_action="
                             << actionName(priority_action)
                             << " priority_stop_threshold=";
                        if (speed_result.priority_stop_threshold) {
                            line << *speed_result.priority_stop_threshold;
                        } else {
                            line << "N/A";
                        }
                        line << " priority_safety_stop="
                             << (speed_result.priority_safety_stop
                                     ? "true" : "false")
                             << " priority_physical_bridge="
                             << (speed_result.priority_physical_bridge
                                     ? "true" : "false")
                             << " yield_effective_ttc="
                             << *speed_result.yielding_effective_ttc
                             << " yield_action=" << actionName(yielding_action)
                             << " yield_stop_threshold=";
                        if (speed_result.yielding_stop_threshold) {
                            line << *speed_result.yielding_stop_threshold;
                        } else {
                            line << "N/A";
                        }
                        line << " yield_safety_stop="
                             << (speed_result.yielding_safety_stop
                                     ? "true" : "false")
                             << " selected="
                             << actionName(speed_result.selected_action_a)
                             << "/"
                             << actionName(speed_result.selected_action_b)
                             << " braking_stop="
                             << (speed_result.emergency_stop ? "true" : "false")
                             << " residual_evaluation=DISABLED"
                             << " reason=" << speed_result.reason
                             << " reservation=not_created";
                        coord_log_sink_(line.str());

                        std::ostringstream bridge_line;
                        bridge_line << std::fixed << std::setprecision(3)
                            << "[BRIDGE-TTC] pair=V" << key.first << "-V"
                            << key.second
                            << " first_overlap_t=" << event.first_overlap_t
                            << " V" << a.id
                            << "_original_danger_s=" << event.collision_s_a
                            << " V" << a.id
                            << "_corrected_boundary_s="
                            << bridge_correction.a.near_boundary_s
                            << " V" << a.id << "_opposing_boundary_s="
                            << bridge_correction.a.opposing_boundary_s
                            << " V" << a.id << "_geometric_boundary_s="
                            << bridge_correction.a.geometric_boundary_s
                            << " V" << a.id << "_original_ttc="
                            << bridge_correction.a.original_ttc
                            << " V" << a.id << "_corrected_ttc="
                            << bridge_correction.a.corrected_ttc
                            << " V" << b.id
                            << "_original_danger_s=" << event.collision_s_b
                            << " V" << b.id
                            << "_corrected_boundary_s="
                            << bridge_correction.b.near_boundary_s
                            << " V" << b.id << "_opposing_boundary_s="
                            << bridge_correction.b.opposing_boundary_s
                            << " V" << b.id << "_geometric_boundary_s="
                            << bridge_correction.b.geometric_boundary_s
                            << " V" << b.id << "_original_ttc="
                            << bridge_correction.b.original_ttc
                            << " V" << b.id << "_corrected_ttc="
                            << bridge_correction.b.corrected_ttc
                            << " bridge_a="
                            << (bridge_correction.a.bridge_related
                                    ? "true" : "false")
                            << " bridge_b="
                            << (bridge_correction.b.bridge_related
                                    ? "true" : "false")
                            << " match_s_a="
                            << bridge_correction.a.matched_other_s
                            << " match_s_b="
                            << bridge_correction.b.matched_other_s
                            << " direction_dot_a="
                            << bridge_correction.a.collision_direction_dot
                            << " direction_dot_b="
                            << bridge_correction.b.collision_direction_dot
                            << " match_distance_a="
                            << bridge_correction.a.collision_match_distance
                            << " match_distance_b="
                            << bridge_correction.b.collision_match_distance
                            << " collision_type_a="
                            << (bridge_correction.a.collision_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " collision_type_b="
                            << (bridge_correction.b.collision_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " boundary_s_a="
                            << bridge_correction.a.near_boundary_s
                            << " boundary_s_b="
                            << bridge_correction.b.near_boundary_s
                            << " boundary_type_a="
                            << (bridge_correction.a.boundary_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " boundary_type_b="
                            << (bridge_correction.b.boundary_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " priority_vehicle=V" << preferred_winner
                            << " yielding_vehicle=V"
                            << (preferred_winner == a.id ? b.id : a.id)
                            << " yielding_ttc="
                            << *speed_result.yielding_effective_ttc
                            << " band="
                            << dynamicInterventionBandName(intervention_band)
                            << " selected="
                            << actionName(speed_result.selected_action_a)
                            << "/"
                            << actionName(speed_result.selected_action_b)
                            << " backtrack_samples="
                            << bridge_correction.a.backtrack_samples << "/"
                            << bridge_correction.b.backtrack_samples
                            << " self_traversal_changes="
                            << bridge_correction.a.self_traversal_changes
                            << "/"
                            << bridge_correction.b.self_traversal_changes
                            << " nearest_other_traversal_changes="
                            << bridge_correction.a.
                                   nearest_other_traversal_changes
                            << "/"
                            << bridge_correction.b.
                                   nearest_other_traversal_changes
                            << " backtrack_end_reason="
                            << bridgeBacktrackEndReasonName(
                                   bridge_correction.a.backtrack_end_reason)
                            << "/"
                            << bridgeBacktrackEndReasonName(
                                   bridge_correction.b.backtrack_end_reason)
                            << " end_query_s="
                            << bridge_correction.a.end_query_s << "/"
                            << bridge_correction.b.end_query_s
                            << " end_match_s="
                            << bridge_correction.a.end_matched_other_s << "/"
                            << bridge_correction.b.end_matched_other_s
                            << " end_match_distance="
                            << bridge_correction.a.end_match_distance << "/"
                            << bridge_correction.b.end_match_distance
                            << " end_direction_dot="
                            << bridge_correction.a.end_direction_dot << "/"
                            << bridge_correction.b.end_direction_dot
                            << " nearest_evaluations="
                            << bridge_correction.a.nearest_search_evaluations
                            << "/"
                            << bridge_correction.b.nearest_search_evaluations
                            << " geometric_attempted="
                            << (bridge_correction.a.
                                    geometric_extension_attempted
                                    ? "true" : "false")
                            << "/"
                            << (bridge_correction.b.
                                    geometric_extension_attempted
                                    ? "true" : "false")
                            << " geometric_applied="
                            << (bridge_correction.a.geometric_extension_applied
                                    ? "true" : "false")
                            << "/"
                            << (bridge_correction.b.geometric_extension_applied
                                    ? "true" : "false")
                            << " geometric_samples="
                            << bridge_correction.a.geometric_outer_samples
                            << "/"
                            << bridge_correction.b.geometric_outer_samples
                            << " geometric_overlap_samples="
                            << bridge_correction.a.geometric_overlap_samples
                            << "/"
                            << bridge_correction.b.geometric_overlap_samples
                            << " geometric_end_reason="
                            << bridgeGeometricEndReasonName(
                                   bridge_correction.a.geometric_end_reason)
                            << "/"
                            << bridgeGeometricEndReasonName(
                                   bridge_correction.b.geometric_end_reason)
                            << " geometric_end_query_s="
                            << bridge_correction.a.geometric_end_query_s
                            << "/"
                            << bridge_correction.b.geometric_end_query_s
                            << " geometric_matched_other_s="
                            << bridge_correction.a.
                                   geometric_end_matched_other_s
                            << "/"
                            << bridge_correction.b.
                                   geometric_end_matched_other_s
                            << " cusp_near_lost="
                            << (bridge_correction.a.cusp_near_relation_loss
                                    ? "true" : "false")
                            << "/"
                            << (bridge_correction.b.cusp_near_relation_loss
                                    ? "true" : "false");
                        coord_log_sink_(bridge_line.str());
                    }

                    recordConflictZones(
                        a, b, std::vector<ConflictZone>{zone},
                        ConflictMarkerKind::CROSSING_OR_OPPOSING,
                        event.first_overlap_t, -1, -1, 0.0,
                        VehicleAction::NOMINAL,
                        preferred_winner,
                        preferred_winner == a.id
                            ? b.id
                            : preferred_winner == b.id ? a.id : -1,
                        decimateTimedOverlaps(event.timed_overlaps),
                        interaction.type, event.last_t);
                    annotateTimedCollisionStartMarker();
                    annotateBridgeMarker();
                    continue;
                }
            }
        }
    }

}

void RuleEngine::enforceFutureA1Admission(
    std::vector<VehicleAgent>& vehicles, double dt) {
    a1_coordinator_.enforceFutureA1Admission(
        vehicles, dt,
        [this](VehicleAgent& vehicle, VehicleAction action,
               const std::string& reason, int blocker_id) {
            applyActionRequest(vehicle, action, reason, blocker_id);
        });
}

void RuleEngine::resolveFollowing(std::vector<VehicleAgent>& vehicles) {
    // 跟车只识别唯一的纵向 leader/follower 并产生低优先级建议。
    // 它不再跳过 timed OBB、不删除 reservation、不参与 holder 选择。
    following_pairs_.clear();
    following_suggestions_.clear();
    std::set<std::pair<int, int>> ambiguous_following_pairs;
    auto motionHeading = [](const VehicleAgent& v) {
        constexpr double kPi = 3.14159265358979323846;
        double heading = v.track.poseAtS(v.path_s).theta;
        if (v.track.typeAtS(v.path_s) == WpType::REVERSE) {
            heading += kPi;
        }
        return heading;
    };

    auto terminalDocking = [&](const VehicleAgent& v) {
        if (!v.active()) return false;
        const double terminal_distance =
            std::max(cfg_.target_request_distance, cfg_.target_stop_distance);
        return v.remainingS() <= terminal_distance;
    };

    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        if (terminalDocking(v)) continue;

        const RoughWp pose_v = v.track.poseAtS(v.path_s);
        const double heading_v = motionHeading(v);

        for (const VehicleAgent& other : vehicles) {
            if (other.id == v.id || !other.active()) continue;
            const RoughWp pose_o = other.track.poseAtS(other.path_s);
            const double heading_o = motionHeading(other);
            const double current_dir_dot =
                std::cos(heading_v) * std::cos(heading_o) +
                std::sin(heading_v) * std::sin(heading_o);
            // Static zone direction is not sufficient near curves/cusps.
            // Require the current physical motion directions (REVERSE already
            // converted by motionHeading) to agree as well.
            if (current_dir_dot <= 0.70) continue;
            // 静态冲突块方向仅作为附加几何条件；当前真实运动方向已在上方
            // 独立校验，避免块中点方向与当前局部方向不一致。
            const std::vector<ConflictZone> fzones = findConflictZones(v, other);
            if (fzones.empty()) continue;
            bool all_same_dir = true;
            for (const ConflictZone& z : fzones) {
                if (!z.same_dir) { all_same_dir = false; break; }
            }
            if (!all_same_dir) continue;

            const double dx = pose_o.x - pose_v.x;
            const double dy = pose_o.y - pose_v.y;
            const double fwd = dx * std::cos(heading_v) +
                               dy * std::sin(heading_v);
            if (fwd <= 0.0) continue;

            const double lat = std::abs(-dx * std::sin(heading_v) +
                                         dy * std::cos(heading_v));
            if (lat > mp_.vehicle_width) continue;

            // 确认 v 在同向局部车道上跟随 other。此集合只用于诊断，
            // resolvePairwiseConflicts 仍会完整执行。
            const std::pair<int, int> key{std::min(v.id, other.id),
                                          std::max(v.id, other.id)};
            if (ambiguous_following_pairs.count(key) != 0) continue;
            if (following_pairs_.count(key) != 0) {
                // Both directed scans claimed to be the follower. Cancel the
                // relation and leave this pair to timed OBB arbitration.
                following_pairs_.erase(key);
                following_suggestions_.erase(key);
                ambiguous_following_pairs.insert(key);
                continue;
            }
            following_pairs_.insert(key);

            const double dist = std::hypot(dx, dy);
            const double gap = dist - mp_.vehicle_length;

            VehicleAction follow_action;
            if (gap <= cfg_.following_min_distance) {
                follow_action = VehicleAction::STOP;
            } else if (gap <= cfg_.following_creep_distance) {
                follow_action = VehicleAction::CREEP;
            } else if (gap <= cfg_.following_normal_distance) {
                follow_action = VehicleAction::YIELD;
            } else {
                continue;
            }
            recordConflictZones(v, other, fzones,
                                ConflictMarkerKind::SAME_DIRECTION, 0.0,
                                v.id, other.id, gap, follow_action);
            following_suggestions_[key] = FollowingSuggestion{
                v.id, other.id, follow_action, gap};
        }
    }
}

void RuleEngine::applyFollowingSuggestions(
    std::vector<VehicleAgent>& vehicles) {
    for (const auto& previous : previous_following_followers_) {
        if (pairwise_managed_pairs_.count(previous.first) == 0) continue;
        for (VehicleAgent& v : vehicles) {
            if (v.id != previous.second || !v.active()) continue;
            if (v.requested_action == VehicleAction::NOMINAL) {
                // Pairwise selected this former follower as the unblocked
                // side. Do not let the old low-priority following action_hold
                // negate that holder decision. Motion still ramps through the
                // normal acceleration limit in the vehicle advance step.
                v.action = VehicleAction::NOMINAL;
                v.action_hold_remaining = 0.0;
            }
            break;
        }
    }

    for (const auto& item : following_suggestions_) {
        if (pairwise_managed_pairs_.count(item.first) != 0) continue;
        const FollowingSuggestion& suggestion = item.second;
        for (VehicleAgent& v : vehicles) {
            if (v.id != suggestion.follower_id || !v.active()) continue;
            // Any earlier arbitration/safety request wins, regardless of
            // action severity. The normal merge is restrictive-only and
            // cannot otherwise express a low-priority STOP suggestion.
            if (v.requested_action != VehicleAction::NOMINAL) break;
            applyActionRequest(
                v, suggestion.action,
                "following_V" + std::to_string(suggestion.leader_id),
                suggestion.leader_id);
            break;
        }
    }
}

void RuleEngine::enforceForwardClearance(std::vector<VehicleAgent>& vehicles,
                                         double dt) {
    // 普适前向净空护栏(§11.13.1 出口检查精神 + 补 following/crossing 分类接缝漏洞)。
    // 接缝 bug:近乎同向、向不同库位汇聚的两车,被 pairwise 当跟车跳过、又不满足
    // resolveFollowing 的横向/间距条件 → 两套都没刹 → 后车 NOMINAL 直撞停在路口的前车
    // → 形成谁前进都撞对方的十字楔死(canStepForward 双 false、破环无效、硬护栏反复)。
    // 兜底:任何车沿自身固定路径在「自己刹车距离 + 车头前伸」内会压上另一辆车的当前
    // 车身,就提前 STOP。不论被哪套逻辑处理这道都在。比硬护栏(0 余量、贴死才停)早刹、
    // 留余量 → 两车干净对停而非重叠 → 破环车 canStepForward 能判出谁可走 → 解开。
    auto bodyAt = [&](const VehicleAgent& v, double s) {
        return makeBody(v.track.poseAtS(s), mp_, 0.0);
    };
    auto nextS = [&](const VehicleAgent& v) {
        if (!v.active()) {
            return v.mode == VehicleMode::DWELL
                ? v.track.length() : v.path_s;
        }
        const MotionOverride motion = motionOverrideFor(v.id);
        if (motion.motion == RecoveryMotion::HOLD) return v.path_s;
        const double desired = motion.motion == RecoveryMotion::RETREAT
            ? cfg_.deadlock_retreat_speed
            : speedForAction(v.requested_action);
        double next_speed = std::max(0.0, v.current_speed);
        if (desired > next_speed) {
            next_speed = std::min(
                desired, next_speed + cfg_.max_accel * dt);
        } else {
            next_speed = std::max(
                desired, next_speed - cfg_.max_decel * dt);
        }
        return motion.motion == RecoveryMotion::RETREAT
            ? std::max(motion.target_s, v.path_s - next_speed * dt)
            : std::min(v.track.length(), v.path_s + next_speed * dt);
    };
    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        const MotionOverride motion = motionOverrideFor(v.id);
        if (v.requested_action == VehicleAction::STOP &&
            motion.motion != RecoveryMotion::RETREAT) continue;
        // 前探距离必须足够远,让车「早早停在冲突区外、留出间隙」,而不是冲到贴上才刹
        // (低速时 brake_dist 极小,只算它会一直蹭到接触才停=楔死)。故在刹车距离之外
        // 再加:车头前伸 + 一个固定安全间隙 kStandoff。kStandoff 同时是「干净对停」后
        // 两车之间留出的余量,使破环车 canStepForward 有空间判别谁能动。另设最小前探
        // kMinLook,保证即便停着(brake_dist≈0)也能看到近处已挡在交叉口的车。
        const double s_end = nextS(v);
        int block_id = -1;
        const OBB body = bodyAt(v, s_end);
        for (const VehicleAgent& o : vehicles) {
            if (o.id == v.id) continue;
            if (o.mode == VehicleMode::NEED_TASK || o.track.empty()) continue;
            if (overlaps(body, bodyAt(o, nextS(o)))) {
                block_id = o.id;
                break;
            }
        }
        if (block_id >= 0) {
            if (motion.a1_intrusion) {
                a1_coordinator_.holdIntrusionCorrection(
                    v.id, "a1_intrusion_next_step_blocked", block_id);
                applyActionRequest(v, VehicleAction::STOP,
                                   "a1_intrusion_next_step_blocked", block_id);
                continue;
            }
            const std::pair<int, int> key{
                std::min(v.id, block_id), std::max(v.id, block_id)};
            const int committed_frames = std::max(
                1, static_cast<int>(std::ceil(
                       cfg_.rolling_refresh_period / std::max(1e-6, dt))));
            const bool executable_frame =
                debug_log_source_ != "ROLLOUT" ||
                (debug_log_frame_id_ >= 0 &&
                 debug_log_frame_id_ < committed_frames);
            if (executable_frame &&
                ordinary_dynamic_pairs_.count(key) != 0 &&
                v.requested_action != VehicleAction::STOP) {
                ++dynamic_speed_metrics_.duplicate_pair_authority_overrides;
            }
            applyActionRequest(v, VehicleAction::STOP,
                               "emergency_next_step_V" +
                                   std::to_string(block_id),
                               block_id);
        }
    }
}

void RuleEngine::resolveTargetSlotOccupancy(
    std::vector<VehicleAgent>& vehicles) {
    constexpr double kMouthWait = 0.35;  // hold this far short of the target
    constexpr double kSlotClear = 0.30;  // occupant must travel this far to free it

    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        const double dist_to_mouth = v.remainingS() - kMouthWait;
        const double v_cur = std::max(0.0, v.current_speed);
        const double braking_dist =
            (v_cur * v_cur) / (2.0 * std::max(1e-6, cfg_.max_decel));
        if (dist_to_mouth > braking_dist + 0.06) continue;

        for (const VehicleAgent& o : vehicles) {
            if (o.id == v.id) continue;
            if (o.current_slot != v.target_slot) continue;  // not at v's slot
            const bool occupying =
                (o.mode == VehicleMode::DWELL) ||
                (o.active() && o.path_s < kSlotClear);
            if (!occupying) continue;
            applyActionRequest(v, VehicleAction::STOP,
                               "wait_slot_V" + std::to_string(o.id), o.id);
            break;
        }
    }
}

void RuleEngine::applyRequestedActions(std::vector<VehicleAgent>& vehicles,
                                       double dt) {
    const double hold = cfg_.action_hold_time;
    for (VehicleAgent& v : vehicles) {
        if (v.mode != VehicleMode::ACTIVE) {
            v.action = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.action_hold_remaining = 0.0;
            v.ttc_stop_hold_remaining = 0.0;
            continue;
        }

        const VehicleAction prev = v.action;            // last cycle's output
        VehicleAction req = v.requested_action;         // this cycle's rules

        if (v.ttc_stop_hold_remaining > 1e-9) {
            req = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.ttc_stop_hold_remaining = std::max(
                0.0, v.ttc_stop_hold_remaining - dt);
            if (v.reason == "clear") v.reason = "ttc_stop_hold";
        }

        if (hold <= 0.0) {                              // smoothing disabled
            v.action = req;
            continue;
        }

        if (moreRestrictive(req, prev)) {
            v.action = req;
            v.action_hold_remaining = hold;
        } else if (req == prev) {
            v.action = prev;
            v.action_hold_remaining =
                std::max(0.0, v.action_hold_remaining - dt);
        } else {
            v.action_hold_remaining -= dt;
            if (v.action_hold_remaining > 0.0) {
                v.action = prev;
                if (v.reason == "clear") v.reason = "action_hold";
            } else {
                v.action = minAction(relaxOneStep(prev), req);
                v.action_hold_remaining = hold;
            }
        }
    }
}

void RuleEngine::arbitrateResources(std::vector<VehicleAgent>& vehicles,
                                    double dt) {
    if (resmap_ == nullptr) return;
    const double front = mp_.body_front_ext();
    const double rear = mp_.body_rear_ext();
    const double decel = std::max(1e-6, cfg_.max_decel);
    const double nominal = speedForAction(VehicleAction::NOMINAL);

    // 资源申请窗口(§15 第二类:不要提前老远占用资源)。只有当车已逼近资源到
    // 「必须开始决策能否在停止线前停住」的预警点时才请求/预约;离得远的资源不预约,
    // 以免一辆还在老远的高优先级车把资源锁死、让近处车无谓干等("明明没啥事就不走")。
    // 窗口 = 标称速刹停距离 + 2 个车身(决策缓冲),纯几何派生,非拍脑袋常数。
    const double request_window =
        nominal * nominal / (2.0 * decel) + 2.0 * mp_.vehicle_length;

    // 1) 按资源聚合请求者:active 车的固定路径会用到该 capacity=1 互斥资源
    //    (窄道/路口/货位口)、车尾还没整车驶出、且已进入申请窗口。
    struct Req { size_t idx; double s_enter; double s_exit; };
    std::map<int, std::vector<Req>> by_res;
    for (size_t i = 0; i < vehicles.size(); ++i) {
        const VehicleAgent& v = vehicles[i];
        if (!v.active()) continue;
        for (const ResourceSpan& sp : v.resource_spans) {
            const TrafficResource* r = resmap_->byId(sp.resource_id);
            if (r == nullptr || r->capacity != 1) continue;
            if (r->type != ResourceType::NARROW &&
                r->type != ResourceType::INTERSECTION &&
                r->type != ResourceType::SLOT_DOCK) {
                continue;
            }
            if (v.path_s - rear > sp.s_exit + 1e-6) continue;  // 已整车驶出
            if (sp.s_enter - v.path_s > request_window) continue;  // 尚远,不预约
            by_res[sp.resource_id].push_back(Req{i, sp.s_enter, sp.s_exit});
        }
    }

    auto bodyInside = [&](const VehicleAgent& v, const Req& q) {
        return v.path_s + front >= q.s_enter - 1e-6 &&
               v.path_s - rear <= q.s_exit + 1e-6;
    };

    // 某车当前车身(后轴还原车身中心)。DWELL 用终点位姿,空轨迹跳过。
    auto poseOf = [&](const VehicleAgent& o) {
        const double s = (o.mode == VehicleMode::DWELL) ? o.track.length()
                                                        : o.path_s;
        return o.track.poseAtS(std::min(s, o.track.length()));
    };
    // 出口检查(§11.4/§11.13.1):候选车驶出资源后的落脚处(s_exit + 半车长)
    // 是否被别的车身占住。被占 → 进去就会卡在资源里 → 不发令牌(除非它已在区内
    // 必须驶完)。这杜绝"令牌发给进得去出不来的车、它攥着令牌却动不了挡死所有人"。
    auto canExit = [&](size_t vi, const Req& q) {
        const VehicleAgent& v = vehicles[vi];
        const double len = v.track.length();
        const double s_check = std::min(q.s_exit + 0.5 * mp_.vehicle_length, len);
        const OBB body = makeBody(v.track.poseAtS(s_check), mp_, 0.0);
        for (size_t o = 0; o < vehicles.size(); ++o) {
            if (o == vi) continue;
            const VehicleAgent& ov = vehicles[o];
            if (ov.mode != VehicleMode::ACTIVE && ov.mode != VehicleMode::DWELL)
                continue;
            if (ov.track.empty()) continue;
            if (overlaps(body, makeBody(poseOf(ov), mp_, 0.0))) return false;
        }
        return true;
    };
    // 可授予 = 已在区内(必须驶完)或 出口畅通。
    auto grantableK = [&](const std::vector<Req>& rs, size_t k) {
        return bodyInside(vehicles[rs[k].idx], rs[k]) || canExit(rs[k].idx, rs[k]);
    };

    // 2) 逐资源仲裁:已持令牌且仍在请求 → 保持(持权到整车驶出,防翻转,§11);
    //    否则按统一优先级 PriorityKey 选 winner 并发令牌(§11.2)。
    for (auto& kv : by_res) {
        const int rid = kv.first;
        std::vector<Req>& reqs = kv.second;

        // 持有者保持令牌的前提:它仍在请求 且 仍可授予(能驶出/已在区内)。
        // 在「可授予」候选里按统一 PriorityKey 选 winner(§11.2)。持令牌者带
        // already_has_token 加成(防翻转,§11),破环车带 emergency 加成(临时最高,§9)
        // —— 于是 破环车 > 持令牌者 > 其他,一套优先级统一裁决,不再用 holder 捷径。
        const int holder = tokens_.holder(rid);
        int winner_k = -1;
        {
            ResourceRequest best;
            bool has_best = false;
            for (size_t k = 0; k < reqs.size(); ++k) {
                if (!grantableK(reqs, k)) continue;  // 出口被堵的不参与(进去会卡死)
                const VehicleAgent& v = vehicles[reqs[k].idx];
                ResourceRequest rq;
                rq.vehicle_id = v.id;
                rq.wait_time = v.wait_time;
                rq.loaded = v.loaded;
                rq.task_count = v.task_count;
                rq.already_inside = bodyInside(v, reqs[k]);
                rq.already_has_token = (v.id == holder);
                rq.emergency_or_clear = false;
                rq.starving = v.wait_time > cfg_.starvation_wait_time;
                rq.eta = rq.already_inside
                             ? 0.0
                             : timeToReachS(v, VehicleAction::NOMINAL,
                                            std::max(0.0, reqs[k].s_enter - front));
                if (!has_best || PriorityKey::betterThan(rq, best)) {
                    best = rq;
                    has_best = true;
                    winner_k = static_cast<int>(k);
                }
            }
        }
        // 无人可授予(都出不去)→ 释放令牌,且下面让所有逼近者停在上游(谁都别进)。
        const int winner_id = (winner_k >= 0) ? vehicles[reqs[winner_k].idx].id : -1;
        if (winner_id >= 0) {
            tokens_.grant(rid, winner_id, now_);  // 授予/刷新(防超时)
        } else {
            tokens_.release(rid);
        }

        // 3) 非 winner(及无人可授予时的全部逼近者):在上游停止线让行。只在「按当前
        //    速度+max_decel 即将刹不住」时才发 STOP(§10 停止线;§15 不过早等——离得
        //    远就继续接近,不原地干等)。
        for (size_t k = 0; k < reqs.size(); ++k) {
            if (winner_k >= 0 && static_cast<int>(k) == winner_k) continue;
            VehicleAgent& v = vehicles[reqs[k].idx];
            const double stop_line = reqs[k].s_enter - front;
            const double dist = stop_line - v.path_s;
            const double vc = std::max(0.0, v.current_speed);
            const double brake = vc * vc / (2.0 * decel) + vc * dt;
            if (dist <= brake + 1e-9) {
                const std::string why = (winner_id >= 0)
                    ? ("res_wait_V" + std::to_string(winner_id))
                    : "res_exit_blocked";
                applyActionRequest(v, VehicleAction::STOP, why, winner_id);
            }
        }
    }

    tokens_.expireStale(now_, std::max(2.0, cfg_.prediction_horizon));
}

void RuleEngine::refreshResourceSpans(std::vector<VehicleAgent>& vehicles) {
    if (resmap_ == nullptr) return;
    for (VehicleAgent& v : vehicles) {
        if (v.track.empty()) {
            if (!v.resource_spans.empty()) v.resource_spans.clear();
            v.spans_track_len = -1.0;
            continue;
        }
        // track 长度变化即视为换了新路径(新任务),重算资源占用区间。
        if (std::abs(v.track.length() - v.spans_track_len) > 1e-6) {
            v.resource_spans = resmap_->spansForPath(v.track);
            v.spans_track_len = v.track.length();
        }
    }
}

void RuleEngine::applyRecoveryPolicy(
    std::vector<VehicleAgent>& vehicles)
{
    const RecoveryDirective& recovery =
        deadlock_manager_.directive();

    for (VehicleAgent& vehicle : vehicles)
    {

        if (!recovery.active())
            continue;

        const RecoveryMotion motion =
            recovery.motionFor(vehicle.id);

        if (motion == RecoveryMotion::HOLD)
        {
            const int blocker_id =
                vehicle.id ==
                        recovery.retreat_vehicle_id
                    ? recovery.pass_vehicle_id
                    : recovery.retreat_vehicle_id;

            applyActionRequest(
                vehicle,
                VehicleAction::STOP,
                "deadlock_hold",
                blocker_id);
        }
        else if (motion ==
                 RecoveryMotion::RETREAT)
        {
            applyActionRequest(
                vehicle,
                VehicleAction::STOP,
                "deadlock_retreat_override",
                recovery.pass_vehicle_id);
        }
    }
}

RuleEngine::MotionOverride RuleEngine::motionOverrideFor(int vehicle_id) const
{
    const RecoveryDirective& recovery = deadlock_manager_.directive();

    // Deadlock recovery 活跃期间，recovery pair 的纵向特殊运动只由 DeadlockManager 控制。
    const bool recovery_vehicle =   recovery.active() &&    (vehicle_id == recovery.retreat_vehicle_id ||   vehicle_id == recovery.pass_vehicle_id);
    if (recovery_vehicle)
        return MotionOverride{recovery.motionFor(vehicle_id),recovery.retreat_target_s,false};
    
    // 非 deadlock recovery 车辆仍正常执行 A1 intrusion correction。
    if (const A1Coordinator::IntrusionCorrection* correction =  a1_coordinator_.intrusionCorrectionFor(vehicle_id))
        return MotionOverride{correction->motion == A1Coordinator::IntrusionCorrectionMotion::RETREAT ? RecoveryMotion::RETREAT: RecoveryMotion::HOLD, correction->target_s,true};
    
    return MotionOverride{RecoveryMotion::NORMAL,0.0,false};
}

void RuleEngine::refreshA1IntrusionCorrections(
    std::vector<VehicleAgent>& vehicles, double dt) {
    const A1IntrusionCorrections previous_corrections =
        a1_coordinator_.intrusionCorrections();
    a1_coordinator_.refreshIntrusionCorrections(vehicles);
    if (coord_log_sink_) {
        for (const auto& entry : a1_coordinator_.intrusionCorrections()) {
            const A1Coordinator::IntrusionCorrection& correction =
                entry.second;
            const auto previous = previous_corrections.find(entry.first);
            const bool changed = previous == previous_corrections.end() ||
                previous->second.motion != correction.motion ||
                previous->second.target_s != correction.target_s ||
                previous->second.blocker_id != correction.blocker_id ||
                previous->second.reason != correction.reason;
            if (!changed) continue;
            const auto waiter = std::find_if(
                vehicles.begin(), vehicles.end(),
                [&](const VehicleAgent& vehicle) {
                    return vehicle.id == correction.waiter_id;
                });
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[A1_INTRUSION_REAL] owner=V" << correction.owner_id
                 << " waiter=V" << correction.waiter_id
                 << " waiter_s="
                 << (waiter != vehicles.end() ? waiter->path_s : -1.0)
                 << " target_s=" << correction.target_s
                 << " motion="
                 << (correction.motion == A1Coordinator::
                            IntrusionCorrectionMotion::RETREAT
                         ? "RETREAT" : "HOLD")
                 << " blocker=V" << correction.blocker_id
                 << " reason=" << correction.reason;
            coord_log_sink_(line.str());
        }
    }
    for (VehicleAgent& vehicle : vehicles) {
        const A1Coordinator::IntrusionCorrection* correction =
            a1_coordinator_.intrusionCorrectionFor(vehicle.id);
        if (correction == nullptr) continue;
        if (!motionOverrideFor(vehicle.id).a1_intrusion) continue;
        applyActionRequest(vehicle, VehicleAction::STOP,
                           correction->reason, correction->blocker_id);
    }
    enforceForwardClearance(vehicles, dt);
    for (VehicleAgent& vehicle : vehicles) {
        const A1Coordinator::IntrusionCorrection* correction =
            a1_coordinator_.intrusionCorrectionFor(vehicle.id);
        if (correction == nullptr) {
            continue;
        }
        if (!motionOverrideFor(vehicle.id).a1_intrusion) continue;
        vehicle.action = VehicleAction::STOP;
        vehicle.requested_action = VehicleAction::STOP;
        vehicle.blocker_id = correction->blocker_id;
    }
}

void RuleEngine::applyRecoveryDirectiveToOutput(
    std::vector<VehicleAgent>& vehicles) {
    applyRecoveryPolicy(vehicles);
    const RecoveryDirective& recovery = deadlock_manager_.directive();
    if (!recovery.active()) return;
    for (VehicleAgent& vehicle : vehicles)
    { 
        if (!recovery.active()) continue;
        if (vehicle.id != recovery.retreat_vehicle_id &&
            vehicle.id != recovery.pass_vehicle_id) continue;
        if (recovery.motionFor(vehicle.id) == RecoveryMotion::NORMAL) {
            continue;
        }
        vehicle.action = vehicle.requested_action;
        if (recovery.motionFor(vehicle.id) == RecoveryMotion::HOLD) {
            vehicle.current_speed = 0.0;
        }
    }
}

void RuleEngine::observeDeadlock(std::vector<VehicleAgent>& vehicles,
                                 double dt, bool emit_logs) {
    std::vector<DeadlockPairGeometry> geometry_items;
    for (size_t i = 0; i < vehicles.size(); ++i) {
        const VehicleAgent& a = vehicles[i];
        if (a.mode != VehicleMode::ACTIVE ||
            a.action != VehicleAction::STOP || a.blocker_id < 0) continue;
        for (size_t j = i + 1; j < vehicles.size(); ++j) {
            const VehicleAgent& b = vehicles[j];
            if (b.mode != VehicleMode::ACTIVE ||
                b.action != VehicleAction::STOP ||
                a.blocker_id != b.id || b.blocker_id != a.id) continue;
            DeadlockPairGeometry geometry;
            geometry.vehicle_a = a.id;
            geometry.vehicle_b = b.id;
            geometry.path_gen_a = a.path_gen;
            geometry.path_gen_b = b.path_gen;
            geometry.preferred_priority_vehicle_id = priorityWinner(a, b);
            geometry_items.push_back(std::move(geometry));
        }
    }
    deadlock_manager_.update(vehicles, geometry_items, dt, emit_logs);
}

void RuleEngine::decide(std::vector<VehicleAgent>& vehicles, double dt,
                        double prediction_horizon_override,
                        bool reuse_ordinary_coordination,
                        const RollingDynamicDecision*
                            period_ordinary_decision) {

    conflicts_.clear();
    last_rolling_dynamic_decision_ = RollingDynamicDecision{};
    now_ += dt;                      // 内部仿真时钟(令牌防抖/超时)
    refreshResourceSpans(vehicles);  // Phase 2:刷新每车路径的资源占用缓存

    previous_following_followers_.clear();
    previous_dynamic_actions_.clear();
    for (const VehicleAgent& v : vehicles) {
        if (v.blocker_id < 0) continue;
        if (!reuse_ordinary_coordination &&
            v.reason.rfind("dynamic_speed_", 0) == 0 &&
                (v.action == VehicleAction::YIELD ||
                 v.action == VehicleAction::CREEP)) {
            const std::pair<int, int> key{std::min(v.id, v.blocker_id),
                                          std::max(v.id, v.blocker_id)};
            previous_dynamic_actions_[key] = v.action;
        }
        const std::string expected =
            "following_V" + std::to_string(v.blocker_id);
        if (v.reason != expected) continue;
        const std::pair<int, int> key{std::min(v.id, v.blocker_id),
                                      std::max(v.id, v.blocker_id)};
        previous_following_followers_[key] = v.id;
    }

    for (VehicleAgent& v : vehicles) {
        if (v.mode != VehicleMode::ACTIVE) {
            v.blocker_id = -1;
            v.requested_action = VehicleAction::STOP;
            v.reason = "not_active";
            v.ttc_stop_hold_remaining = 0.0;
            continue;
        }
        if (v.ttc_stop_hold_remaining > 1e-9) {
            v.requested_action = VehicleAction::STOP;
            if (v.reason.rfind("dynamic_speed_STOP_", 0) != 0) {
                v.reason = "ttc_stop_hold";
            }
            continue;
        }
        v.blocker_id = -1;
        v.requested_action = VehicleAction::NOMINAL;
        v.reason = "clear";
        if (reuse_ordinary_coordination &&
            period_ordinary_decision != nullptr) {
            for (const RollingDynamicDecision::Target& target :
                 period_ordinary_decision->targets) {
                if (target.vehicle_id != v.id ||
                    target.path_gen != v.path_gen) {
                    continue;
                }
                v.requested_action = target.action;
                v.blocker_id = target.blocker_id;
                v.reason = target.reason;
                break;
            }
        }
    }

    // 深层根治(用户洞察:路径固定→只信精确几何):停用粗粒度资源盒仲裁(路口/车道
    // 令牌),它把"共用一个路口盒"当冲突造成幻象冲突→打架→死锁。改由精确的 pairwise
    // 几何冲突(findConflictZones:沿固定路径采样车身OBB,只标真实重叠弧段)作唯一交叉
    // 协调权威。八竿子打不着的两车它根本不报冲突→各自全速。
    // arbitrateResources(vehicles, dt);   // 已停用(资源盒=幻象冲突源)
    const double pairwise_horizon = prediction_horizon_override >= 0.0 ? prediction_horizon_override : cfg_.prediction_horizon;

    refreshDepartureClusterCommitments(vehicles);
    resolvePairwiseConflicts(vehicles, dt, pairwise_horizon,reuse_ordinary_coordination);
    enforceFutureA1Admission(vehicles, dt);
    enforceDepartureClusterCommitments(vehicles, dt);
    resolveTargetSlotOccupancy(vehicles);  // slot-mouth queueing (spec 6/7)
    applyRecoveryPolicy(vehicles);
    enforceForwardClearance(vehicles, dt);
    applyRequestedActions(vehicles, dt);
    observeDeadlock(vehicles, dt, debug_log_source_ == "REAL");

    for (VehicleAgent& v : vehicles) {
        if (v.mode != VehicleMode::ACTIVE) continue;  // DWELL/NEED_TASK do not accumulate
        if (v.action == VehicleAction::STOP ||
            v.action == VehicleAction::CREEP ||
            v.action == VehicleAction::YIELD) {
            v.wait_time += dt;
        } else {
            v.wait_time = 0.0;
        }
    }
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
