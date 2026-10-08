#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/a1/a1_coordinator.h"
#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"
#include "forklift_planner/multi_vehicle/deadlock/deadlock_manager.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"
#include "forklift_planner/multi_vehicle/traffic_resource.h"
#include "forklift_planner/multi_vehicle/traffic_resource_map.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class ConflictMarkerKind {
    SAME_DIRECTION,
    CROSSING_OR_OPPOSING,
    POTENTIAL_CONFLICT_ZONE,
    CONFLICT_RESERVATION,
};

struct ConflictMarker {
    using Point = InteractionPoint;
    using TimedOverlap = TimedOverlapGeometry;

    // For CROSSING_OR_OPPOSING, x/y and scale_x/scale_y are the AABB around
    // sampled, time-synchronised OBB intersections. For the two resource-only
    // marker kinds they are a label anchor and diagnostic bounds. None of
    // these display fields participates in arbitration.
    double x = 0.0;
    double y = 0.0;
    double t = 0.0;
    double last_t = 0.0;
    double scale_x = 0.08;
    double scale_y = 0.08;
    int vehicle_a = -1;
    int vehicle_b = -1;
    int follower_id = -1;
    int leader_id = -1;
    int holder_id = -1;
    int waiter_id = -1;
    double following_gap = 0.0;
    VehicleAction following_action = VehicleAction::NOMINAL;
    double follower_x = 0.0;
    double follower_y = 0.0;
    double leader_x = 0.0;
    double leader_y = 0.0;
    PairInteractionType interaction_type = PairInteractionType::NONE;
    // RViz-only location of the two vehicles at the first synchronized OBB
    // overlap. These values never participate in arbitration or TTC logic.
    bool timed_collision_start_valid = false;
    double collision_s_a = 0.0;
    double collision_s_b = 0.0;
    double collision_a_x = 0.0;
    double collision_a_y = 0.0;
    double collision_b_x = 0.0;
    double collision_b_y = 0.0;
    bool bridge_a_related = false;
    bool bridge_b_related = false;
    double bridge_boundary_a_x = 0.0;
    double bridge_boundary_a_y = 0.0;
    double bridge_boundary_b_x = 0.0;
    double bridge_boundary_b_y = 0.0;
    double bridge_corrected_ttc_a = 0.0;
    double bridge_corrected_ttc_b = 0.0;
    std::vector<TimedOverlap> timed_overlaps;
    // Static, time-independent OBB intersections reconstructed from one
    // ConflictZone/reservation arc-length rectangle. Kept separate from
    // timed_overlaps so RViz cannot confuse potential/locked resources with
    // a time-synchronised predicted collision.
    std::vector<std::vector<Point>> spatial_overlap_polygons;
    // Diagnostic identity only. raw_zone_index is stable for one
    // (vehicle pair, path generation pair); active_zone_index is the current
    // findConflictZones() vector position after cleared zones are filtered.
    int raw_zone_index = -1;
    int active_zone_index = -1;
    int path_gen_a = -1;
    int path_gen_b = -1;
    bool zone_aabb_valid = false;
    double s_a_enter = 0.0;
    double s_a_exit = 0.0;
    double s_b_enter = 0.0;
    double s_b_exit = 0.0;
    ConflictMarkerKind kind = ConflictMarkerKind::CROSSING_OR_OPPOSING;
};

class RuleEngine {
public:
    RuleEngine(const MapParam& mp, const MultiVehicleConfig& cfg);

    using FutureA1Commitment = A1Coordinator::FutureA1Commitment;
    using DepartureClusterCommitment =
        A1Coordinator::DepartureClusterCommitment;
    using A1ArrivalKinematics = A1Coordinator::ArrivalKinematics;
    using A1ServiceMetrics = A1Coordinator::ServiceMetrics;
    using DepartureTransactionIdentity =
        A1Coordinator::DepartureTransactionIdentity;

    void setFutureA1Commitment(const FutureA1Commitment& commitment) {
        a1_coordinator_.setFutureA1Commitment(commitment);
    }
    void clearFutureA1Commitment() {
        a1_coordinator_.clearFutureA1Commitment();
    }
    void refreshA1PlanningContext(
        const std::vector<VehicleAgent>& vehicles, double horizon, double now,
        const A1ArrivalKinematics& kinematics) {
        a1_coordinator_.refreshPlanningContext(vehicles, horizon, now,
                                               kinematics);
    }

    void setCoordLogSink(const std::function<void(const std::string&)>& sink) {
        coord_log_sink_ = sink;
        a1_coordinator_.setCoordLogSink(sink);
        deadlock_manager_.setLogSink(sink);
    }
    // Diagnostic metadata only. It never participates in a rule decision.
    void setDebugLogContext(const std::string& source, uint64_t plan_id,
                            int frame_id, int rollout_step) {
        if (source != debug_log_source_ || plan_id != debug_log_plan_id_) {
            pairwise_conflict_logs_.clear();
        }
        debug_log_source_ = source;
        debug_log_plan_id_ = plan_id;
        debug_log_frame_id_ = frame_id;
        debug_log_rollout_step_ = rollout_step;
        a1_coordinator_.setDebugLogContext(source, plan_id, frame_id,
                                           rollout_step);
    }

    // Reservation for one visible spatiotemporal conflict event. Arc ranges
    // use canonical vehicle-id order (lo, hi).
    struct ConflictReservation {
        int owner_id = -1;
        int gen_lo = -1;
        int gen_hi = -1;
        double enter_lo = 0.0;
        double exit_lo = 0.0;
        double enter_hi = 0.0;
        double exit_hi = 0.0;
        double x = 0.0;
        double y = 0.0;
        double first_conflict_t = 0.0;
        std::string create_reason;
        // RViz/debug metadata. These fields never participate in arbitration.
        int raw_zone_index = -1;
        double aabb_min_x = 0.0;
        double aabb_min_y = 0.0;
        double aabb_max_x = 0.0;
        double aabb_max_y = 0.0;
        bool aabb_valid = false;
    };

    // 接入资源地图(Phase 2:资源仲裁需要它把路径映射到资源占用)。
    void setResourceMap(const TrafficResourceMap* m) { resmap_ = m; }

    struct RollingDynamicDecision;

    // prediction_horizon_override >= 0 constrains this decision to the
    // supplied prediction window. reuse_ordinary_coordination skips a new
    // non-A1 pairwise arbitration and reapplies this period's aggregate motion
    // targets; following, A1 reservation/commitment and safety rules still run.
    void decide(std::vector<VehicleAgent>& vehicles, double dt,
                double prediction_horizon_override = -1.0,
                bool reuse_ordinary_coordination = false,
                const RollingDynamicDecision* period_ordinary_decision =
                    nullptr);
    void observeDeadlock(std::vector<VehicleAgent>& vehicles, double dt,
                         bool emit_logs);
    void applyRecoveryDirectiveToOutput(
        std::vector<VehicleAgent>& vehicles);
    struct MotionOverride {
        RecoveryMotion motion = RecoveryMotion::NORMAL;
        double target_s = 0.0;
        bool a1_intrusion = false;
    };
    MotionOverride motionOverrideFor(int vehicle_id) const;
    void refreshA1IntrusionCorrections(
        std::vector<VehicleAgent>& vehicles, double dt);
    using A1IntrusionCorrections =
        std::map<int, A1Coordinator::IntrusionCorrection>;
    A1IntrusionCorrections captureLiveA1IntrusionCorrections() const {
        return a1_coordinator_.intrusionCorrections();
    }
    void restoreLiveA1IntrusionCorrections(
        const A1IntrusionCorrections& corrections) {
        a1_coordinator_.restoreLiveIntrusionCorrections(corrections);
    }
    double speedForAction(VehicleAction action) const;
    const RecoveryDirective& recoveryDirective() const {
        return deadlock_manager_.directive();
    }

    // 前瞻仿真用:快照/恢复跨周期持久状态,使「克隆-空跑」忠实复现真实协调(确定性⇒预测准)。
    struct SimSnapshot {
        std::map<std::pair<int, int>, ConflictReservation> reservations;
        A1Coordinator::Snapshot a1;
        std::set<std::pair<int, int>> following_pairs;
        ResourceTokenTable tokens;
        // conflicts_ is frame output, but sandbox rollout calls decide() and
        // rewrites it on every predicted step. Snapshot it as well so a
        // rollout cannot leak its final predicted frame into current RViz.
        std::vector<ConflictMarker> conflicts;
        DeadlockManager::Snapshot deadlock;
        double now = 0.0;
    };
    SimSnapshot snapshot() const;
    void restore(const SimSnapshot& s, bool restore_deadlock = true);
    // Read-only stress-test diagnostics. This exposes the current commitment
    // without creating, changing, or releasing any coordination state.
    const FutureA1Commitment& futureA1Commitment() const {
        return a1_coordinator_.futureA1Commitment();
    }
    const A1ServiceMetrics& a1ServiceMetrics() const {
        return a1_coordinator_.serviceMetrics();
    }
    std::vector<DepartureTransactionIdentity>
    a1DepartureTransactionIdentity() const {
        return a1_coordinator_.departureTransactionIdentity();
    }
    const std::map<std::pair<int, int>, DepartureClusterCommitment>&
    a1DepartureClusters() const {
        return a1_coordinator_.departureClusters();
    }
    const std::vector<ConflictMarker>& conflicts() const { return conflicts_; }
    // RViz-only resource diagnostics. This reads the current static conflict
    // cache and reservation table; it never creates, updates or releases a
    // reservation and is intentionally outside decide().
    std::vector<ConflictMarker> conflictResourceMarkers(
        const std::vector<VehicleAgent>& vehicles) const;

    int priorityWinner(const VehicleAgent& a, const VehicleAgent& b) const;

    int unifiedPriority(const VehicleAgent& a, const VehicleAgent& b) const;

    struct DynamicSpeedMetrics {
        unsigned long long baseline_conflicts = 0;
        unsigned long long crossing_conflicts = 0;
        unsigned long long opposing_conflicts = 0;
        unsigned long long bridge_checked_pairs = 0;
        unsigned long long bridge_related_a = 0;
        unsigned long long bridge_related_b = 0;
        unsigned long long bridge_corrected_pairs = 0;
        unsigned long long bridge_backtrack_samples = 0;
        unsigned long long bridge_max_backtrack_samples = 0;
        unsigned long long bridge_nearest_evaluations = 0;
        unsigned long long same_direction_conflicts = 0;
        unsigned long long far_decisions = 0;
        unsigned long long mid_decisions = 0;
        unsigned long long near_decisions = 0;
        unsigned long long emergency_stop_decisions = 0;
        unsigned long long yield_evaluations = 0;
        unsigned long long yield_conflict_free = 0;
        unsigned long long yield_delayed = 0;
        unsigned long long creep_evaluations = 0;
        unsigned long long creep_conflict_free = 0;
        unsigned long long creep_delayed = 0;
        unsigned long long selected_conflict_remaining = 0;
        unsigned long long a1_fallbacks = 0;
        unsigned long long existing_reservation_skips = 0;
        unsigned long long nominal_recoveries = 0;
        unsigned long long duplicate_pair_authority_overrides = 0;
        unsigned long long reservation_creates = 0;
        unsigned long long reservation_updates = 0;
        unsigned long long reservation_deletes = 0;
        unsigned long long reservation_create_ordinary_dynamic = 0;
        unsigned long long reservation_create_a1 = 0;
        unsigned long long reservation_create_terminal = 0;
        unsigned long long reservation_create_already_inside = 0;
        unsigned long long reservation_create_braking_safety = 0;
        unsigned long long reservation_create_multi_vehicle = 0;
        unsigned long long reservation_create_other = 0;
    };

    using A1LaunchAdmission = A1Coordinator::A1LaunchAdmission;

    // Read-only resource check for a vehicle that is still parked at B and is
    // considering a TO_A1 launch. Service-owner identity alone is never a
    // reason to hold: only overlap with the owner's protected A1->B prefix is.
    A1LaunchAdmission checkA1LaunchAdmission(
        const VehicleAgent& service_owner,
        const VehicleAgent& launch_candidate) const;

    struct SlotDepartureAdmission {
        bool clear = true;
        bool a1_departure_conflict = false;
        bool ordinary_road_conflict = false;
        int blocker_id = -1;
        double first_conflict_t = -1.0;
        double candidate_conflict_s = -1.0;
        PairInteractionType interaction_type = PairInteractionType::NONE;
        A1LaunchAdmission a1;
    };

    // Unified pre-activation gate for a candidate B->A1 leg. It predicts the
    // complete horizon but holds only when the candidate conflicts before its
    // complete body clears the source-slot sweep.
    SlotDepartureAdmission checkSlotDepartureAdmission(
        const VehicleAgent* service_owner,
        const VehicleAgent& launch_candidate,
        const std::vector<VehicleAgent>& vehicles,
        double prediction_horizon) const;
    const DynamicSpeedMetrics& dynamicSpeedMetrics() const {
        return dynamic_speed_metrics_;
    }

    struct RollingDynamicDecision {
        struct VehicleTtcDiagnostic {
            int vehicle_id = -1;
            int path_gen = 0;
            std::optional<double> ttc;
            std::string reason = "clear";
        };
        struct Target {
            int vehicle_id = -1;
            int path_gen = 0;
            VehicleAction action = VehicleAction::NOMINAL;
            VehicleAction previous_action = VehicleAction::NOMINAL;
            int blocker_id = -1;
            std::string reason;
        };
        bool valid = false;
        bool baseline_evaluated = false;
        DynamicInterventionBand band = DynamicInterventionBand::FAR;
        bool legacy_fallback = false;
        bool emergency_stop = false;
        VehicleAction selected_action_a = VehicleAction::NOMINAL;
        VehicleAction selected_action_b = VehicleAction::NOMINAL;
        std::optional<double> baseline_first_overlap_t;
        std::vector<Target> targets;
        std::vector<VehicleTtcDiagnostic> vehicle_ttc_diagnostics;
    };
    const RollingDynamicDecision& lastRollingDynamicDecision() const {
        return last_rolling_dynamic_decision_;
    }

    // Read-only coordination interface. The path_gen-keyed geometry cache is
    // the only mutable implementation detail this may populate.
    PairInteractionResult detectPairInteraction(
        const VehicleAgent& a, const VehicleAgent& b,
        double prediction_horizon) const;

    // 诊断:打印一对车的冲突区全貌(各块 se/sx、same_dir、committed、停止线、owner 预约、
    // 是否在 following_pairs_)。供无头批处理在碰撞现场调用,定位"该不该门控/谁越线"。
    void debugDumpConflict(const VehicleAgent& a, const VehicleAgent& b) const;

private:
    using ConflictZone = PotentialConflictZone;

    // One-cycle longitudinal hint. It is intentionally separate from
    // VehicleAgent::requested_action because following is not an arbitration
    // authority: pairwise reservations/priority and all later safety rules
    // must be able to decide first.
    struct FollowingSuggestion {
        int follower_id = -1;
        int leader_id = -1;
        VehicleAction action = VehicleAction::NOMINAL;
        double gap = 0.0;
    };

    // 当前位置下、属于 (self,other) 的有效冲突块:取缓存的静态 C_ij(见
    // conflictBlocksCanonical),按 self/other 朝向取用,并裁掉任一方已完全清出的块、
    // 把入口夹到各自车尾起点——与历史"逐拍沿剩余路径扫描"的产物在同一离散精度下等价。
    std::vector<ConflictZone> findConflictZones(const VehicleAgent& self,
                                                const VehicleAgent& other) const;
    // 在两条「完全固定」路径的整段 [0,L]×[0,L] 上一次算定的静态冲突集 C_ij(与
    // 时间/速度/朝向/当前位置无关)。仅几何,重活在此。
    std::vector<ConflictZone> computeConflictZonesFull(
        const VehicleAgent& self, const VehicleAgent& other) const;
    // 取 (lo,hi)(lo.id<hi.id)的静态 C_ij,按 path_gen 缓存;任一方换路径才重算。
    // 返回的块以 self=lo、other=hi 的朝向存储(s_self_*=lo 路径弧长)。
    const std::vector<ConflictZone>& conflictBlocksCanonical(
        const VehicleAgent& lo, const VehicleAgent& hi) const;
    void recordConflictZones(const VehicleAgent& self,
                             const VehicleAgent& other,
                             const std::vector<ConflictZone>& zones,
                             ConflictMarkerKind kind,
                             double first_conflict_t = 0.0,
                             int follower_id = -1,
                             int leader_id = -1,
                             double following_gap = 0.0,
                             VehicleAction following_action =
                                 VehicleAction::NOMINAL,
                             int holder_id = -1,
                             int waiter_id = -1,
                             const std::vector<ConflictMarker::TimedOverlap>&
                                 timed_overlaps = {},
                             PairInteractionType interaction_type =
                                 PairInteractionType::NONE,
                             double last_conflict_t = -1.0);
    double timeToReachS(const VehicleAgent& v, VehicleAction action,
                        double target_s) const;
    double predictedTravelDistance(const VehicleAgent& v,
                                   VehicleAction action,
                                   double t) const;
    void applyActionRequest(VehicleAgent& v, VehicleAction action,
                            const std::string& reason, int blocker_id = -1);
    void resolvePairwiseConflicts(std::vector<VehicleAgent>& vehicles, double dt,
                                  double prediction_horizon,
                                  bool reuse_ordinary_coordination);
    void enforceFutureA1Admission(std::vector<VehicleAgent>& vehicles,
                                  double dt);
    void refreshDepartureClusterCommitments(
        std::vector<VehicleAgent>& vehicles);
    void enforceDepartureClusterCommitments(
        std::vector<VehicleAgent>& vehicles, double dt);
    int departureClusterOwnerForPair(const VehicleAgent& a,
                                     const VehicleAgent& b) const;
    void resolveFollowing(std::vector<VehicleAgent>& vehicles);
    void applyFollowingSuggestions(std::vector<VehicleAgent>& vehicles);
    // 普适前向净空护栏:任何车若沿自身固定路径在自己刹车距离内会撞上另一辆车的当前
    // 车身,提前 STOP(留余量、干净对停)。堵死 following/crossing 分类接缝处「两套都
    // 没刹→NOMINAL 直撞停着的车→十字楔死」的漏洞。比硬护栏早刹留余量。
    void enforceForwardClearance(std::vector<VehicleAgent>& vehicles,
                                 double dt);
    void resolveTargetSlotOccupancy(std::vector<VehicleAgent>& vehicles);
    void applyRequestedActions(std::vector<VehicleAgent>& vehicles, double dt);
    void applyRecoveryPolicy(std::vector<VehicleAgent>& vehicles);
    // Phase 2:track 变化时,用资源地图重算每车路径的资源占用区间(缓存到 agent)。
    void refreshResourceSpans(std::vector<VehicleAgent>& vehicles);
    // Phase 2.2:对 capacity=1 互斥资源(窄道/路口/货位口)按统一优先级发令牌,
    // winner 通过、其余在上游停止线让行;令牌持有到车身驶出才释放(§11)。
    void arbitrateResources(std::vector<VehicleAgent>& vehicles, double dt);
    std::string debugLogPrefix() const;

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
    std::vector<ConflictMarker> conflicts_;
    // Diagnostic de-duplication only: one pair/holder event per plan context.
    std::set<std::tuple<int, int, int>> pairwise_conflict_logs_;
    bool shouldLogA1Decision(const VehicleAgent& vehicle, int blocker_id);

    // Stage 3.1: reservations are retained only for A1 service transactions.
    // Ordinary-road pairs use rolling motion actions without cross-period
    // holder/waiter ownership.
    std::map<std::pair<int, int>, ConflictReservation> conflict_reservations_;

    // 每周期重算的唯一有向跟车对，仅作诊断/纵向建议依据。
    // pairwise timed OBB、reservation 和 holder 仲裁不再使用它作跳过条件。
    std::set<std::pair<int, int>> following_pairs_;
    std::map<std::pair<int, int>, FollowingSuggestion> following_suggestions_;
    // Reconstructed before each reset from the live vehicle reason. This is
    // diagnostic-only and never participates in coordination.
    std::map<std::pair<int, int>, VehicleAction> previous_dynamic_actions_;
    // Directed following actions applied in the preceding decision, captured
    // before VehicleAgent::reason is reset. Used only to release a stale
    // following action_hold when pairwise now grants that follower passage.
    std::map<std::pair<int, int>, int> previous_following_followers_;
    // Pairs for which reservation/OBB arbitration supplied the result in this
    // decision. A following hint must not be applied to those pairs.
    std::set<std::pair<int, int>> pairwise_managed_pairs_;
    // Current decide() calls in which an ordinary pair was handled by the
    // unified rolling coordinator (including frozen-period reuse). This is
    // diagnostic input for detecting a downstream authority override.
    std::set<std::pair<int, int>> ordinary_dynamic_pairs_;

    // 静态冲突集 C_ij 缓存(协调图第一步)。key={lo.id,hi.id};块以 self=lo 朝向存储。
    // gen_lo/gen_hi 记录算定时两车的 path_gen;任一方 path_gen 变(换了固定路径)即失效
    // 重算。把"逐拍沿固定路径全程精扫"摊销成"每对路径只算一次",冲突几何恒定可见、不抖动。
    struct ConflictCacheEntry {
        int gen_lo = -1;
        int gen_hi = -1;
        std::vector<ConflictZone> blocks;  // canonical: s_self_*=lo, s_other_*=hi
        // Lazily reconstructed RViz geometry, aligned one-to-one with blocks.
        // It is diagnostic-only and has no role in conflict decisions.
        std::vector<std::vector<std::vector<ConflictMarker::Point>>>
            display_overlap_polygons;
    };
    mutable std::map<std::pair<int, int>, ConflictCacheEntry> conflict_cache_;

    // Phase 2 资源模型:资源地图(只读)+ 资源令牌表(跨周期持久,§11.10)。
    const TrafficResourceMap* resmap_ = nullptr;
    ResourceTokenTable tokens_;
    std::function<void(const std::string&)> coord_log_sink_;
    std::string debug_log_source_ = "REAL";
    uint64_t debug_log_plan_id_ = 0;
    int debug_log_frame_id_ = -1;
    int debug_log_rollout_step_ = -1;
    // Intentionally excluded from SimSnapshot: rollout evaluations are
    // diagnostic evidence, and these counters never affect restored control.
    DynamicSpeedMetrics dynamic_speed_metrics_;
    RollingDynamicDecision last_rolling_dynamic_decision_;
    double now_ = 0.0;  // 内部仿真时钟(每 decide 累加 dt),供令牌防抖/超时用
    A1Coordinator a1_coordinator_;
    DeadlockManager deadlock_manager_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
