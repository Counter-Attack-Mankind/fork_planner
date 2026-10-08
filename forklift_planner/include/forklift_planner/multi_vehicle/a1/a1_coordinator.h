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
#include "forklift_planner/multi_vehicle/future_a1_policy.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"
#include "forklift_planner/multi_vehicle/traffic_resource.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

// Owns A1-specific policy and persistent coordination state. Generic pairwise
// reservation execution and final restrictive action merging remain in
// RuleEngine.
class A1Coordinator {
public:
    using ConflictZone = PotentialConflictZone;
    using GeometryQuery = std::function<std::vector<ConflictZone>(
        const VehicleAgent&, const VehicleAgent&)>;
    using PriorityQuery = std::function<int(const VehicleAgent&,
                                            const VehicleAgent&)>;
    using ActionRequest = std::function<void(
        VehicleAgent&, VehicleAction, const std::string&, int)>;

    struct Dependencies {
        GeometryQuery compute_full_conflict_zones;
        GeometryQuery current_conflict_zones;
        PriorityQuery unified_priority;
    };

    struct FutureA1Commitment {
        int owner_id = -1;
        int owner_path_gen = -1;
        double predicted_a1_arrival_time = -1.0;
        double predicted_to_b_time = -1.0;

        bool valid() const {
            return owner_id >= 0 && owner_path_gen >= 0 &&
                   predicted_a1_arrival_time >= 0.0 &&
                   predicted_to_b_time >= predicted_a1_arrival_time;
        }
    };

    struct DepartureClusterCommitment {
        struct DiagnosticAabb {
            double min_x = 0.0;
            double min_y = 0.0;
            double max_x = 0.0;
            double max_y = 0.0;
            bool valid = false;
        };

        int owner_id = -1;
        int transaction_owner_path_gen = -1;
        int owner_path_gen = -1;
        int other_id = -1;
        int other_path_gen = -1;
        PathTrack frozen_owner_track;
        PathTrack frozen_waiter_track;
        std::vector<size_t> seed_indices;
        std::vector<size_t> cluster_indices;
        std::vector<FutureA1ConflictInterval> intervals;
        // Diagnostic-only geometry frozen with the transaction. It must not
        // participate in authority, stop, release, or arbitration decisions.
        std::vector<DiagnosticAabb> diagnostic_protected_zone_aabbs;
        double waiter_stop_boundary_s = 0.0;
        double waiter_stop_s = 0.0;
        double owner_release_exit_s = 0.0;
        double other_release_exit_s = 0.0;
        bool active = false;
        bool handed_off_from_future = false;
        bool hold_logged = false;
        bool invariant_violation_logged = false;
        bool intrusion_correction_logged = false;
    };

    enum class IntrusionCorrectionMotion {
        RETREAT,
        HOLD,
    };

    struct IntrusionCorrection {
        int owner_id = -1;
        int waiter_id = -1;
        int waiter_path_gen = -1;
        int blocker_id = -1;
        double target_s = 0.0;
        IntrusionCorrectionMotion motion =
            IntrusionCorrectionMotion::HOLD;
        std::string reason;
    };

    struct Snapshot {
        std::map<std::pair<int, int>, DepartureClusterCommitment>
            departure_clusters;
    };

    struct PairAuthority {
        int future_owner_id = -1;
        int departure_owner_id = -1;

        bool a1Related() const {
            return departure_owner_id >= 0 || future_owner_id >= 0;
        }
        int protectedOwner() const {
            return departure_owner_id >= 0 ? departure_owner_id
                                            : future_owner_id;
        }
    };

    struct A1LaunchAdmission {
        bool departure_resource_conflict = false;
        bool source_slot_hold = false;
        bool actual_occupancy_priority = false;
        bool spatial_stop_launch_infeasible = false;
        bool owner_uses_pending_preview = false;
        size_t protected_zone_count = 0;
        double waiter_stop_s = -1.0;
    };

    struct WaiterStopConstraint {
        int owner_id = -1;
        int waiter_id = -1;
        int waiter_path_gen = -1;
        double waiter_stop_s = 0.0;
    };

    struct ArrivalPrediction {
        int vehicle_id = -1;
        int path_gen = -1;
        int service_path_gen = -1;
        double arrival_time = -1.0;
        double to_b_time = -1.0;
    };

    struct ArrivalSummary {
        std::map<int, ArrivalPrediction> candidates;
        std::map<int, std::string> excluded;
    };

    struct ArrivalKinematics {
        double dt = 0.1;
        std::function<bool(int)> enabled;
        // Must reproduce min(NOMINAL speed, the current curvature speed).
        std::function<double(const VehicleAgent&)> desired_speed;
        std::function<double(double, double, double)> limited_speed;
        // Read-only lookup of an already-built B->A1 cache entry.
        std::function<bool(int, PathTrack&)> pickup_leg_track;
    };

    struct ServiceMetrics {
        unsigned long long creates = 0;
        unsigned long long holds = 0;
        unsigned long long changes = 0;
        unsigned long long releases = 0;
        unsigned long long invalidates = 0;
        unsigned long long arrival_ranking_preemptions = 0;
        unsigned long long faster_candidate_observations = 0;
        double active_since = -1.0;
        double max_duration = 0.0;
    };

    using DepartureTransactionIdentity =
        std::tuple<int, int, int, int, int, int, int, bool>;

    A1Coordinator(const MapParam& map_param, const MultiVehicleConfig& cfg,
                  Dependencies dependencies);

    void setCoordLogSink(const std::function<void(const std::string&)>& sink) {
        coord_log_sink_ = sink;
    }
    void setDebugLogContext(const std::string& source, uint64_t plan_id,
                            int frame_id, int rollout_step);

    void refreshPlanningContext(const std::vector<VehicleAgent>& vehicles,
                                double horizon, double now,
                                const ArrivalKinematics& kinematics);
    void setFutureA1Commitment(const FutureA1Commitment& commitment);
    void clearFutureA1Commitment();
    const FutureA1Commitment& futureA1Commitment() const {
        return future_a1_commitment_;
    }
    const ServiceMetrics& serviceMetrics() const { return service_metrics_; }

    Snapshot snapshot() const;
    void restore(const Snapshot& snapshot,
                 bool clear_intrusion_corrections = true);

    PairAuthority authorityForPair(const VehicleAgent& a,
                                   const VehicleAgent& b) const;
    std::optional<WaiterStopConstraint> waiterStopConstraint(
        const VehicleAgent& waiter) const;
    A1LaunchAdmission checkA1LaunchAdmission(
        const VehicleAgent& service_owner,
        const VehicleAgent& launch_candidate) const;

    void refreshDepartureClusterCommitments(
        std::vector<VehicleAgent>& vehicles);
    void enforceFutureA1Admission(std::vector<VehicleAgent>& vehicles,
                                  double dt,
                                  const ActionRequest& request_action);
    void enforceDepartureClusterCommitments(
        std::vector<VehicleAgent>& vehicles, double dt,
        const ActionRequest& request_action);
    void refreshIntrusionCorrections(
        const std::vector<VehicleAgent>& vehicles);
    const IntrusionCorrection* intrusionCorrectionFor(int vehicle_id) const;
    void holdIntrusionCorrection(int vehicle_id,
                                 const std::string& reason,
                                 int blocker_id);
    const std::map<int, IntrusionCorrection>& intrusionCorrections() const {
        return intrusion_corrections_;
    }
    void restoreLiveIntrusionCorrections(
        const std::map<int, IntrusionCorrection>& corrections) {
        intrusion_corrections_ = corrections;
    }

    std::vector<DepartureTransactionIdentity>
    departureTransactionIdentity() const;
    const std::map<std::pair<int, int>, DepartureClusterCommitment>&
    departureClusters() const {
        return departure_cluster_commitments_;
    }
    bool shouldLogA1Decision(const VehicleAgent& vehicle, int blocker_id);

private:
    struct FutureA1ZoneSelection {
        std::vector<ConflictZone> normalized_zones;
        std::vector<size_t> seed_indices;
        std::vector<size_t> protected_indices;
        int upstream_index = -1;
        bool other_already_inside = false;
    };

    struct ConflictCacheEntry {
        int gen_lo = -1;
        int gen_hi = -1;
        std::vector<ConflictZone> blocks;
    };

    ArrivalSummary predictA1Arrivals(
        const std::vector<VehicleAgent>& vehicles, double horizon,
        const ArrivalKinematics& kinematics) const;
    FutureA1Commitment selectFutureA1Owner(
        const std::vector<VehicleAgent>& vehicles,
        const ArrivalSummary& summary) const;
    FutureA1Commitment retainLockedFutureA1Owner(
        const std::vector<VehicleAgent>& vehicles,
        const ArrivalSummary& summary, std::string& retain_reason) const;
    void logFutureA1Transition(const std::vector<VehicleAgent>& vehicles,
                               const FutureA1Commitment& previous,
                               const FutureA1Commitment& current,
                               const ArrivalSummary& summary,
                               const std::string& change_reason,
                               double now, double horizon);

    FutureA1ZoneSelection selectFutureA1ProtectedZones(
        const std::vector<ConflictZone>& canonical_zones,
        bool preview_is_lo, double protected_until,
        double other_path_s) const;
    int futureA1OwnerForPair(const VehicleAgent& a,
                             const VehicleAgent& b) const;
    int departureClusterOwnerForPair(const VehicleAgent& a,
                                     const VehicleAgent& b) const;
    bool waiterPoseClearsFrozenClosure(
        const DepartureClusterCommitment& commitment,
        double waiter_s) const;
    bool waiterRetreatSweepClear(
        const DepartureClusterCommitment& commitment,
        const VehicleAgent& waiter,
        const std::vector<VehicleAgent>& vehicles,
        double target_s, int* blocker_id) const;
    std::string debugLogPrefix() const;

    const MapParam& map_param_;
    const MultiVehicleConfig& cfg_;
    Dependencies dependencies_;
    FutureA1Commitment future_a1_commitment_;
    std::map<std::pair<int, int>, DepartureClusterCommitment>
        departure_cluster_commitments_;
    std::map<int, IntrusionCorrection> intrusion_corrections_;
    mutable std::map<std::pair<int, int>, ConflictCacheEntry>
        future_a1_conflict_cache_;
    std::set<std::pair<int, int>> future_a1_admission_logged_;
    int future_a1_admission_log_owner_id_ = -1;
    int future_a1_admission_log_owner_path_gen_ = -1;
    std::set<std::tuple<int, int, int, int>> a1_decision_logs_;
    ServiceMetrics service_metrics_;
    std::function<void(const std::string&)> coord_log_sink_;
    std::function<bool(int, PathTrack&)> pickup_leg_track_;
    std::string debug_log_source_ = "REAL";
    uint64_t debug_log_plan_id_ = 0;
    int debug_log_frame_id_ = -1;
    int debug_log_rollout_step_ = -1;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
