#include "forklift_planner/multi_vehicle/a1/a1_coordinator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {
namespace {

constexpr double kIntrusionRetreatClearance = 0.02;
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

bool isB0ToB9Source(const VehicleAgent& vehicle) {
    return vehicle.current_slot >= 0 && vehicle.current_slot <= 9;
}

std::string readableSimTime(double seconds) {
    const double nonnegative = std::max(0.0, seconds);
    const long long tenths =
        static_cast<long long>(std::llround(nonnegative * 10.0));
    const long long minutes = tenths / 600;
    const double remainder = static_cast<double>(tenths % 600) / 10.0;
    char text[80];
    std::snprintf(text, sizeof(text), "%lldmin%.1fs", minutes, remainder);
    return text;
}

const VehicleAgent* agentById(const std::vector<VehicleAgent>& vehicles,
                              int id) {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.id == id) return &vehicle;
    }
    return nullptr;
}

VehicleAgent* agentById(std::vector<VehicleAgent>& vehicles, int id) {
    for (VehicleAgent& vehicle : vehicles) {
        if (vehicle.id == id) return &vehicle;
    }
    return nullptr;
}

bool sameDepartureCluster(
    const A1Coordinator::DepartureClusterCommitment& a,
    const A1Coordinator::DepartureClusterCommitment& b) {
    return a.owner_id == b.owner_id &&
           a.transaction_owner_path_gen == b.transaction_owner_path_gen &&
           a.owner_path_gen == b.owner_path_gen &&
           a.other_id == b.other_id &&
           a.other_path_gen == b.other_path_gen &&
           a.seed_indices == b.seed_indices &&
           a.cluster_indices == b.cluster_indices &&
           a.waiter_stop_boundary_s == b.waiter_stop_boundary_s &&
           a.waiter_stop_s == b.waiter_stop_s &&
           a.owner_release_exit_s == b.owner_release_exit_s &&
           a.other_release_exit_s == b.other_release_exit_s &&
           a.active == b.active &&
           a.handed_off_from_future == b.handed_off_from_future &&
           a.invariant_violation_logged == b.invariant_violation_logged &&
           a.intrusion_correction_logged ==
               b.intrusion_correction_logged;
}

void logDepartureCluster(
    const std::function<void(const std::string&)>& sink,
    const char* event, const char* reason,
    const A1Coordinator::DepartureClusterCommitment& commitment,
    double owner_s, double other_s) {
    if (!sink) return;
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[DEPARTURE_CLUSTER] event=" << event
         << " reason=" << reason
         << " owner=V" << commitment.owner_id
         << " transaction_owner_gen="
         << commitment.transaction_owner_path_gen
         << " owner_gen=" << commitment.owner_path_gen
         << " other=V" << commitment.other_id
         << " other_gen=" << commitment.other_path_gen
         << " zones=[";
    for (size_t i = 0; i < commitment.cluster_indices.size(); ++i) {
        if (i > 0) line << ",";
        line << commitment.cluster_indices[i];
    }
    line << "] waiter_stop_boundary_s="
         << commitment.waiter_stop_boundary_s
         << " stop_s=" << commitment.waiter_stop_s
         << " owner_release_exit_s=" << commitment.owner_release_exit_s
         << " frozen_owner_track_length="
         << commitment.frozen_owner_track.length()
         << " owner_s=" << owner_s
         << " other_s=" << other_s
         << " future_handoff="
         << (commitment.handed_off_from_future ? "true" : "false");
    sink(line.str());
}

void logAdmissionInvariantViolation(
    const std::function<void(const std::string&)>& sink,
    const A1Coordinator::DepartureClusterCommitment& commitment,
    const VehicleAgent& owner, const VehicleAgent& waiter,
    const char* reason) {
    if (!sink) return;
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[A1_ADMISSION_INVARIANT_VIOLATION]"
         << " owner=V" << owner.id
         << " waiter=V" << waiter.id
         << " owner_phase=" << missionPhaseName(owner.mission_phase)
         << " waiter_phase=" << missionPhaseName(waiter.mission_phase)
         << " waiter_s=" << waiter.path_s
         << " stop_s=" << commitment.waiter_stop_s
         << " waiter_boundary_s=" << commitment.waiter_stop_boundary_s
         << " owner_progress_s=" << owner.path_s
         << " owner_current_gen=" << owner.path_gen
         << " frozen_owner_gen=" << commitment.owner_path_gen
         << " waiter_current_gen=" << waiter.path_gen
         << " frozen_waiter_gen=" << commitment.other_path_gen
         << " reason=" << reason;
    sink(line.str());
}

}  // namespace

A1Coordinator::A1Coordinator(const MapParam& map_param,const MultiVehicleConfig& cfg,Dependencies dependencies)
                            : map_param_(map_param), cfg_(cfg),dependencies_(std::move(dependencies)) {}

void A1Coordinator::setDebugLogContext(const std::string& source,uint64_t plan_id, int frame_id,int rollout_step)
{
    if (source != debug_log_source_ || plan_id != debug_log_plan_id_) {
        a1_decision_logs_.clear();
    }
    debug_log_source_ = source;
    debug_log_plan_id_ = plan_id;
    debug_log_frame_id_ = frame_id;
    debug_log_rollout_step_ = rollout_step;
}

std::string A1Coordinator::debugLogPrefix() const {
    std::ostringstream out;
    out << "[SOURCE=" << debug_log_source_ << "]"
        << " [plan=" << debug_log_plan_id_ << "]"
        << " [frame=" << debug_log_frame_id_ << "]"
        << " [rollout_step=" << debug_log_rollout_step_ << "]";
    return out.str();
}

bool A1Coordinator::shouldLogA1Decision(const VehicleAgent& vehicle,
                                        int blocker_id) {
    return a1_decision_logs_.insert(std::make_tuple(
        vehicle.id, blocker_id, static_cast<int>(vehicle.mission_phase),
        static_cast<int>(vehicle.requested_action))).second;
}

void A1Coordinator::setFutureA1Commitment(
    const FutureA1Commitment& commitment) {
    if (commitment.owner_id != future_a1_admission_log_owner_id_ ||
        commitment.owner_path_gen !=
            future_a1_admission_log_owner_path_gen_) {
        future_a1_admission_logged_.clear();
        future_a1_admission_log_owner_id_ = commitment.owner_id;
        future_a1_admission_log_owner_path_gen_ = commitment.owner_path_gen;
    }
    future_a1_commitment_ = commitment;
}

void A1Coordinator::clearFutureA1Commitment() {
    future_a1_commitment_ = FutureA1Commitment{};
}

A1Coordinator::ArrivalSummary A1Coordinator::predictA1Arrivals(
    const std::vector<VehicleAgent>& vehicles, double horizon,
    const ArrivalKinematics& kinematics) const {
    ArrivalSummary summary;
    const double dt = kinematics.dt;
    const auto trackTime = [&](VehicleAgent preview,
                               double time_limit) -> double {
        if (preview.track.empty()) return -1.0;
        if (preview.path_s >= preview.track.length() - 1e-9) return 0.0;
        const int max_steps = std::max(
            0, static_cast<int>(std::ceil(std::max(0.0, time_limit) / dt)));
        for (int step = 1; step <= max_steps; ++step) {
            const double desired = kinematics.desired_speed
                ? kinematics.desired_speed(preview) : 0.0;
            if (!std::isfinite(desired) || desired <= 1e-9) return -1.0;
            preview.current_speed = kinematics.limited_speed
                ? kinematics.limited_speed(preview.current_speed, desired, dt)
                : desired;
            const double old_s = preview.path_s;
            preview.path_s = std::min(
                preview.track.length(),
                preview.path_s + preview.current_speed * dt);
            if (preview.path_s <= old_s + 1e-12) return -1.0;
            if (preview.path_s >= preview.track.length() - 1e-9) {
                return static_cast<double>(step) * dt;
            }
        }
        return -1.0;
    };

    const auto pickupTrack = [&](int slot, PathTrack& out) {
        return kinematics.pickup_leg_track &&
               kinematics.pickup_leg_track(slot, out) && !out.empty();
    };

    for (const VehicleAgent& vehicle : vehicles) {
        if (kinematics.enabled && !kinematics.enabled(vehicle.id)) {
            continue;
        }

        double arrival_time = -1.0;
        int service_path_gen = vehicle.path_gen;
        if (vehicle.active() &&
            vehicle.mission_phase == MissionPhase::TO_A1 &&
            vehicle.leg_target == LegTargetKind::A1) {
            arrival_time = trackTime(vehicle, horizon);
        } else if (vehicle.active() &&
                   vehicle.mission_phase == MissionPhase::TO_B &&
                   vehicle.leg_target == LegTargetKind::B_SLOT) {
            const double to_b_time = trackTime(vehicle, horizon);
            PathTrack next_pickup;
            if (to_b_time < 0.0) {
                // The current A1->B leg alone already exceeds the horizon.
            } else if (pickupTrack(vehicle.target_slot, next_pickup)) {
                VehicleAgent pickup = vehicle;
                pickup.track = next_pickup;
                pickup.path_s = 0.0;
                pickup.current_speed = 0.0;
                pickup.mission_phase = MissionPhase::TO_A1;
                pickup.leg_target = LegTargetKind::A1;
                const double fixed_time =
                    to_b_time + cfg_.unload_dwell_time;
                const double pickup_time = trackTime(
                    pickup, std::max(0.0, horizon - fixed_time));
                if (pickup_time >= 0.0) {
                    arrival_time = fixed_time + pickup_time;
                }
                service_path_gen = vehicle.path_gen + 1;
            } else {
                summary.excluded[vehicle.id] = "pickup_leg_unavailable";
            }
        } else if (vehicle.mode == VehicleMode::DWELL &&
                   vehicle.mission_phase == MissionPhase::UNLOAD_DWELL) {
            PathTrack next_pickup;
            if (pickupTrack(vehicle.current_slot, next_pickup)) {
                VehicleAgent pickup = vehicle;
                pickup.track = next_pickup;
                pickup.path_s = 0.0;
                pickup.current_speed = 0.0;
                pickup.mission_phase = MissionPhase::TO_A1;
                pickup.leg_target = LegTargetKind::A1;
                const double fixed_time =
                    std::max(0.0, vehicle.dwell_remaining);
                const double pickup_time = trackTime(
                    pickup, std::max(0.0, horizon - fixed_time));
                if (pickup_time >= 0.0) {
                    arrival_time = fixed_time + pickup_time;
                }
                service_path_gen = vehicle.path_gen + 1;
            } else {
                summary.excluded[vehicle.id] = "pickup_leg_unavailable";
            }
        } else {
            continue;
        }
        if (!futureA1ArrivalWithinHorizon(arrival_time, horizon)) {
            if (summary.excluded.find(vehicle.id) == summary.excluded.end()) {
                summary.excluded[vehicle.id] = "horizon_exceeded";
            }
            continue;
        }

        ArrivalPrediction prediction;
        prediction.vehicle_id = vehicle.id;
        prediction.path_gen = vehicle.path_gen;
        prediction.service_path_gen = service_path_gen;
        prediction.arrival_time = arrival_time;
        prediction.to_b_time = arrival_time + cfg_.pickup_dwell_time;
        summary.candidates[vehicle.id] = prediction;
    }
    return summary;
}

A1Coordinator::FutureA1Commitment A1Coordinator::selectFutureA1Owner(
    const std::vector<VehicleAgent>& vehicles,
    const ArrivalSummary& summary) const {
    const double tie_window = std::max(0.02, cfg_.prediction_step);
    std::vector<FutureA1RankedCandidate> ranked;
    ranked.reserve(summary.candidates.size());
    for (const auto& item : summary.candidates) {
        ranked.push_back({item.second.vehicle_id, item.second.arrival_time});
    }
    const int best_id = selectFutureA1Candidate(
        ranked, tie_window, [&](int lhs, int rhs) {
            const VehicleAgent* lhs_agent = agentById(vehicles, lhs);
            const VehicleAgent* rhs_agent = agentById(vehicles, rhs);
            if (lhs_agent == nullptr || rhs_agent == nullptr ||
                !dependencies_.unified_priority) {
                return -1;
            }
            return dependencies_.unified_priority(*lhs_agent, *rhs_agent);
        });

    FutureA1Commitment commitment;
    const auto best = summary.candidates.find(best_id);
    if (best == summary.candidates.end()) return commitment;
    commitment.owner_id = best->second.vehicle_id;
    commitment.owner_path_gen = best->second.service_path_gen;
    commitment.predicted_a1_arrival_time = best->second.arrival_time;
    commitment.predicted_to_b_time = best->second.to_b_time;
    return commitment;
}

A1Coordinator::FutureA1Commitment
A1Coordinator::retainLockedFutureA1Owner(
    const std::vector<VehicleAgent>& vehicles,
    const ArrivalSummary& summary, std::string& retain_reason) const {
    retain_reason = "no_service_owner";
    if (!future_a1_commitment_.valid()) return FutureA1Commitment{};

    const VehicleAgent* owner = agentById(vehicles,
                                          future_a1_commitment_.owner_id);
    if (owner == nullptr) {
        retain_reason = "owner_missing";
        return FutureA1Commitment{};
    }

    FutureA1Commitment retained = future_a1_commitment_;
    const bool awaiting_next_service_path =
        owner->path_gen + 1 == retained.owner_path_gen;
    if (awaiting_next_service_path &&
        owner->mission_phase == MissionPhase::TO_B) {
        if (owner->mode != VehicleMode::ACTIVE ||
            owner->leg_target != LegTargetKind::B_SLOT ||
            owner->track.empty()) {
            retain_reason = "future_owner_to_b_invalid";
            return FutureA1Commitment{};
        }
        retain_reason = "future_owner_locked_to_b";
        return retained;
    }
    if (awaiting_next_service_path &&
        owner->mission_phase == MissionPhase::UNLOAD_DWELL) {
        if (owner->mode != VehicleMode::DWELL) {
            retain_reason = "future_owner_unload_invalid";
            return FutureA1Commitment{};
        }
        retain_reason = "future_owner_locked_unload";
        return retained;
    }
    if (owner->mission_phase == MissionPhase::TO_A1) {
        if (owner->mode != VehicleMode::ACTIVE ||
            owner->leg_target != LegTargetKind::A1 ||
            owner->path_gen != retained.owner_path_gen ||
            !owner->pending_dropoff_valid ||
            owner->pending_dropoff_track.empty()) {
            retain_reason = "to_a1_service_invalid";
            return FutureA1Commitment{};
        }
        const auto prediction = summary.candidates.find(owner->id);
        if (prediction != summary.candidates.end()) {
            retained.predicted_a1_arrival_time =
                prediction->second.arrival_time;
            retained.predicted_to_b_time = prediction->second.to_b_time;
        }
        retain_reason = "service_owner_locked_to_a1";
        return retained;
    }

    if (owner->mission_phase == MissionPhase::PICKUP_DWELL) {
        if (owner->mode != VehicleMode::DWELL ||
            owner->path_gen != retained.owner_path_gen ||
            !owner->pending_dropoff_valid ||
            owner->pending_dropoff_track.empty()) {
            retain_reason = "pickup_service_invalid";
            return FutureA1Commitment{};
        }
        retained.predicted_a1_arrival_time = 0.0;
        retained.predicted_to_b_time = std::max(0.0, owner->dwell_remaining);
        retain_reason = "service_owner_locked_pickup";
        return retained;
    }

    const auto activeClusterCount = [&]() {
        size_t count = 0;
        for (const auto& entry : departure_cluster_commitments_) {
            if (entry.second.active && entry.second.owner_id == owner->id) {
                ++count;
            }
        }
        return count;
    };
    if (owner->mission_phase == MissionPhase::UNLOAD_DWELL &&
        owner->path_gen == retained.owner_path_gen &&
        activeClusterCount() == 0) {
        retain_reason = "departure_resource_clear";
        return FutureA1Commitment{};
    }

    if (owner->mission_phase != MissionPhase::TO_B ||
        owner->mode != VehicleMode::ACTIVE ||
        owner->leg_target != LegTargetKind::B_SLOT || !owner->loaded ||
        owner->a1_departure_priority_until_s <= 1e-9) {
        retain_reason = "service_phase_invalid";
        return FutureA1Commitment{};
    }

    const bool same_generation = owner->path_gen == retained.owner_path_gen;
    const bool prepared_handoff =
        owner->path_gen == retained.owner_path_gen + 1;
    if (!same_generation && !prepared_handoff) {
        retain_reason = "unexplained_path_gen_change";
        return FutureA1Commitment{};
    }
    if (activeClusterCount() == 0) {
        retain_reason = "departure_resource_clear";
        return FutureA1Commitment{};
    }

    retained.owner_path_gen = owner->path_gen;
    retained.predicted_a1_arrival_time = 0.0;
    retained.predicted_to_b_time = 0.0;
    retain_reason = "service_owner_locked_active_cluster";
    return retained;
}

void A1Coordinator::refreshPlanningContext(
    const std::vector<VehicleAgent>& vehicles, double horizon, double now,
    const ArrivalKinematics& kinematics) {
    // Retain the same read-only allocator-cache lookup used for arrival
    // prediction so UNLOAD_DWELL waiters can preview their already-built
    // next B->A1 service without generating another path.
    pickup_leg_track_ = kinematics.pickup_leg_track;
    const FutureA1Commitment previous = future_a1_commitment_;
    const ArrivalSummary arrivals =
        predictA1Arrivals(vehicles, horizon, kinematics);
    std::string change_reason;
    FutureA1Commitment commitment =
        retainLockedFutureA1Owner(vehicles, arrivals, change_reason);
    // Preserve the old rolling behavior: invalidating an existing owner does
    // not select a replacement until the next rolling refresh.
    if (!previous.valid()) {
        commitment = selectFutureA1Owner(vehicles, arrivals);
        change_reason = commitment.valid() ? "service_owner_selected"
                                           : "no_candidate";
    }
    setFutureA1Commitment(commitment);
    logFutureA1Transition(vehicles, previous, commitment, arrivals,
                          change_reason,
                          now, horizon);
}

void A1Coordinator::logFutureA1Transition(
    const std::vector<VehicleAgent>& vehicles,
    const FutureA1Commitment& previous,
    const FutureA1Commitment& current,
    const ArrivalSummary& summary, const std::string& change_reason,
    double now, double horizon) {
    const bool same_owner = previous.valid() && current.valid() &&
                            previous.owner_id == current.owner_id;
    std::string event;
    if (same_owner) event = "HOLD";
    else if (!previous.valid() && current.valid()) event = "CREATE";
    else if (previous.valid() && !current.valid()) {
        event = change_reason == "departure_resource_clear"
                    ? "RELEASE" : "INVALIDATE";
    } else if (previous.valid() && current.valid()) event = "CHANGE";
    else event = "HOLD";

    if (event == "CREATE") {
        ++service_metrics_.creates;
        service_metrics_.active_since = now;
    } else if (event == "HOLD" && current.valid()) {
        ++service_metrics_.holds;
    } else if (event == "CHANGE") {
        ++service_metrics_.changes;
        ++service_metrics_.arrival_ranking_preemptions;
    } else if (event == "RELEASE" || event == "INVALIDATE") {
        const double duration = service_metrics_.active_since >= 0.0
            ? now - service_metrics_.active_since : 0.0;
        service_metrics_.max_duration = std::max(
            service_metrics_.max_duration, duration);
        service_metrics_.active_since = -1.0;
        if (event == "RELEASE") ++service_metrics_.releases;
        else ++service_metrics_.invalidates;
    }

    const int observed_owner = current.valid()
        ? current.owner_id : previous.owner_id;
    const VehicleAgent* logged_owner = agentById(vehicles, observed_owner);
    const char* owner_phase = logged_owner == nullptr
        ? "MISSING" : missionPhaseName(logged_owner->mission_phase);
    int faster_candidate_id = -1;
    const auto owner_prediction = summary.candidates.find(observed_owner);
    if (owner_prediction != summary.candidates.end()) {
        const double tie_window = std::max(0.02, cfg_.prediction_step);
        for (const auto& item : summary.candidates) {
            if (item.first != observed_owner &&
                item.second.arrival_time + tie_window <
                    owner_prediction->second.arrival_time) {
                faster_candidate_id = item.first;
                break;
            }
        }
    }
    if (same_owner && faster_candidate_id >= 0) {
        ++service_metrics_.faster_candidate_observations;
    }

    if (!coord_log_sink_) return;
    std::ostringstream candidates;
    candidates << "[";
    bool first = true;
    for (const auto& item : summary.candidates) {
        if (!first) candidates << ",";
        first = false;
        candidates << "V" << item.first << ":" << std::fixed
                   << std::setprecision(2) << item.second.arrival_time << "s";
    }
    candidates << "]";
    std::ostringstream excluded;
    excluded << "[";
    first = true;
    for (const auto& item : summary.excluded) {
        if (!first) excluded << ",";
        first = false;
        excluded << "V" << item.first << ":" << item.second;
    }
    excluded << "]";

    std::ostringstream line;
    line << std::fixed << std::setprecision(2)
         << "[FUTURE_A1] time=" << readableSimTime(now)
         << " event=" << event;
    if (current.valid()) {
        line << " old="
             << (previous.valid() ? "V" + std::to_string(previous.owner_id)
                                  : "none")
             << " owner=V" << current.owner_id
             << " arrival=" << current.predicted_a1_arrival_time << "s"
             << " to_b=" << current.predicted_to_b_time << "s"
             << " path_gen=" << current.owner_path_gen
             << " phase=" << owner_phase;
    } else if (previous.valid()) {
        line << " owner=V" << previous.owner_id
             << " path_gen=" << previous.owner_path_gen
             << " phase=" << owner_phase;
    } else {
        line << " old=none owner=none";
    }
    line << " horizon=" << horizon << "s candidates=" << candidates.str()
         << " excluded=" << excluded.str();
    if (faster_candidate_id >= 0) {
        line << " faster_candidate=V" << faster_candidate_id;
    } else {
        line << " faster_candidate=none";
    }
    line << " change_reason=" << change_reason;
    coord_log_sink_(line.str());
}

A1Coordinator::Snapshot A1Coordinator::snapshot() const {
    return Snapshot{departure_cluster_commitments_};
}

void A1Coordinator::restore(const Snapshot& snapshot,
                            bool clear_intrusion_corrections) {
    for (const auto& current : departure_cluster_commitments_) {
        const auto incoming = snapshot.departure_clusters.find(current.first);
        if (current.second.active &&
            (incoming == snapshot.departure_clusters.end() ||
             !incoming->second.active)) {
            logDepartureCluster(coord_log_sink_, "RELEASE", "snapshot_restore",
                                current.second, -1.0, -1.0);
        }
    }
    for (const auto& incoming : snapshot.departure_clusters) {
        const auto current = departure_cluster_commitments_.find(incoming.first);
        if (incoming.second.active &&
            (current == departure_cluster_commitments_.end() ||
             !current->second.active)) {
            logDepartureCluster(coord_log_sink_, "CREATE", "snapshot_restore",
                                incoming.second, -1.0, -1.0);
        } else if (incoming.second.active &&
                   current != departure_cluster_commitments_.end() &&
                   !sameDepartureCluster(current->second, incoming.second)) {
            logDepartureCluster(coord_log_sink_, "HOLD", "snapshot_restore",
                                incoming.second, -1.0, -1.0);
        }
    }
    departure_cluster_commitments_ = snapshot.departure_clusters;
    // Intrusion correction is live, derived motion state. A rollout snapshot
    // must never install or complete it for the real executor.
    if (clear_intrusion_corrections) intrusion_corrections_.clear();
}

A1Coordinator::FutureA1ZoneSelection
A1Coordinator::selectFutureA1ProtectedZones(
    const std::vector<ConflictZone>& canonical_zones,
    bool preview_is_lo, double protected_until,
    double other_path_s) const {
    FutureA1ZoneSelection selection;
    selection.normalized_zones.reserve(canonical_zones.size());
    std::vector<FutureA1ConflictInterval> intervals;
    intervals.reserve(canonical_zones.size());
    for (const ConflictZone& canonical : canonical_zones) {
        ConflictZone normalized = canonical;
        if (!preview_is_lo) {
            std::swap(normalized.s_self_enter, normalized.s_other_enter);
            std::swap(normalized.s_self_exit, normalized.s_other_exit);
        }
        selection.normalized_zones.push_back(normalized);
        intervals.push_back(FutureA1ConflictInterval{
            normalized.s_self_enter, normalized.s_self_exit,
            normalized.s_other_enter, normalized.s_other_exit});
    }
    const FutureA1ProtectedCluster cluster = selectFutureA1ProtectedCluster(
        intervals, protected_until, other_path_s);
    selection.seed_indices = cluster.seed_indices;
    selection.protected_indices = cluster.protected_indices;
    selection.other_already_inside = cluster.other_already_inside;
    if (cluster.upstream_other_enter) {
        for (size_t index : selection.protected_indices) {
            if (std::abs(selection.normalized_zones[index].s_other_enter -
                         *cluster.upstream_other_enter) <= 1e-9) {
                selection.upstream_index = static_cast<int>(index);
                break;
            }
        }
    }
    return selection;
}

int A1Coordinator::departureClusterOwnerForPair(
    const VehicleAgent& a, const VehicleAgent& b) const {
    const std::pair<int, int> key{std::min(a.id, b.id), std::max(a.id, b.id)};
    const auto it = departure_cluster_commitments_.find(key);
    if (it == departure_cluster_commitments_.end() || !it->second.active) {
        return -1;
    }
    const DepartureClusterCommitment& commitment = it->second;
    const VehicleAgent* owner = a.id == commitment.owner_id ? &a :
                                b.id == commitment.owner_id ? &b : nullptr;
    const VehicleAgent* other = a.id == commitment.other_id ? &a :
                                b.id == commitment.other_id ? &b : nullptr;
    const bool owner_generation_matches = owner != nullptr &&
        departureClusterOwnerGenerationMatches(
            commitment.transaction_owner_path_gen,
            commitment.owner_path_gen, owner->path_gen);
    const bool owner_on_service_leg = owner_generation_matches &&
        owner->path_gen == commitment.transaction_owner_path_gen &&
        (owner->mission_phase == MissionPhase::TO_A1 ||
         owner->mission_phase == MissionPhase::PICKUP_DWELL);
    const bool owner_on_frozen_departure = owner_generation_matches &&
        owner->path_gen == commitment.owner_path_gen &&
        (owner->mission_phase == MissionPhase::TO_B ||
         owner->mission_phase == MissionPhase::UNLOAD_DWELL);
    if (owner == nullptr || other == nullptr ||
        (!owner_on_service_leg && !owner_on_frozen_departure) ||
        other->path_gen != commitment.other_path_gen) {
        return -1;
    }
    return owner->id;
}

int A1Coordinator::futureA1OwnerForPair(const VehicleAgent& a,
                                        const VehicleAgent& b) const {
    if (!future_a1_commitment_.valid() ||
        a.mission_phase != MissionPhase::TO_A1 ||
        b.mission_phase != MissionPhase::TO_A1) {
        return -1;
    }
    const VehicleAgent* owner = nullptr;
    const VehicleAgent* other = nullptr;
    if (a.id == future_a1_commitment_.owner_id &&
        a.path_gen == future_a1_commitment_.owner_path_gen) {
        owner = &a;
        other = &b;
    } else if (b.id == future_a1_commitment_.owner_id &&
               b.path_gen == future_a1_commitment_.owner_path_gen) {
        owner = &b;
        other = &a;
    }
    if (owner == nullptr || !owner->pending_dropoff_valid ||
        owner->pending_dropoff_track.empty() ||
        owner->a1_departure_priority_until_s <= 1e-9 ||
        !dependencies_.compute_full_conflict_zones ||
        !dependencies_.current_conflict_zones) {
        return -1;
    }

    VehicleAgent exit_preview = *owner;
    exit_preview.track = owner->pending_dropoff_track;
    exit_preview.path_s = 0.0;
    exit_preview.path_gen = owner->path_gen + 1;
    exit_preview.mode = VehicleMode::ACTIVE;
    exit_preview.mission_phase = MissionPhase::TO_B;

    const bool preview_is_lo = exit_preview.id < other->id;
    const VehicleAgent& lo = preview_is_lo ? exit_preview : *other;
    const VehicleAgent& hi = preview_is_lo ? *other : exit_preview;
    const std::pair<int, int> cache_key{lo.id, hi.id};
    ConflictCacheEntry& cache = future_a1_conflict_cache_[cache_key];
    if (cache.gen_lo != lo.path_gen || cache.gen_hi != hi.path_gen) {
        cache.blocks = dependencies_.compute_full_conflict_zones(lo, hi);
        cache.gen_lo = lo.path_gen;
        cache.gen_hi = hi.path_gen;
    }
    const FutureA1ZoneSelection future_zones =
        selectFutureA1ProtectedZones(
            cache.blocks, preview_is_lo,
            owner->a1_departure_priority_until_s, other->path_s);
    if (future_zones.protected_indices.empty() ||
        future_zones.other_already_inside) {
        return -1;
    }

    const bool owner_is_lo = owner->id < other->id;
    const VehicleAgent& ordinary_lo = owner_is_lo ? *owner : *other;
    const VehicleAgent& ordinary_hi = owner_is_lo ? *other : *owner;
    const auto ordinary_blocks =
        dependencies_.current_conflict_zones(ordinary_lo, ordinary_hi);
    for (const ConflictZone& canonical : ordinary_blocks) {
        const double owner_exit = owner_is_lo
            ? canonical.s_self_exit : canonical.s_other_exit;
        const double other_enter = owner_is_lo
            ? canonical.s_other_enter : canonical.s_self_enter;
        const double other_exit = owner_is_lo
            ? canonical.s_other_exit : canonical.s_self_exit;
        if (owner->path_s > owner_exit + 1e-9 ||
            other->path_s > other_exit + 1e-9) {
            continue;
        }
        if (other->path_s > other_enter + 1e-9) return -1;
    }
    return owner->id;
}

A1Coordinator::PairAuthority A1Coordinator::authorityForPair(
    const VehicleAgent& a, const VehicleAgent& b) const {
    PairAuthority result;
    result.departure_owner_id = departureClusterOwnerForPair(a, b);
    result.future_owner_id = futureA1OwnerForPair(a, b);
    return result;
}

std::optional<A1Coordinator::WaiterStopConstraint>
A1Coordinator::waiterStopConstraint(const VehicleAgent& waiter) const {
    if (!waiter.active() || waiter.mission_phase != MissionPhase::TO_A1) {
        return std::nullopt;
    }
    std::optional<WaiterStopConstraint> selected;
    for (const auto& entry : departure_cluster_commitments_) {
        const DepartureClusterCommitment& commitment = entry.second;
        if (!commitment.active || commitment.other_id != waiter.id ||
            commitment.other_path_gen != waiter.path_gen) {
            continue;
        }
        WaiterStopConstraint candidate;
        candidate.owner_id = commitment.owner_id;
        candidate.waiter_id = commitment.other_id;
        candidate.waiter_path_gen = commitment.other_path_gen;
        candidate.waiter_stop_s = commitment.waiter_stop_s;
        if (!selected ||
            candidate.waiter_stop_s < selected->waiter_stop_s - 1e-9 ||
            (std::abs(candidate.waiter_stop_s - selected->waiter_stop_s) <=
                 1e-9 &&
             candidate.owner_id < selected->owner_id)) {
            selected = candidate;
        }
    }
    return selected;
}

A1Coordinator::A1LaunchAdmission A1Coordinator::checkA1LaunchAdmission(
    const VehicleAgent& service_owner,
    const VehicleAgent& launch_candidate) const {
    A1LaunchAdmission result;
    if (!launch_candidate.active() || launch_candidate.track.empty() ||
        launch_candidate.mission_phase != MissionPhase::TO_A1 ||
        service_owner.id == launch_candidate.id ||
        service_owner.a1_departure_priority_until_s <= 1e-9 ||
        !dependencies_.compute_full_conflict_zones) {
        return result;
    }

    VehicleAgent exit = service_owner;
    if ((service_owner.mission_phase == MissionPhase::TO_A1 ||
         service_owner.mission_phase == MissionPhase::PICKUP_DWELL) &&
        service_owner.pending_dropoff_valid &&
        !service_owner.pending_dropoff_track.empty()) {
        exit.track = service_owner.pending_dropoff_track;
        exit.path_s = 0.0;
        exit.path_gen = service_owner.path_gen + 1;
        exit.mode = VehicleMode::ACTIVE;
        exit.mission_phase = MissionPhase::TO_B;
        result.owner_uses_pending_preview = true;
    } else if (service_owner.active() &&
               service_owner.mission_phase == MissionPhase::TO_B &&
               service_owner.a1_departure_committed &&
               !service_owner.track.empty() &&
               service_owner.path_s <
                   service_owner.a1_departure_priority_until_s - 1e-9) {
        // Use the actual TO_B path and current progress.
    } else {
        return result;
    }

    const bool exit_is_lo = exit.id < launch_candidate.id;
    const VehicleAgent& lo = exit_is_lo ? exit : launch_candidate;
    const VehicleAgent& hi = exit_is_lo ? launch_candidate : exit;
    const auto blocks = dependencies_.compute_full_conflict_zones(lo, hi);
    const FutureA1ZoneSelection selected = selectFutureA1ProtectedZones(
        blocks, exit_is_lo, service_owner.a1_departure_priority_until_s,
        launch_candidate.path_s);
    if (isB0ToB9Source(launch_candidate)) {
        for (size_t index : selected.protected_indices) {
            const ConflictZone& zone = selected.normalized_zones[index];
            if (!result.owner_uses_pending_preview &&
                service_owner.path_s > zone.s_self_exit + 1e-9) {
                continue;
            }
            ++result.protected_zone_count;
        }
        if (result.protected_zone_count > 0) {
            result.departure_resource_conflict = true;
            result.source_slot_hold = true;
            return result;
        }
    }
    if (selected.upstream_index >= 0) {
        const ConflictZone& upstream = selected.normalized_zones[
            static_cast<size_t>(selected.upstream_index)];
        const std::optional<double> stop_s = futureA1StopS(
            upstream.s_other_enter, cfg_.a1_stop_margin);
        if (stop_s) {
            result.waiter_stop_s = *stop_s;
            // A parked vehicle must be able to clear its source-slot sweep
            // before it can stop at the frozen A1 departure boundary.
            result.spatial_stop_launch_infeasible =
                *stop_s <= launch_candidate.slot_departure_clear_s + 1e-9;
            if (result.spatial_stop_launch_infeasible) {
                result.departure_resource_conflict = true;
                result.protected_zone_count =
                    std::max<size_t>(1, selected.protected_indices.size());
                return result;
            }
        }
    }
    const bool candidate_has_cleared_slot =
        launch_candidate.path_s + 1e-9 >=
        launch_candidate.slot_departure_clear_s;
    result.actual_occupancy_priority =
        selected.other_already_inside && candidate_has_cleared_slot;
    if (result.actual_occupancy_priority) return result;

    for (size_t index : selected.protected_indices) {
        const ConflictZone& zone = selected.normalized_zones[index];
        if (zone.s_other_enter >
            launch_candidate.slot_departure_clear_s + 1e-9) {
            continue;
        }
        if (!result.owner_uses_pending_preview &&
            service_owner.path_s > zone.s_self_exit + 1e-9) {
            continue;
        }
        ++result.protected_zone_count;
    }
    result.departure_resource_conflict = result.protected_zone_count > 0;
    return result;
}

void A1Coordinator::refreshDepartureClusterCommitments(
    std::vector<VehicleAgent>& vehicles) {
    auto eraseWithEvent = [&](auto it, const char* event,
                              const char* reason) {
        VehicleAgent* owner = agentById(vehicles, it->second.owner_id);
        VehicleAgent* other = agentById(vehicles, it->second.other_id);
        logDepartureCluster(coord_log_sink_, event, reason, it->second,
                            owner ? owner->path_s : -1.0,
                            other ? other->path_s : -1.0);
        return departure_cluster_commitments_.erase(it);
    };

    std::map<std::pair<int, int>, double> transaction_release_s;
    for (const auto& entry : departure_cluster_commitments_) {
        const DepartureClusterCommitment& commitment = entry.second;
        if (!commitment.active) continue;
        const std::pair<int, int> transaction{
            commitment.owner_id, commitment.owner_path_gen};
        transaction_release_s[transaction] = std::max(
            transaction_release_s[transaction],
            commitment.owner_release_exit_s);
    }
    for (const auto& transaction : transaction_release_s) {
        VehicleAgent* owner = agentById(vehicles, transaction.first.first);
        if (owner == nullptr || owner->path_gen != transaction.first.second ||
            (owner->mission_phase != MissionPhase::TO_B &&
             owner->mission_phase != MissionPhase::UNLOAD_DWELL) ||
            !departureClusterCleared(owner->path_s, transaction.second)) {
            continue;
        }
        for (auto it = departure_cluster_commitments_.begin();
             it != departure_cluster_commitments_.end();) {
            const DepartureClusterCommitment& commitment = it->second;
            if (commitment.active &&
                commitment.owner_id == transaction.first.first &&
                commitment.owner_path_gen == transaction.first.second) {
                it = eraseWithEvent(
                    it, "RELEASE", "owner_cleared_frozen_transaction");
            } else {
                ++it;
            }
        }
    }

    for (auto it = departure_cluster_commitments_.begin();
         it != departure_cluster_commitments_.end();) {
        DepartureClusterCommitment& commitment = it->second;
        VehicleAgent* owner = agentById(vehicles, commitment.owner_id);
        VehicleAgent* other = agentById(vehicles, commitment.other_id);
        if (owner == nullptr || other == nullptr) {
            it = eraseWithEvent(it, "INVALIDATE", "vehicle_missing");
            continue;
        }
        if (!commitment.active) {
            commitment.active = true;
            logDepartureCluster(coord_log_sink_, "CREATE",
                                "legacy_stage_promoted", commitment,
                                owner->path_s, other->path_s);
        }
        ++it;
    }
    // Deliberately no TO_B deterministic rebuild.
}

void A1Coordinator::enforceFutureA1Admission(
    std::vector<VehicleAgent>& vehicles, double dt,
    const ActionRequest& request_action) {
    if (!future_a1_commitment_.valid() ||
        !dependencies_.compute_full_conflict_zones) {
        return;
    }
    VehicleAgent* owner = agentById(vehicles, future_a1_commitment_.owner_id);
    if (owner == nullptr ||
        owner->path_gen != future_a1_commitment_.owner_path_gen ||
        !owner->pending_dropoff_valid ||
        owner->pending_dropoff_track.empty() ||
        (owner->mission_phase != MissionPhase::TO_A1 &&
         owner->mission_phase != MissionPhase::PICKUP_DWELL)) {
        return;
    }

    VehicleAgent exit_preview = *owner;
    exit_preview.track = owner->pending_dropoff_track;
    exit_preview.path_s = 0.0;
    exit_preview.path_gen = owner->path_gen + 1;
    exit_preview.mode = VehicleMode::ACTIVE;
    exit_preview.mission_phase = MissionPhase::TO_B;
    const double protected_until = owner->a1_departure_priority_until_s;
    if (protected_until <= 1e-9) return;

    for (VehicleAgent& other : vehicles) {
        if (other.id == owner->id) {
            continue;
        }
        const bool active_waiter = other.active() &&
            other.mission_phase == MissionPhase::TO_A1 &&
            !other.track.empty();
        const bool dwell_waiter = other.mode == VehicleMode::DWELL &&
            other.mission_phase == MissionPhase::UNLOAD_DWELL;
        if (!active_waiter && !dwell_waiter) continue;

        VehicleAgent waiter_preview = other;
        if (dwell_waiter) {
            PathTrack next_pickup;
            if (!pickup_leg_track_ || other.current_slot < 0 ||
                !pickup_leg_track_(other.current_slot, next_pickup) ||
                next_pickup.empty()) {
                continue;
            }
            waiter_preview.track = std::move(next_pickup);
            waiter_preview.path_s = 0.0;
            waiter_preview.path_gen = other.path_gen + 1;
            waiter_preview.current_speed = 0.0;
            waiter_preview.mode = VehicleMode::ACTIVE;
            waiter_preview.mission_phase = MissionPhase::TO_A1;
            waiter_preview.leg_target = LegTargetKind::A1;
        }
        const double waiter_path_s = dwell_waiter ? 0.0 : other.path_s;
        const bool preview_is_lo = exit_preview.id < waiter_preview.id;
        const VehicleAgent& lo = preview_is_lo ? exit_preview : waiter_preview;
        const VehicleAgent& hi = preview_is_lo ? waiter_preview : exit_preview;
        const std::pair<int, int> key{lo.id, hi.id};
        ConflictCacheEntry& cache = future_a1_conflict_cache_[key];
        if (cache.gen_lo != lo.path_gen || cache.gen_hi != hi.path_gen) {
            cache.blocks = dependencies_.compute_full_conflict_zones(lo, hi);
            cache.gen_lo = lo.path_gen;
            cache.gen_hi = hi.path_gen;
        }
        const FutureA1ZoneSelection future_zones =
            selectFutureA1ProtectedZones(cache.blocks, preview_is_lo,
                                         protected_until, waiter_path_s);
        std::optional<double> future_exit_enter_s;
        ConflictZone future_selected;
        if (future_zones.upstream_index >= 0) {
            future_selected = future_zones.normalized_zones[
                static_cast<size_t>(future_zones.upstream_index)];
            future_exit_enter_s = future_selected.s_other_enter;
        }

        if (!future_exit_enter_s) continue;

        const bool already_inside = future_zones.other_already_inside;
        const double selected_stop_boundary_s = *future_exit_enter_s;
        const std::optional<double> selected_stop_s = futureA1StopS(
            future_exit_enter_s, cfg_.a1_stop_margin);
        if (!selected_stop_s) continue;

        const std::pair<int, int> cluster_key{
            std::min(owner->id, other.id), std::max(owner->id, other.id)};
        auto existing_cluster =
            departure_cluster_commitments_.find(cluster_key);
        if (existing_cluster == departure_cluster_commitments_.end() ||
            !existing_cluster->second.active) {
            DepartureClusterCommitment staged;
            staged.owner_id = owner->id;
            staged.transaction_owner_path_gen = owner->path_gen;
            staged.owner_path_gen = exit_preview.path_gen;
            staged.other_id = other.id;
            staged.other_path_gen = waiter_preview.path_gen;
            staged.frozen_owner_track = exit_preview.track;
            staged.frozen_waiter_track = waiter_preview.track;
            staged.seed_indices = future_zones.seed_indices;
            staged.cluster_indices = future_zones.protected_indices;
            staged.waiter_stop_boundary_s = selected_stop_boundary_s;
            staged.waiter_stop_s = *selected_stop_s;
            staged.active = true;
            staged.handed_off_from_future = true;
            for (size_t index : future_zones.protected_indices) {
                const ConflictZone& zone =
                    future_zones.normalized_zones[index];
                staged.intervals.push_back(FutureA1ConflictInterval{
                    zone.s_self_enter, zone.s_self_exit,
                    zone.s_other_enter, zone.s_other_exit});
                staged.diagnostic_protected_zone_aabbs.push_back({
                    zone.aabb_min_x, zone.aabb_min_y,
                    zone.aabb_max_x, zone.aabb_max_y,
                    zone.aabb_valid});
                staged.owner_release_exit_s = std::max(
                    staged.owner_release_exit_s, zone.s_self_exit);
                staged.other_release_exit_s = std::max(
                    staged.other_release_exit_s, zone.s_other_exit);
            }
            logDepartureCluster(coord_log_sink_, "CREATE",
                                "frozen_transaction", staged,
                                owner->path_s, waiter_path_s);
            departure_cluster_commitments_[cluster_key] = std::move(staged);
        }

        if (already_inside) continue;
        const double distance = *selected_stop_s - waiter_path_s;
        const double speed = dwell_waiter
            ? 0.0 : std::max(0.0, other.current_speed);
        const double stopping_distance =
            speed * speed / (2.0 * std::max(1e-6, cfg_.max_decel)) +
            speed * dt;
        if (distance > stopping_distance + 1e-9) continue;
        if (request_action) {
            request_action(other, VehicleAction::STOP,
                           "future_a1_exit_priority", owner->id);
        }
        const std::pair<int, int> log_key{owner->id, other.id};
        if (future_a1_admission_logged_.insert(log_key).second &&
            coord_log_sink_) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[FUTURE_A1_ADMISSION] owner=V" << owner->id
                 << " blocked=V" << other.id
                 << " reason=future_a1_exit_priority early_stop=true"
                 << " holder=V" << owner->id
                 << " conflict_zone=(" << future_selected.x << ","
                 << future_selected.y
                 << ") future_exit_enter_s=" << *future_exit_enter_s
                 << " selected_stop_boundary_s="
                 << selected_stop_boundary_s
                 << " stop_s=" << *selected_stop_s
                 << " other_s=" << waiter_path_s
                 << " waiter_phase=" << missionPhaseName(other.mission_phase)
                 << " already_inside=false";
            coord_log_sink_(line.str());
        }
    }
}

bool A1Coordinator::waiterPoseClearsFrozenClosure(
    const DepartureClusterCommitment& commitment,
    double waiter_s) const {
    if (commitment.frozen_waiter_track.empty() ||
        commitment.frozen_owner_track.empty() ||
        commitment.intervals.empty()) {
        return false;
    }
    const OBB waiter_body = makeBody(
        commitment.frozen_waiter_track.poseAtS(waiter_s), map_param_, 0.0);
    constexpr double kStep = 0.01;
    for (const FutureA1ConflictInterval& interval : commitment.intervals) {
        const double distance = std::max(
            0.0, interval.owner_exit - interval.owner_enter);
        const int count = std::max(
            1, static_cast<int>(std::ceil(distance / kStep)));
        for (int sample = 0; sample <= count; ++sample) {
            const double ratio = static_cast<double>(sample) / count;
            const double owner_s = interval.owner_enter + distance * ratio;
            const OBB owner_body = makeBody(
                commitment.frozen_owner_track.poseAtS(owner_s),
                map_param_, 0.0);
            if (overlaps(waiter_body, owner_body)) return false;
        }
    }
    return true;
}

bool A1Coordinator::waiterRetreatSweepClear(
    const DepartureClusterCommitment& commitment,
    const VehicleAgent& waiter,
    const std::vector<VehicleAgent>& vehicles,
    double target_s, int* blocker_id) const {
    if (blocker_id != nullptr) *blocker_id = -1;
    if (waiter.track.empty() || target_s > waiter.path_s + 1e-9) {
        return false;
    }
    constexpr double kStep = 0.01;
    const double distance = std::max(0.0, waiter.path_s - target_s);
    const int count = std::max(
        1, static_cast<int>(std::ceil(distance / kStep)));
    for (int sample = 0; sample <= count; ++sample) {
        const double ratio = static_cast<double>(sample) / count;
        const double waiter_s = waiter.path_s - distance * ratio;
        const OBB waiter_body = makeBody(
            commitment.frozen_waiter_track.poseAtS(waiter_s),
            map_param_, 0.0);
        for (const VehicleAgent& other : vehicles) {
            if (other.id == waiter.id || other.track.empty() ||
                other.mode == VehicleMode::NEED_TASK) {
                continue;
            }
            const double other_s = other.mode == VehicleMode::DWELL
                ? other.track.length() : other.path_s;
            const OBB other_body = makeBody(
                other.track.poseAtS(other_s), map_param_, 0.0);
            if (overlaps(waiter_body, other_body)) {
                if (blocker_id != nullptr) *blocker_id = other.id;
                return false;
            }
        }
    }
    return true;
}

void A1Coordinator::refreshIntrusionCorrections(
    const std::vector<VehicleAgent>& vehicles) {
    std::map<int, IntrusionCorrection> refreshed;
    constexpr double kSearchStep = 0.01;
    constexpr double kTargetTolerance = 0.005;
    constexpr double kStoppedSpeed = 1e-3;

    for (const auto& entry : departure_cluster_commitments_) {
        const DepartureClusterCommitment& commitment = entry.second;
        if (!commitment.active) continue;
        const VehicleAgent* waiter = agentById(vehicles,
                                                commitment.other_id);
        if (waiter == nullptr || waiter->mode != VehicleMode::ACTIVE ||
            waiter->mission_phase != MissionPhase::TO_A1 ||
            waiter->path_gen != commitment.other_path_gen ||
            waiter->track.empty()) {
            continue;
        }
        const bool crossed = waiter->path_s >
            commitment.waiter_stop_boundary_s + 1e-9;
        const bool inside = futureA1OtherInsideCluster(
            commitment.intervals, waiter->path_s);
        const auto previous = intrusion_corrections_.find(waiter->id);
        const bool correction_active =
            previous != intrusion_corrections_.end() &&
            previous->second.waiter_path_gen == waiter->path_gen;
        if (!crossed && !inside && !correction_active) continue;
        
        //检验回退库位是否是B0-B9，若是，则退回原库位。否则在原来计算的基础stop_s上多退一个阈值，防止车身入侵
        const bool return_to_source_slot = isB0ToB9Source(*waiter);
        double target_s = return_to_source_slot ? 0.0   : std::max(0.0, commitment.waiter_stop_s -  kIntrusionRetreatClearance);
        while (!return_to_source_slot && target_s > 1e-9 &&
               !waiterPoseClearsFrozenClosure(commitment, target_s)) {
            target_s = std::max(0.0, target_s - kSearchStep);
        }

        IntrusionCorrection correction;
        correction.owner_id = commitment.owner_id;
        correction.waiter_id = commitment.other_id;
        correction.waiter_path_gen = commitment.other_path_gen;
        correction.target_s = target_s;
        const bool target_clear =
            waiterPoseClearsFrozenClosure(commitment, target_s);
        const bool at_target = waiter->path_s <=
            target_s + kTargetTolerance;
        const bool actual_clear = waiterPoseClearsFrozenClosure(
            commitment, waiter->path_s);
        if (at_target && actual_clear) {
            if (waiter->current_speed <= kStoppedSpeed) continue;
            correction.motion = IntrusionCorrectionMotion::HOLD;
            correction.reason = "a1_intrusion_target_braking";
        } else if (!target_clear) {
            correction.motion = IntrusionCorrectionMotion::HOLD;
            correction.reason = "a1_intrusion_no_clear_target";
        } else if (!waiterRetreatSweepClear(
                       commitment, *waiter, vehicles, target_s,
                       &correction.blocker_id)) {
            correction.motion = IntrusionCorrectionMotion::HOLD;
            correction.reason = "a1_intrusion_retreat_sweep_blocked";
        } else {
            correction.motion = IntrusionCorrectionMotion::RETREAT;
            correction.reason = "a1_intrusion_retreat";
        }
        refreshed[waiter->id] = std::move(correction);
    }
    intrusion_corrections_.swap(refreshed);
}

const A1Coordinator::IntrusionCorrection*
A1Coordinator::intrusionCorrectionFor(int vehicle_id) const {
    const auto it = intrusion_corrections_.find(vehicle_id);
    return it == intrusion_corrections_.end() ? nullptr : &it->second;
}

void A1Coordinator::holdIntrusionCorrection(
    int vehicle_id, const std::string& reason, int blocker_id) {
    const auto it = intrusion_corrections_.find(vehicle_id);
    if (it == intrusion_corrections_.end()) return;
    it->second.motion = IntrusionCorrectionMotion::HOLD;
    it->second.reason = reason;
    it->second.blocker_id = blocker_id;
}

void A1Coordinator::enforceDepartureClusterCommitments(
    std::vector<VehicleAgent>& vehicles, double dt,
    const ActionRequest& request_action) {
    refreshIntrusionCorrections(vehicles);
    for (auto& entry : departure_cluster_commitments_) {
        DepartureClusterCommitment& commitment = entry.second;
        if (!commitment.active) continue;
        VehicleAgent* owner = agentById(vehicles, commitment.owner_id);
        VehicleAgent* other = agentById(vehicles, commitment.other_id);
        if (owner == nullptr || other == nullptr) continue;
        const bool dwell_waiter = other->mode == VehicleMode::DWELL &&
            other->mission_phase == MissionPhase::UNLOAD_DWELL;
        const bool active_waiter = other->mode == VehicleMode::ACTIVE &&
            other->mission_phase == MissionPhase::TO_A1;
        const bool waiter_identity_changed = !dwell_waiter &&
            (!active_waiter ||
             other->path_gen != commitment.other_path_gen);
        const double waiter_path_s = dwell_waiter ? 0.0 : other->path_s;
        const bool waiter_crossed_boundary =
            waiter_path_s > commitment.waiter_stop_boundary_s + 1e-9;
        const bool waiter_inside_closure = futureA1OtherInsideCluster(
            commitment.intervals, waiter_path_s);
        if (waiter_identity_changed) {
            if (request_action) {
                request_action(*owner, VehicleAction::STOP,
                               "a1_admission_invariant_violation", other->id);
                request_action(*other, VehicleAction::STOP,
                               "a1_admission_invariant_violation", owner->id);
            }
            if (!commitment.invariant_violation_logged) {
                logAdmissionInvariantViolation(
                    coord_log_sink_, commitment, *owner, *other,
                    "waiter_transaction_identity_changed");
                commitment.invariant_violation_logged = true;
            }
            continue;
        }
        if (waiter_crossed_boundary || waiter_inside_closure) {
            const IntrusionCorrection* correction =
                intrusionCorrectionFor(other->id);
            if (request_action) {
                request_action(*other, VehicleAction::STOP,
                               correction != nullptr
                                   ? correction->reason
                                   : "a1_intrusion_correction_unavailable",
                               correction != nullptr
                                   ? correction->blocker_id : -1);
            }
            if (!commitment.intrusion_correction_logged &&
                coord_log_sink_) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(3)
                     << "[A1_INTRUSION_CORRECTION] owner=V" << owner->id
                     << " waiter=V" << other->id
                     << " waiter_s=" << other->path_s
                     << " stop_s=" << commitment.waiter_stop_s
                     << " target_s="
                     << (correction != nullptr ? correction->target_s : -1.0)
                     << " motion="
                     << (correction != nullptr &&
                                 correction->motion ==
                                     IntrusionCorrectionMotion::RETREAT
                             ? "RETREAT" : "HOLD")
                     << " blocker=V"
                     << (correction != nullptr
                             ? correction->blocker_id : -1)
                     << " reason="
                     << (correction != nullptr ? correction->reason
                                               : "unavailable");
                coord_log_sink_(line.str());
                commitment.intrusion_correction_logged = true;
            }
            continue;
        }
        const double distance = commitment.waiter_stop_s - waiter_path_s;
        const double speed = dwell_waiter
            ? 0.0 : std::max(0.0, other->current_speed);
        const double stopping_distance =
            speed * speed / (2.0 * std::max(1e-6, cfg_.max_decel)) +
            speed * dt;
        if (distance > stopping_distance + 1e-9) continue;
        if (request_action) {
            request_action(*other, VehicleAction::STOP,
                           "departure_cluster_priority", owner->id);
        }
        if (!commitment.hold_logged) {
            logDepartureCluster(coord_log_sink_, "HOLD",
                                "cluster_stop_boundary", commitment,
                                owner->path_s, waiter_path_s);
            commitment.hold_logged = true;
        }
    }
}

std::vector<A1Coordinator::DepartureTransactionIdentity>
A1Coordinator::departureTransactionIdentity() const {
    std::vector<DepartureTransactionIdentity> identity;
    identity.reserve(departure_cluster_commitments_.size());
    for (const auto& entry : departure_cluster_commitments_) {
        const auto& commitment = entry.second;
        identity.emplace_back(
            entry.first.first, entry.first.second, commitment.owner_id,
            commitment.transaction_owner_path_gen,
            commitment.owner_path_gen, commitment.other_id,
            commitment.other_path_gen, commitment.active);
    }
    return identity;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
