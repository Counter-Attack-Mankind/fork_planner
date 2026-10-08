#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace forklift_planner {
namespace multi_vehicle {

struct FutureA1RankedCandidate {
    int vehicle_id = -1;
    double arrival_time = -1.0;
};

// Arc-length projection of one future A1-exit conflict zone, normalized so
// owner_* always belongs to the prepared A1->B exit and other_* to the
// competing TO_A1 vehicle.  computeConflictZonesFull() already incorporates
// the configured OBB conflict margin when it builds these intervals.
struct FutureA1ConflictInterval {
    double owner_enter = 0.0;
    double owner_exit = 0.0;
    double other_enter = 0.0;
    double other_exit = 0.0;
};

struct FutureA1ProtectedCluster {
    std::vector<size_t> seed_indices;
    std::vector<size_t> protected_indices;
    std::optional<double> upstream_other_enter;
    bool other_already_inside = false;
};

inline bool futureA1IntervalsOverlapOrTouch(double lhs_enter,
                                             double lhs_exit,
                                             double rhs_enter,
                                             double rhs_exit) {
    return lhs_enter <= rhs_exit + 1e-9 &&
           rhs_enter <= lhs_exit + 1e-9;
}

// Start from the portion of the prepared exit covered by departure priority,
// then close transitively over conflict zones that overlap/touch on the other
// vehicle's path.  A zone already fully passed by the other vehicle is not a
// current admission constraint and cannot bridge two remaining zones.
inline FutureA1ProtectedCluster selectFutureA1ProtectedCluster(
    const std::vector<FutureA1ConflictInterval>& zones,
    double protected_until, double other_path_s) {
    FutureA1ProtectedCluster result;
    if (protected_until <= 1e-9) return result;

    std::vector<bool> available(zones.size(), false);
    std::vector<bool> selected(zones.size(), false);
    for (size_t i = 0; i < zones.size(); ++i) {
        const auto& zone = zones[i];
        if (other_path_s > zone.other_exit + 1e-9) continue;
        available[i] = true;
        if (zone.owner_enter < protected_until - 1e-9) {
            selected[i] = true;
            result.seed_indices.push_back(i);
        }
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t candidate = 0; candidate < zones.size(); ++candidate) {
            if (!available[candidate] || selected[candidate]) continue;
            for (size_t member = 0; member < zones.size(); ++member) {
                if (!selected[member]) continue;
                if (!futureA1IntervalsOverlapOrTouch(
                        zones[member].other_enter,
                        zones[member].other_exit,
                        zones[candidate].other_enter,
                        zones[candidate].other_exit)) {
                    continue;
                }
                selected[candidate] = true;
                changed = true;
                break;
            }
        }
    }

    for (size_t i = 0; i < zones.size(); ++i) {
        if (!selected[i]) continue;
        result.protected_indices.push_back(i);
        const auto& zone = zones[i];
        if (!result.upstream_other_enter ||
            zone.other_enter < *result.upstream_other_enter) {
            result.upstream_other_enter = zone.other_enter;
        }
        if (other_path_s > zone.other_enter + 1e-9 &&
            other_path_s <= zone.other_exit + 1e-9) {
            result.other_already_inside = true;
        }
    }
    return result;
}

inline bool futureA1OtherInsideCluster(
    const std::vector<FutureA1ConflictInterval>& intervals,
    double other_path_s) {
    for (const auto& zone : intervals) {
        if (other_path_s > zone.other_enter + 1e-9 &&
            other_path_s <= zone.other_exit + 1e-9) {
            return true;
        }
    }
    return false;
}

inline bool departureClusterOwnerGenerationMatches(
    int transaction_owner_gen, int frozen_departure_gen,
    int actual_owner_gen) {
    return actual_owner_gen == transaction_owner_gen ||
           actual_owner_gen == frozen_departure_gen;
}

inline bool departureClusterCleared(
    double owner_path_s, double max_owner_release_exit_s) {
    return owner_path_s > max_owner_release_exit_s + 1e-9;
}

inline bool futureA1ArrivalWithinHorizon(double arrival_time,
                                         double horizon) {
    return std::isfinite(arrival_time) && arrival_time >= 0.0 &&
           arrival_time <= horizon + 1e-9;
}

inline int selectFutureA1Candidate(
    const std::vector<FutureA1RankedCandidate>& candidates,
    double tie_window,
    const std::function<int(int, int)>& priority_winner) {
    const FutureA1RankedCandidate* best = nullptr;
    for (const FutureA1RankedCandidate& candidate : candidates) {
        if (candidate.vehicle_id < 0 || candidate.arrival_time < 0.0) continue;
        if (best == nullptr ||
            candidate.arrival_time < best->arrival_time - tie_window) {
            best = &candidate;
            continue;
        }
        if (candidate.arrival_time > best->arrival_time + tie_window) continue;

        const int priority = priority_winner
            ? priority_winner(candidate.vehicle_id, best->vehicle_id)
            : -1;
        if (priority == candidate.vehicle_id) {
            best = &candidate;
        } else if (priority != best->vehicle_id &&
                   candidate.vehicle_id < best->vehicle_id) {
            best = &candidate;
        }
    }
    return best == nullptr ? -1 : best->vehicle_id;
}

inline std::optional<double> futureA1StopS(
    const std::optional<double>& frozen_departure_enter_s,
    double stop_buffer) {
    if (!frozen_departure_enter_s) return std::nullopt;
    return std::max(0.0, *frozen_departure_enter_s -
                             std::max(0.0, stop_buffer));
}

inline std::string futureA1TransitionReason(
    bool previous_valid, int previous_owner_id,
    bool current_valid, int current_owner_id,
    bool owner_locked, bool previous_owner_valid,
    bool previous_owner_horizon_exceeded, bool departure_handoff) {
    if (owner_locked) return "owner_entered_a1_locked";
    if (!previous_valid) {
        return current_valid ? "earliest_candidate" : "no_candidate";
    }
    if (departure_handoff) return "handoff_to_a1_departure_priority";
    if (!previous_owner_valid) return "owner_invalid";
    if (previous_owner_horizon_exceeded) return "owner_horizon_exceeded";
    if (!current_valid) return "no_candidate";
    if (current_owner_id != previous_owner_id) return "earlier_candidate";
    return "earliest_candidate";
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
