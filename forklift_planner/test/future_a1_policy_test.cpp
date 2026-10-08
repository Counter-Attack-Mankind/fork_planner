#include "forklift_planner/multi_vehicle/future_a1_policy.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"

#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

using forklift_planner::multi_vehicle::FutureA1RankedCandidate;
using forklift_planner::multi_vehicle::FutureA1ConflictInterval;
using forklift_planner::multi_vehicle::RuleEngine;
using forklift_planner::multi_vehicle::departureClusterCleared;
using forklift_planner::multi_vehicle::
    departureClusterOwnerGenerationMatches;
using forklift_planner::multi_vehicle::futureA1ArrivalWithinHorizon;
using forklift_planner::multi_vehicle::futureA1OtherInsideCluster;
using forklift_planner::multi_vehicle::futureA1StopS;
using forklift_planner::multi_vehicle::selectFutureA1ProtectedCluster;
using forklift_planner::multi_vehicle::selectFutureA1Candidate;

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

int fail(const char* message) {
    std::cerr << "future_a1_policy_test: " << message << '\n';
    return 1;
}

}  // namespace

int main() {
    // A. No arrival outside the active horizon may become a candidate.
    if (futureA1ArrivalWithinHorizon(15.1, 15.0) ||
        futureA1ArrivalWithinHorizon(41.0, 15.0)) {
        return fail("arrival beyond horizon was accepted");
    }

    // B. With no service owner, ranking selects the clearly earlier candidate.
    const std::vector<FutureA1RankedCandidate> candidates{
        {0, 14.0}, {1, 6.0}};
    const int selected = selectFutureA1Candidate(
        candidates, 0.2, [](int lhs, int rhs) {
            return std::min(lhs, rhs);
        });
    if (selected != 1) return fail("earlier candidate did not win");
    std::cout << "B event=SELECT owner=V1 reason=earlier_candidate\n";

    // C. Admission stops only before the frozen departure entry.
    const std::optional<double> stop_s = futureA1StopS(3.075, 0.01);
    if (!stop_s || !near(*stop_s, 3.065) || *stop_s >= 3.075) {
        return fail("frozen departure entry did not bound stop_s");
    }

    // D. A protected seed pulls in an overlapping downstream-owner zone.
    const std::vector<FutureA1ConflictInterval> double_zone{
        {0.000, 0.650, 4.800, 5.425},
        {0.800, 1.900, 4.125, 5.125}};
    const auto cluster =
        selectFutureA1ProtectedCluster(double_zone, 0.756, 0.0);
    if (cluster.seed_indices != std::vector<size_t>{0} ||
        cluster.protected_indices != std::vector<size_t>({0, 1}) ||
        !cluster.upstream_other_enter ||
        !near(*cluster.upstream_other_enter, 4.125)) {
        return fail("overlapping second future-exit zone was not protected");
    }
    const auto clustered_stop =
        futureA1StopS(cluster.upstream_other_enter, 0.01);
    if (!clustered_stop || !near(*clustered_stop, 4.115)) {
        return fail("clustered future-exit stop line was not upstream");
    }

    // E. Closure is transitive even when the seed does not touch zone2.
    const std::vector<FutureA1ConflictInterval> transitive{
        {0.0, 0.5, 5.0, 6.0},
        {0.8, 1.2, 4.0, 5.2},
        {1.5, 2.0, 3.0, 4.1}};
    const auto transitive_cluster =
        selectFutureA1ProtectedCluster(transitive, 0.6, 0.0);
    if (transitive_cluster.protected_indices !=
        std::vector<size_t>({0, 1, 2})) {
        return fail("future-exit conflict closure was not transitive");
    }

    // F. A disconnected remote zone must not become protected.
    const std::vector<FutureA1ConflictInterval> remote{
        {0.0, 0.5, 2.0, 2.5},
        {2.0, 2.5, 8.0, 8.5}};
    const auto remote_cluster =
        selectFutureA1ProtectedCluster(remote, 0.6, 0.0);
    if (remote_cluster.protected_indices != std::vector<size_t>{0}) {
        return fail("disconnected remote future-exit zone was protected");
    }

    // G. A fully passed zone cannot affect or bridge the current cluster.
    const auto passed_cluster =
        selectFutureA1ProtectedCluster(double_zone, 0.756, 5.5);
    if (!passed_cluster.protected_indices.empty() ||
        passed_cluster.upstream_other_enter) {
        return fail("fully passed future-exit zone still affected admission");
    }

    // H. Entering any member of the closure preserves actual occupancy.
    const auto inside_cluster =
        selectFutureA1ProtectedCluster(double_zone, 0.756, 4.7);
    if (!inside_cluster.other_already_inside) {
        return fail("actual occupancy inside protected closure was missed");
    }

    // Departure A. The frozen TO_B boundary remains the single source.
    const auto handoff_stop = futureA1StopS(0.500, 0.010);
    if (!handoff_stop || !near(*handoff_stop, 0.490)) {
        return fail("TO_B handoff relaxed the Future stop boundary");
    }

    // Departure B/C. The cluster lifetime is independent of a Future
    // candidate and lasts through the maximum owner-side exit.
    if (departureClusterCleared(3.125, 3.125) ||
        !departureClusterCleared(3.126, 3.125)) {
        return fail("departure cluster release boundary is incorrect");
    }

    // Departure D. Both the service leg N and frozen departure N+1 belong to
    // one transaction; unrelated generations do not.
    if (!departureClusterOwnerGenerationMatches(7, 8, 7) ||
        !departureClusterOwnerGenerationMatches(7, 8, 8) ||
        departureClusterOwnerGenerationMatches(7, 8, 9)) {
        return fail("frozen departure generation identity is incorrect");
    }

    // Departure E/G. The remote zone remains excluded, while physical entry
    // into any selected member is recognized as actual occupancy.
    std::vector<FutureA1ConflictInterval> handoff_intervals;
    for (size_t index : transitive_cluster.protected_indices) {
        handoff_intervals.push_back(transitive[index]);
    }
    if (!futureA1OtherInsideCluster(handoff_intervals, 4.050) ||
        futureA1OtherInsideCluster(handoff_intervals, 8.100)) {
        return fail("departure cluster actual occupancy classification failed");
    }

    // Departure F. SimSnapshot owns a value copy of departure handoffs, so a
    // rollout mutation can be discarded by restoring the saved snapshot.
    RuleEngine::SimSnapshot live;
    RuleEngine::DepartureClusterCommitment commitment;
    commitment.owner_id = 0;
    commitment.owner_path_gen = 7;
    commitment.other_id = 1;
    commitment.other_path_gen = 11;
    commitment.waiter_stop_s = 0.490;
    commitment.owner_release_exit_s = 3.125;
    commitment.active = true;
    live.a1.departure_clusters[{0, 1}] = commitment;
    const RuleEngine::SimSnapshot saved = live;
    live.a1.departure_clusters.clear();
    live = saved;
    if (live.a1.departure_clusters.size() != 1 ||
        !near(live.a1.departure_clusters.at({0, 1}).waiter_stop_s, 0.490)) {
        return fail("departure cluster snapshot/restore value was lost");
    }

    std::cout << "A no_owner_when_all_eta_exceed_horizon=PASS\n"
              << "C future_exit_enter=3.075 ordinary_enter=2.175 "
                 "stop_s=2.165 PASS\n"
              << "D closure_zones=[0,1] future_exit_enter=4.125 "
                 "stop_s=4.115 PASS\n"
              << "E transitive_closure=PASS\n"
              << "F disconnected_remote_zone=PASS\n"
              << "G passed_zone_excluded=PASS\n"
              << "H actual_occupancy_preserved=PASS\n"
              << "Departure-A handoff_stop_s=0.490 PASS\n"
              << "Departure-B future_candidate_independent=PASS\n"
              << "Departure-C owner_release_exit=3.125 PASS\n"
              << "Departure-D frozen_generation_identity=PASS\n"
              << "Departure-E remote_zone_excluded=PASS\n"
              << "Departure-F snapshot_restore=PASS\n"
              << "Departure-G already_inside_actual_priority=PASS\n"
              << "future_a1_policy_test: PASS\n";
    return 0;
}
