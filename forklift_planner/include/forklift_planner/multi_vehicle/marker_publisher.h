#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <map>
#include <set>
#include <utility>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/task_allocator.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"
#include "forklift_planner/planner_param.h"

namespace forklift_planner {
namespace multi_vehicle {

class MarkerPublisher {
public:
    MarkerPublisher(ros::NodeHandle& nh, const MapParam& mp,
                    const PlannerParam& pp,
                    const std::vector<Slot>& slots,
                    const MultiVehicleConfig& cfg,
                    const Slot& a1_pickup);
    bool hasSubscribers() const { return pub_.getNumSubscribers() > 0; }

    // 地图原点+XY轴(标定核对)。public:real_mode 启动时可经 latched 话题先发一次,
    // 不必等所有车动捕就绪(否则 tick 早退、per-tick publish 不跑 → 摆车前看不到轴)。
    void addOriginAxes(visualization_msgs::MarkerArray& arr) const;

    void publish(const std::vector<VehicleAgent>& vehicles,
                 const std::vector<bool>& visited_slots,
                 const std::vector<ConflictMarker>& conflicts,
                 const std::vector<ConflictMarker>& resource_markers,
                 const RuleEngine::FutureA1Commitment& future_a1,
                 const std::map<std::pair<int, int>,
                                RuleEngine::DepartureClusterCommitment>&
                     departure_clusters,
                 const RecoveryDirective& recovery) const;
    void setRollingDecision(
        const RuleEngine::RollingDynamicDecision& rolling_decision) {
        rolling_decision_ = rolling_decision;
    }

private:
    void addPathMarker(visualization_msgs::MarkerArray& arr,
                       const VehicleAgent& v) const;
    void addBodyMarker(visualization_msgs::MarkerArray& arr,
                       const VehicleAgent& v) const;
    void addArrowMarker(visualization_msgs::MarkerArray& arr,
                        const VehicleAgent& v) const;
    void addLabelMarker(visualization_msgs::MarkerArray& arr,
                        const VehicleAgent& v,
                        const RecoveryDirective& recovery) const;
    void addVisitedSlotMarkers(visualization_msgs::MarkerArray& arr,
                               const std::vector<bool>& visited_slots) const;
    void addConflictMarkers(visualization_msgs::MarkerArray& arr,
                            const std::vector<ConflictMarker>& conflicts,
                            const std::vector<ConflictMarker>&
                                resource_markers) const;
    void addA1DiagnosticMarkers(
        visualization_msgs::MarkerArray& arr,
        const std::vector<VehicleAgent>& vehicles,
        const RuleEngine::FutureA1Commitment& future_a1,
        const std::map<std::pair<int, int>,
                       RuleEngine::DepartureClusterCommitment>&
            departure_clusters) const;
    void addDeadlockRetreatTargetMarkers(
        visualization_msgs::MarkerArray& arr,
        const std::vector<VehicleAgent>& vehicles,
        const RecoveryDirective& recovery) const;

    ros::Publisher pub_;
    const MapParam& mp_;
    const PlannerParam& pp_;
    const std::vector<Slot>& slots_;
    const MultiVehicleConfig& cfg_;
    Slot a1_pickup_;
    mutable int last_same_direction_conflict_marker_count_ = 0;
    mutable int last_crossing_opposing_conflict_marker_count_ = 0;
    mutable int last_potential_conflict_zone_marker_count_ = 0;
    mutable int last_conflict_reservation_marker_count_ = 0;
    mutable std::set<int> last_zone_marker_ids_;
    mutable int last_a1_waiter_stop_marker_count_ = 0;
    mutable int last_a1_frozen_zone_marker_count_ = 0;
    mutable int publish_seq_ = 0;
    RuleEngine::RollingDynamicDecision rolling_decision_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
