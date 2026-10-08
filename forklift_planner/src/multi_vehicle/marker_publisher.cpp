#include "forklift_planner/multi_vehicle/marker_publisher.h"

#include <geometry_msgs/Point.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

geometry_msgs::Point pt3(double x, double y, double z = 0.08) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

int stableZoneMarkerId(const ConflictMarker& marker) {
    // FNV-1a over diagnostic identity. The sign bit is cleared because RViz
    // marker IDs are signed int32. Identity stays fixed while active-zone
    // positions are allowed to renumber after filtering.
    uint32_t hash = 2166136261u;
    for (int value : {marker.vehicle_a, marker.vehicle_b,
                      marker.path_gen_a, marker.path_gen_b,
                      marker.raw_zone_index}) {
        const uint32_t word = static_cast<uint32_t>(value);
        for (int shift = 0; shift < 32; shift += 8) {
            hash ^= (word >> shift) & 0xffu;
            hash *= 16777619u;
        }
    }
    return static_cast<int>(hash & 0x7fffffffu);
}

RoughWp displayPose(const VehicleAgent& v) {
    // 实车:显示真实 /object 位姿(实际在哪就画在哪,偏离路径多少看得见),而非投影点。
    if (v.real_pose_valid) {
        RoughWp p; p.x = v.real_x; p.y = v.real_y; p.theta = v.real_yaw;
        p.type = WpType::FORWARD;
        return p;
    }
    if (v.track.empty()) return {};
    if (v.mode == VehicleMode::DWELL) return v.track.poseAtS(v.track.length());
    return v.track.poseAtS(v.path_s);
}

void shelfCellY(const MapParam& p, int row_id, double& y0, double& y1) {
    constexpr double kSlotGapHalf = 0.012;
    const double g = kSlotGapHalf;
    switch (row_id) {
        case 0: y0 = p.y8();                  y1 = p.field_height;          break;
        case 1: y0 = (p.y6()+p.y7())*0.5 + g; y1 = p.y7();                  break;
        case 2: y0 = p.y6();                  y1 = (p.y6()+p.y7())*0.5 - g; break;
        case 3: y0 = (p.y4()+p.y5())*0.5 + g; y1 = p.y5();                  break;
        case 4: y0 = p.y4();                  y1 = (p.y4()+p.y5())*0.5 - g; break;
        case 5: y0 = (p.y2()+p.y3())*0.5 + g; y1 = p.y3();                  break;
        case 6: y0 = p.y2();                  y1 = (p.y2()+p.y3())*0.5 - g; break;
        default:y0 = 0.0;                     y1 = p.bottom_shelf_depth;    break;
    }
}

}  // namespace

MarkerPublisher::MarkerPublisher(ros::NodeHandle& nh, const MapParam& mp,
                                 const PlannerParam& pp,
                                 const std::vector<Slot>& slots,
                                 const MultiVehicleConfig& cfg,
                                 const Slot& a1_pickup)
    : mp_(mp), pp_(pp), slots_(slots), cfg_(cfg),
      a1_pickup_(a1_pickup) {
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/forklift_planner/markers", 10);
}

void MarkerPublisher::addA1DiagnosticMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<VehicleAgent>& vehicles,
    const RuleEngine::FutureA1Commitment& future_a1,
    const std::map<std::pair<int, int>,
                   RuleEngine::DepartureClusterCommitment>&
        departure_clusters) const {
    if (!cfg_.use_a1_cycle) return;
    const ros::Time now = ros::Time::now();

    auto deleteMarker = [&](const char* marker_ns, int id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = pp_.frame_id;
        marker.header.stamp = now;
        marker.ns = marker_ns;
        marker.id = id;
        marker.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(marker);
    };

    // Remove the former pickup footprint, direction arrow, pre-dock point,
    // and verbose diagnostic label.
    for (int id = 0; id < 4; ++id) {
        deleteMarker("a1_diagnostic_region", id);
    }

    int owner_id = -1;
    for (const auto& entry : departure_clusters) {
        if (entry.second.active) {
            owner_id = entry.second.owner_id;
            break;
        }
    }
    if (owner_id < 0 && future_a1.valid()) {
        owner_id = future_a1.owner_id;
    }

    visualization_msgs::Marker owner;
    owner.header.frame_id = pp_.frame_id;
    owner.header.stamp = now;
    owner.ns = "a1_owner";
    owner.id = 0;
    owner.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    owner.action = visualization_msgs::Marker::ADD;
    owner.pose.position.x = a1_pickup_.cx;
    owner.pose.position.y = a1_pickup_.cy;
    owner.pose.position.z = 0.19;
    owner.pose.orientation.w = 1.0;
    owner.scale.z = 0.075;
    owner.color = rgba(0.20f, 1.00f, 0.30f, 1.0f);
    owner.text = owner_id >= 0
        ? "A1 owner=V" + std::to_string(owner_id)
        : "A1 owner=none";
    arr.markers.push_back(owner);

    int waiter_marker_id = 0;
    int frozen_zone_marker_id = 0;
    for (const auto& entry : departure_clusters) {
        const RuleEngine::DepartureClusterCommitment& cluster =
            entry.second;
        if (!cluster.active) continue;

        const auto waiter = std::find_if(
            vehicles.begin(), vehicles.end(),
            [&](const VehicleAgent& vehicle) {
                return vehicle.id == cluster.other_id;
            });
        const bool active_waiter = waiter != vehicles.end() &&
            waiter->mission_phase == MissionPhase::TO_A1 &&
            !waiter->track.empty();
        const bool dwell_waiter = waiter != vehicles.end() &&
            waiter->mode == VehicleMode::DWELL &&
            waiter->mission_phase == MissionPhase::UNLOAD_DWELL &&
            !cluster.frozen_waiter_track.empty();
        if (active_waiter || dwell_waiter) {
            const PathTrack& waiter_track = active_waiter
                ? waiter->track : cluster.frozen_waiter_track;
            const RoughWp pose =
                waiter_track.poseAtS(cluster.waiter_stop_s);
            constexpr double kStopLineHalfLength = 0.18;
            const double nx = -std::sin(pose.theta);
            const double ny = std::cos(pose.theta);

            visualization_msgs::Marker stop_line;
            stop_line.header.frame_id = pp_.frame_id;
            stop_line.header.stamp = now;
            stop_line.ns = "a1_waiter_stop";
            stop_line.id = waiter_marker_id++;
            stop_line.type = visualization_msgs::Marker::LINE_STRIP;
            stop_line.action = visualization_msgs::Marker::ADD;
            stop_line.pose.orientation.w = 1.0;
            stop_line.scale.x = 0.030;
            stop_line.color = rgba(1.00f, 0.15f, 0.10f, 1.0f);
            stop_line.points.push_back(pt3(
                pose.x - kStopLineHalfLength * nx,
                pose.y - kStopLineHalfLength * ny, 0.070));
            stop_line.points.push_back(pt3(
                pose.x + kStopLineHalfLength * nx,
                pose.y + kStopLineHalfLength * ny, 0.070));
            arr.markers.push_back(stop_line);

            visualization_msgs::Marker stop_label;
            stop_label.header = stop_line.header;
            stop_label.ns = "a1_waiter_stop";
            stop_label.id = waiter_marker_id++;
            stop_label.type =
                visualization_msgs::Marker::TEXT_VIEW_FACING;
            stop_label.action = visualization_msgs::Marker::ADD;
            stop_label.pose.position.x = pose.x;
            stop_label.pose.position.y = pose.y;
            stop_label.pose.position.z = 0.16;
            stop_label.pose.orientation.w = 1.0;
            stop_label.scale.z = 0.060;
            stop_label.color = rgba(1.00f, 0.85f, 0.10f, 1.0f);
            std::ostringstream text;
            text << std::fixed << std::setprecision(2)
                 << "A1 STOP V" << waiter->id
                 << " s=" << cluster.waiter_stop_s;
            stop_label.text = text.str();
            arr.markers.push_back(stop_label);
        }

        for (const auto& aabb :
             cluster.diagnostic_protected_zone_aabbs) {
            if (!aabb.valid) continue;
            visualization_msgs::Marker zone;
            zone.header.frame_id = pp_.frame_id;
            zone.header.stamp = now;
            zone.ns = "a1_frozen_zone";
            zone.id = frozen_zone_marker_id++;
            zone.type = visualization_msgs::Marker::LINE_STRIP;
            zone.action = visualization_msgs::Marker::ADD;
            zone.pose.orientation.w = 1.0;
            zone.scale.x = 0.025;
            zone.color = rgba(1.00f, 0.05f, 0.05f, 1.0f);
            zone.points.push_back(
                pt3(aabb.min_x, aabb.min_y, 0.060));
            zone.points.push_back(
                pt3(aabb.max_x, aabb.min_y, 0.060));
            zone.points.push_back(
                pt3(aabb.max_x, aabb.max_y, 0.060));
            zone.points.push_back(
                pt3(aabb.min_x, aabb.max_y, 0.060));
            zone.points.push_back(
                pt3(aabb.min_x, aabb.min_y, 0.060));
            arr.markers.push_back(zone);
        }
    }

    for (int id = waiter_marker_id;
         id < last_a1_waiter_stop_marker_count_; ++id) {
        deleteMarker("a1_waiter_stop", id);
    }
    for (int id = frozen_zone_marker_id;
         id < last_a1_frozen_zone_marker_count_; ++id) {
        deleteMarker("a1_frozen_zone", id);
    }
    last_a1_waiter_stop_marker_count_ = waiter_marker_id;
    last_a1_frozen_zone_marker_count_ = frozen_zone_marker_id;
}

void MarkerPublisher::addPathMarker(visualization_msgs::MarkerArray& arr,
                                    const VehicleAgent& v) const {
    constexpr int kPathPublishStride = 10;
    if (cfg_.show_paths && publish_seq_ % kPathPublishStride != 0) {
        return;
    }

    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_path";
    m.id = v.id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = cfg_.show_paths ? visualization_msgs::Marker::ADD
                               : visualization_msgs::Marker::DELETE;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.016;
    m.color = v.color;
    if (cfg_.show_paths) {
        for (const RoughWp& p : v.track.path()) {
            m.points.push_back(pt3(p.x, p.y, 0.045));
        }
    }
    arr.markers.push_back(m);
}

void MarkerPublisher::addBodyMarker(visualization_msgs::MarkerArray& arr,
                                    const VehicleAgent& v) const {
    // 路径点是后轴参考；车身方块画在车身几何中心。
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    const double c = std::cos(p.theta);
    const double s = std::sin(p.theta);
    const double L = mp_.vehicle_length;
    const double W = mp_.vehicle_width;

    // RViz 只改变外观，不改变规划/碰撞 footprint：
    // 外包络仍是 L×W；后半段是车身，前半段分成两根叉臂，便于区分车头。
    const double body_len = L * 0.62;
    const double fork_len = L - body_len;
    const double fork_w = W * 0.22;
    const double body_x = -0.5 * L + 0.5 * body_len;
    const double fork_x = 0.5 * L - 0.5 * fork_len;
    const double fork_y = 0.5 * W - 0.5 * fork_w;

    auto addPart = [&](int id, double local_x, double local_y,
                       double sx, double sy) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "multi_patrol_body";
        m.id = id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = p.x + c * local_x - s * local_y;
        m.pose.position.y = p.y + s * local_x + c * local_y;
        m.pose.position.z = 0.035;
        m.pose.orientation.z = std::sin(p.theta * 0.5);
        m.pose.orientation.w = std::cos(p.theta * 0.5);
        m.scale.x = sx;
        m.scale.y = sy;
        m.scale.z = 0.050;
        m.color = v.color;
        arr.markers.push_back(m);
    };

    addPart(v.id, body_x, 0.0, body_len, W);
    addPart(1000 + v.id * 2, fork_x, fork_y, fork_len, fork_w);
    addPart(1001 + v.id * 2, fork_x, -fork_y, fork_len, fork_w);
}

void MarkerPublisher::addArrowMarker(visualization_msgs::MarkerArray& arr,
                                     const VehicleAgent& v) const {
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_arrow";
    m.id = v.id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.010;
    m.scale.y = 0.022;
    m.scale.z = 0.0;
    m.color = v.color;

    const double half = mp_.vehicle_length * 0.55;
    const double dx = std::cos(p.theta) * half;
    const double dy = std::sin(p.theta) * half;
    m.points.push_back(pt3(p.x - dx * 0.5, p.y - dy * 0.5, 0.070));
    m.points.push_back(pt3(p.x + dx, p.y + dy, 0.070));
    arr.markers.push_back(m);
}

void MarkerPublisher::addLabelMarker(visualization_msgs::MarkerArray& arr,
                                     const VehicleAgent& v,
                                     const RecoveryDirective& recovery) const {
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    constexpr double kLabelLongitudinalOffsetScale = 0.85;
    const double offset =
        kLabelLongitudinalOffsetScale * mp_.vehicle_length;
    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_label";
    m.id = v.id;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x + offset * std::cos(p.theta);
    m.pose.position.y = p.y + offset * std::sin(p.theta);
    m.pose.position.z = 0.160;
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.070;
    m.color = v.color;
    std::string displayed_action = actionName(v.action);
    if (recovery.active()) {
        const RecoveryMotion motion = recovery.motionFor(v.id);
        if (motion == RecoveryMotion::RETREAT) {
            displayed_action = "RETREAT";
        } else if (motion == RecoveryMotion::HOLD) {
            displayed_action = "HOLD";
        } else if (recovery.phase == RecoveryPhase::PASS &&
                   recovery.pass_vehicle_id == v.id) {
            displayed_action = "PASS";
        }
    }
    std::ostringstream text;
    text << std::fixed << std::setprecision(2)
         << "V" << v.id << " " << displayed_action
         << " V=" << v.current_speed << "\nTTC=";
    const RuleEngine::RollingDynamicDecision::VehicleTtcDiagnostic*
        diagnostic = nullptr;
    {
        const auto it = std::find_if(
            rolling_decision_.vehicle_ttc_diagnostics.begin(),
            rolling_decision_.vehicle_ttc_diagnostics.end(),
            [&](const RuleEngine::RollingDynamicDecision::
                    VehicleTtcDiagnostic& item) {
                return item.vehicle_id == v.id &&
                       item.path_gen == v.path_gen;
            });
        if (it != rolling_decision_.vehicle_ttc_diagnostics.end()) {
            diagnostic = &*it;
        }
    }
    if (diagnostic != nullptr && diagnostic->ttc) {
        text << *diagnostic->ttc << "s ";
        if (diagnostic->reason == "rolling_emergency_stop") {
            // This reason describes the pair. The label describes this
            // vehicle, so show its own final action instead.
            text << "final_" << actionName(v.action);
        } else {
            text << diagnostic->reason;
        }
    } else {
        text << "clear";
    }

    m.text = text.str();
    arr.markers.push_back(m);
}

void MarkerPublisher::addDeadlockRetreatTargetMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<VehicleAgent>& vehicles,
    const RecoveryDirective& recovery) const {
    const ros::Time now = ros::Time::now();
    auto erase = [&](const char* marker_ns) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = pp_.frame_id;
        marker.header.stamp = now;
        marker.ns = marker_ns;
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(marker);
    };

    const bool visible = recovery.phase == RecoveryPhase::RETREAT ||
        recovery.phase == RecoveryPhase::PASS;
    const auto retreat = std::find_if(
        vehicles.begin(), vehicles.end(), [&](const VehicleAgent& vehicle) {
            return vehicle.id == recovery.retreat_vehicle_id;
        });
    if (!visible || retreat == vehicles.end() || retreat->track.empty() ||
        retreat->path_gen != recovery.retreat_path_gen) {
        erase("deadlock_retreat_target");
        erase("deadlock_retreat_target_label");
        return;
    }

    const RoughWp pose = retreat->track.poseAtS(std::max(
        0.0, std::min(recovery.retreat_target_s,
                      retreat->track.length())));
    const double half_width = 0.75 * mp_.vehicle_width;
    const double nx = -std::sin(pose.theta);
    const double ny = std::cos(pose.theta);

    visualization_msgs::Marker line;
    line.header.frame_id = pp_.frame_id;
    line.header.stamp = now;
    line.ns = "deadlock_retreat_target";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.025;
    line.color = rgba(1.0f, 0.15f, 0.85f, 1.0f);
    line.points.push_back(pt3(pose.x - half_width * nx,
                              pose.y - half_width * ny, 0.095));
    line.points.push_back(pt3(pose.x + half_width * nx,
                              pose.y + half_width * ny, 0.095));
    arr.markers.push_back(line);

    visualization_msgs::Marker label;
    label.header = line.header;
    label.ns = "deadlock_retreat_target_label";
    label.id = 0;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = pose.x + half_width * nx;
    label.pose.position.y = pose.y + half_width * ny;
    label.pose.position.z = 0.14;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.055;
    label.color = line.color;
    label.text = "V" + std::to_string(retreat->id) + " retreat target";
    arr.markers.push_back(label);
}

void MarkerPublisher::addVisitedSlotMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<bool>& visited_slots) const {
    const size_t n = std::min(visited_slots.size(), slots_.size());
    for (size_t i = 0; i < n; ++i) {
        if (!visited_slots[i]) continue;
        const Slot& s = slots_[i];
        double y0 = 0.0;
        double y1 = 0.0;
        shelfCellY(mp_, s.row_id, y0, y1);

        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "visited_slots";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = s.cx;
        m.pose.position.y = 0.5 * (y0 + y1);
        m.pose.position.z = 0.018;
        m.pose.orientation.w = 1.0;
        m.scale.x = mp_.vehicle_width;
        m.scale.y = y1 - y0;
        m.scale.z = 0.004;
        m.color = rgba(1.0f, 0.88f, 0.05f, 0.92f);
        arr.markers.push_back(m);
    }
}

void MarkerPublisher::addConflictMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<ConflictMarker>& conflicts,
    const std::vector<ConflictMarker>& resource_markers) const {
    const char* same_ns = "conflict_same_direction";
    const char* mutual_ns = "conflict_crossing_or_opposing";
    const char* actual_ns = "conflict_timed_obb_overlap";
    const char* collision_start_ns = "timed_collision_start";
    const char* bridge_boundary_ns = "bridge_ttc_near_boundary";
    const char* conflict_label_ns = "conflict_explanation";
    const char* following_relation_ns = "following_relation";
    const char* following_label_ns = "following_explanation";
    const char* potential_zone_ns = "potential_conflict_zone_overlap";
    const char* potential_zone_label_ns = "potential_conflict_zone_explanation";
    const char* zone_aabb_ns = "conflict_zone_aabb";
    auto deleteMarker = [&](const char* marker_ns, int id) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = marker_ns;
        m.id = id;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    };
    auto deleteMarkers = [&](const char* marker_ns, int count) {
        for (int id = 0; id < count; ++id) {
            deleteMarker(marker_ns, id);
        }
    };

    if (!cfg_.show_prediction_conflicts) {
        deleteMarkers(same_ns,
                      last_same_direction_conflict_marker_count_);
        deleteMarkers(following_relation_ns,
                      last_same_direction_conflict_marker_count_);
        deleteMarkers(following_label_ns,
                      last_same_direction_conflict_marker_count_);
        deleteMarkers(mutual_ns,
                      last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(actual_ns,
                      last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(conflict_label_ns,
                      last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(bridge_boundary_ns,
                      2 * last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(collision_start_ns,
                      4 * last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(potential_zone_ns,
                      last_potential_conflict_zone_marker_count_);
        deleteMarkers(potential_zone_label_ns,
                      last_potential_conflict_zone_marker_count_);
        for (int id : last_zone_marker_ids_) {
            deleteMarker(potential_zone_ns, id);
            deleteMarker(potential_zone_label_ns, id);
            deleteMarker(zone_aabb_ns, id);
        }
        last_zone_marker_ids_.clear();
        last_same_direction_conflict_marker_count_ = 0;
        last_crossing_opposing_conflict_marker_count_ = 0;
        last_potential_conflict_zone_marker_count_ = 0;
        last_conflict_reservation_marker_count_ = 0;
        return;
    }

    int same_id = 0;
    int mutual_id = 0;
    int potential_zone_id = 0;

    std::set<int> current_zone_ids;
    auto addSpatialResource = [&](const ConflictMarker& c, int marker_id) {
        const ros::Time now = ros::Time::now();
        current_zone_ids.insert(marker_id);
        visualization_msgs::Marker geometry;
        geometry.header.frame_id = pp_.frame_id;
        geometry.header.stamp = now;
        geometry.ns = potential_zone_ns;
        geometry.id = marker_id;
        geometry.type = visualization_msgs::Marker::TRIANGLE_LIST;
        geometry.action = visualization_msgs::Marker::ADD;
        geometry.pose.orientation.w = 1.0;
        geometry.scale.x = 1.0;
        geometry.scale.y = 1.0;
        geometry.scale.z = 1.0;
        geometry.color = rgba(0.05f, 0.55f, 1.00f, 0.20f);
        const double z = 0.026;
        for (const auto& polygon : c.spatial_overlap_polygons) {
            if (polygon.size() < 3) continue;
            for (size_t p = 1; p + 1 < polygon.size(); ++p) {
                geometry.points.push_back(
                    pt3(polygon[0].x, polygon[0].y, z));
                geometry.points.push_back(
                    pt3(polygon[p].x, polygon[p].y, z));
                geometry.points.push_back(
                    pt3(polygon[p + 1].x, polygon[p + 1].y, z));
            }
        }
        if (!geometry.points.empty()) {
            arr.markers.push_back(geometry);
        } else {
            deleteMarker(potential_zone_ns, marker_id);
        }

        // Stage 3.2: blue polygons are geometry reference only. Static orange
        // ConflictZone AABBs and resource-style labels are deliberately
        // removed; active orange geometry is emitted from the current dynamic
        // interaction below.
        deleteMarker(zone_aabb_ns, marker_id);
        deleteMarker(potential_zone_label_ns, marker_id);
    };

    std::vector<const ConflictMarker*> all_markers;
    all_markers.reserve(conflicts.size() + resource_markers.size());
    for (const ConflictMarker& marker : conflicts) {
        all_markers.push_back(&marker);
    }
    for (const ConflictMarker& marker : resource_markers) {
        all_markers.push_back(&marker);
    }

    for (const ConflictMarker* marker_ptr : all_markers) {
        const ConflictMarker& c = *marker_ptr;
        if (c.kind == ConflictMarkerKind::POTENTIAL_CONFLICT_ZONE) {
            addSpatialResource(c, stableZoneMarkerId(c));
            ++potential_zone_id;
            continue;
        }
        if (c.kind == ConflictMarkerKind::CONFLICT_RESERVATION) {
            continue;
        }
        const bool same_direction =
            c.kind == ConflictMarkerKind::SAME_DIRECTION;
        const ros::Time now = ros::Time::now();
        const int marker_id = same_direction ? same_id++ : mutual_id++;
        if (same_direction) {
            deleteMarker(same_ns, marker_id);
        } else if (c.timed_overlaps.empty() &&
                   c.interaction_type != PairInteractionType::OPPOSING) {
            deleteMarker(mutual_ns, marker_id);
            deleteMarker(actual_ns, marker_id);
        }
        if (!same_direction) {
            auto addCollisionStart = [&](bool valid, double x, double y,
                                         int vehicle_id, int point_offset,
                                         const std_msgs::ColorRGBA& color) {
                const int point_id = 4 * marker_id + point_offset;
                const int label_id = 4 * marker_id + 2 + point_offset;
                if (!valid) {
                    deleteMarker(collision_start_ns, point_id);
                    deleteMarker(collision_start_ns, label_id);
                    return;
                }
                visualization_msgs::Marker point;
                point.header.frame_id = pp_.frame_id;
                point.header.stamp = now;
                point.ns = collision_start_ns;
                point.id = point_id;
                point.type = visualization_msgs::Marker::CYLINDER;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.position.x = x;
                point.pose.position.y = y;
                point.pose.position.z = 0.060;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.060;
                point.scale.y = 0.060;
                point.scale.z = 0.025;
                point.color = color;
                arr.markers.push_back(point);

                visualization_msgs::Marker label;
                label.header = point.header;
                label.ns = collision_start_ns;
                label.id = label_id;
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.action = visualization_msgs::Marker::ADD;
                label.pose.position.x = x;
                label.pose.position.y = y;
                label.pose.position.z = 0.145;
                label.pose.orientation.w = 1.0;
                label.scale.z = 0.055;
                label.color = color;
                label.text = "V" + std::to_string(vehicle_id) +
                    " collision";
                arr.markers.push_back(label);
            };
            addCollisionStart(
                c.timed_collision_start_valid,
                c.collision_a_x, c.collision_a_y, c.vehicle_a, 0,
                rgba(0.20f, 1.00f, 0.25f, 1.0f));
            addCollisionStart(
                c.timed_collision_start_valid,
                c.collision_b_x, c.collision_b_y, c.vehicle_b, 1,
                rgba(1.00f, 0.90f, 0.10f, 1.0f));

            auto addBridgeBoundary = [&](bool related, double x, double y,
                                         double corrected_ttc, int offset,
                                         const std_msgs::ColorRGBA& color) {
                const int boundary_id = 2 * marker_id + offset;
                if (!related) {
                    deleteMarker(bridge_boundary_ns, boundary_id);
                    return;
                }
                visualization_msgs::Marker boundary;
                boundary.header.frame_id = pp_.frame_id;
                boundary.header.stamp = now;
                boundary.ns = bridge_boundary_ns;
                boundary.id = boundary_id;
                boundary.type = visualization_msgs::Marker::SPHERE;
                boundary.action = visualization_msgs::Marker::ADD;
                boundary.pose.position.x = x;
                boundary.pose.position.y = y;
                boundary.pose.position.z = 0.055;
                boundary.pose.orientation.w = 1.0;
                boundary.scale.x = 0.075;
                boundary.scale.y = 0.075;
                boundary.scale.z = 0.025;
                boundary.color = color;
                boundary.text = "corrected_ttc=" +
                    std::to_string(corrected_ttc);
                arr.markers.push_back(boundary);
            };
            addBridgeBoundary(
                c.bridge_a_related, c.bridge_boundary_a_x,
                c.bridge_boundary_a_y, c.bridge_corrected_ttc_a, 0,
                rgba(0.15f, 0.95f, 1.00f, 0.95f));
            addBridgeBoundary(
                c.bridge_b_related, c.bridge_boundary_b_x,
                c.bridge_boundary_b_y, c.bridge_corrected_ttc_b, 1,
                rgba(1.00f, 0.35f, 0.85f, 0.95f));
        }
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = now;
        m.ns = same_direction ? same_ns : mutual_ns;
        m.id = marker_id;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = 0.018;
        m.pose.orientation.w = 1.0;
        if (!same_direction &&
            (!c.timed_overlaps.empty() ||
             c.interaction_type == PairInteractionType::OPPOSING)) {
            // Orange is only the axis-aligned bounding outline of the sampled
            // time-synchronised OBB intersections rendered below.
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.pose.position.x = 0.0;
            m.pose.position.y = 0.0;
            m.scale.x = 0.012;
            m.color = rgba(1.00f, 0.62f, 0.18f, 0.95f);
            const double x0 = c.x - 0.5 * c.scale_x;
            const double x1 = c.x + 0.5 * c.scale_x;
            const double y0 = c.y - 0.5 * c.scale_y;
            const double y1 = c.y + 0.5 * c.scale_y;
            m.points.push_back(pt3(x0, y0, 0.035));
            m.points.push_back(pt3(x1, y0, 0.035));
            m.points.push_back(pt3(x1, y1, 0.035));
            m.points.push_back(pt3(x0, y1, 0.035));
            m.points.push_back(pt3(x0, y0, 0.035));
            arr.markers.push_back(m);
        }

        if (same_direction) {
            visualization_msgs::Marker relation;
            relation.header = m.header;
            relation.ns = following_relation_ns;
            relation.id = marker_id;
            relation.type = visualization_msgs::Marker::ARROW;
            relation.action = visualization_msgs::Marker::ADD;
            relation.pose.orientation.w = 1.0;
            relation.scale.x = 0.015;
            relation.scale.y = 0.040;
            relation.scale.z = 0.055;
            relation.color = rgba(0.10f, 0.90f, 1.00f, 1.0f);
            relation.points.push_back(
                pt3(c.follower_x, c.follower_y, 0.075));
            relation.points.push_back(
                pt3(c.leader_x, c.leader_y, 0.075));
            arr.markers.push_back(relation);

            visualization_msgs::Marker label;
            label.header = m.header;
            label.ns = following_label_ns;
            label.id = marker_id;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position.x = 0.5 * (c.follower_x + c.leader_x);
            label.pose.position.y = 0.5 * (c.follower_y + c.leader_y);
            label.pose.position.z = 0.16;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.070;
            label.color = rgba(0.10f, 0.90f, 1.00f, 1.0f);
            std::ostringstream text;
            text << std::fixed << std::setprecision(2)
                 << "FOLLOW V" << c.follower_id << " -> V" << c.leader_id
                 << " gap=" << c.following_gap << "m"
                 << " action=" << actionName(c.following_action);
            label.text = text.str();
            arr.markers.push_back(label);
        } else {
            visualization_msgs::Marker actual;
            actual.header = m.header;
            actual.ns = actual_ns;
            actual.id = marker_id;
            actual.type = visualization_msgs::Marker::TRIANGLE_LIST;
            actual.action = visualization_msgs::Marker::ADD;
            actual.pose.orientation.w = 1.0;
            actual.scale.x = 1.0;
            actual.scale.y = 1.0;
            actual.scale.z = 1.0;
            actual.color = rgba(1.00f, 0.10f, 0.08f, 0.34f);
            for (const ConflictMarker::TimedOverlap& overlap :
                 c.timed_overlaps) {
                if (overlap.polygon.size() < 3) continue;
                for (size_t p = 1; p + 1 < overlap.polygon.size(); ++p) {
                    actual.points.push_back(pt3(overlap.polygon[0].x,
                                                overlap.polygon[0].y, 0.042));
                    actual.points.push_back(pt3(overlap.polygon[p].x,
                                                overlap.polygon[p].y, 0.042));
                    actual.points.push_back(pt3(overlap.polygon[p + 1].x,
                                                overlap.polygon[p + 1].y, 0.042));
                }
            }
            if (!actual.points.empty()) arr.markers.push_back(actual);

            visualization_msgs::Marker label;
            label.header = m.header;
            label.ns = conflict_label_ns;
            label.id = marker_id;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position.x = c.x;
            label.pose.position.y = c.y;
            label.pose.position.z = 0.16;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.070;
            label.color = rgba(1.00f, 0.80f, 0.25f, 1.0f);
            std::ostringstream text;
            text << "V" << c.vehicle_a << "-V" << c.vehicle_b
                 << " type="
                 << (c.interaction_type == PairInteractionType::OPPOSING
                         ? "OPPOSING" : "CROSSING");
            label.text = text.str();
            arr.markers.push_back(label);
        }
    }

    for (int stale = same_id;
         stale < last_same_direction_conflict_marker_count_; ++stale) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = same_ns;
        m.id = stale;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    }
    for (int stale = same_id;
         stale < last_same_direction_conflict_marker_count_; ++stale) {
        deleteMarker(following_relation_ns, stale);
        deleteMarker(following_label_ns, stale);
    }
    for (int stale = mutual_id;
         stale < last_crossing_opposing_conflict_marker_count_; ++stale) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = mutual_ns;
        m.id = stale;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    }
    for (int stale = mutual_id;
         stale < last_crossing_opposing_conflict_marker_count_; ++stale) {
        deleteMarker(actual_ns, stale);
        deleteMarker(conflict_label_ns, stale);
        deleteMarker(bridge_boundary_ns, 2 * stale);
        deleteMarker(bridge_boundary_ns, 2 * stale + 1);
        deleteMarker(collision_start_ns, 4 * stale);
        deleteMarker(collision_start_ns, 4 * stale + 1);
        deleteMarker(collision_start_ns, 4 * stale + 2);
        deleteMarker(collision_start_ns, 4 * stale + 3);
    }
    for (int stale : last_zone_marker_ids_) {
        if (current_zone_ids.count(stale) != 0) continue;
        deleteMarker(potential_zone_ns, stale);
        deleteMarker(potential_zone_label_ns, stale);
        deleteMarker(zone_aabb_ns, stale);
    }
    last_same_direction_conflict_marker_count_ = same_id;
    last_crossing_opposing_conflict_marker_count_ = mutual_id;
    last_potential_conflict_zone_marker_count_ = potential_zone_id;
    last_conflict_reservation_marker_count_ = 0;
    last_zone_marker_ids_ = std::move(current_zone_ids);
}

void MarkerPublisher::addOriginAxes(visualization_msgs::MarkerArray& arr) const {
    // 地图原点 (0,0) 与 X/Y 正方向。实车标定时:动捕 world 原点应与此重合、
    // 某车放已知槽位时 /object÷1000 应≈该槽坐标,车头朝 +X(红轴)时 yaw≈0。
    constexpr double L = 0.5;   // 轴长 0.5m(地图 ~2.5×4.5,够看又不挡)
    const double z = 0.075;     // 抬到与车身箭头同高(z=0.07),否则被地图平面盖住看不见
    auto axis = [&](int id, double ex, double ey, const std_msgs::ColorRGBA& col) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "map_origin_axes";
        m.id = id;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.022;   // 杆径(加粗,醒目)
        m.scale.y = 0.05;    // 箭头径
        m.scale.z = 0.08;    // 箭头长
        m.color = col;
        m.points.push_back(pt3(0.0, 0.0, z));
        m.points.push_back(pt3(ex * L, ey * L, z));
        arr.markers.push_back(m);
    };
    auto label = [&](int id, double x, double y, const std::string& txt,
                     const std_msgs::ColorRGBA& col) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "map_origin_axes";
        m.id = id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.14;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.10;
        m.color = col;
        m.text = txt;
        arr.markers.push_back(m);
    };
    axis(0, 1.0, 0.0, rgba(0.95f, 0.1f, 0.1f));   // +X 红
    axis(1, 0.0, 1.0, rgba(0.1f, 0.9f, 0.1f));    // +Y 绿
    label(2, L + 0.06, 0.0, "+X", rgba(0.95f, 0.1f, 0.1f));
    label(3, 0.0, L + 0.06, "+Y", rgba(0.1f, 0.9f, 0.1f));
    label(4, -0.06, -0.06, "O(0,0)", rgba(1.0f, 1.0f, 1.0f));
}

void MarkerPublisher::publish(
    const std::vector<VehicleAgent>& vehicles,
    const std::vector<bool>& visited_slots,
    const std::vector<ConflictMarker>& conflicts,
    const std::vector<ConflictMarker>& resource_markers,
    const RuleEngine::FutureA1Commitment& future_a1,
    const std::map<std::pair<int, int>,
                   RuleEngine::DepartureClusterCommitment>&
        departure_clusters,
    const RecoveryDirective& recovery) const {
    ++publish_seq_;
    visualization_msgs::MarkerArray arr;
    addVisitedSlotMarkers(arr, visited_slots);
    addA1DiagnosticMarkers(arr, vehicles, future_a1,
                           departure_clusters);
    addDeadlockRetreatTargetMarkers(arr, vehicles, recovery);
    addOriginAxes(arr);  // 地图原点+XY正方向(标定核对用)
    for (const VehicleAgent& v : vehicles) {
        addPathMarker(arr, v);
        if (v.mode == VehicleMode::NEED_TASK || v.track.empty()) continue;
        addBodyMarker(arr, v);
        addArrowMarker(arr, v);
        addLabelMarker(arr, v, recovery);
    }
    addConflictMarkers(arr, conflicts, resource_markers);
    pub_.publish(arr);
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
