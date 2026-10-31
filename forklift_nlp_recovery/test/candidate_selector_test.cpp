#include <gtest/gtest.h>

#include <algorithm>

#include <sandbox_msgs/RecoverySnapshot.h>

#include "forklift_map/forklift_map.h"
#include "forklift_nlp_recovery/candidate_selector.h"

namespace fnr = forklift_nlp_recovery;

namespace {

sandbox_msgs::RecoveryVehicleState vehicle(
    int id, const fnr::Pose2d& pose, const std::vector<fnr::Pose2d>& path) {
    sandbox_msgs::RecoveryVehicleState state;
    state.id = id;
    state.path_gen = id + 10;
    state.pose_valid = true;
    state.pose_fresh = true;
    state.departed_source_slot = true;
    state.pose.x = pose.x;
    state.pose.y = pose.y;
    state.pose.theta = pose.yaw;
    for (const auto& point : path) {
        geometry_msgs::Pose2D message;
        message.x = point.x;
        message.y = point.y;
        message.theta = point.yaw;
        state.path.push_back(message);
        state.path_types.push_back(0);
    }
    return state;
}

fnr::VehicleParameters parameters(const MapParam& map,
                                  const fnr::CorridorBuilder& corridor) {
    fnr::VehicleParameters value;
    value.length = map.vehicle_length;
    value.width = map.vehicle_width;
    value.rear_axle_to_center = map.rear_axle_to_center;
    value.disc_radius = corridor.vehicleRadius();
    value.wheel_base = map.wheel_base;
    value.max_steer_angle = map.max_steer_angle;
    value.max_steer_rate = map.max_steer_rate;
    value.max_speed = 0.26;
    value.max_accel = 0.20;
    value.max_decel = 0.30;
    return value;
}

}  // namespace

TEST(CandidateSelectorTest, ExcludesAnyObbIntersectingRow2Bridge) {
    MapParam map;
    ForkliftMap forklift_map(map);
    fnr::CorridorConfig corridor_config;
    fnr::CorridorBuilder corridor(map, forklift_map.shelf_blocks(), corridor_config);
    fnr::OptimizationInputBuilder input_builder(corridor,
        parameters(map, corridor));
    fnr::CandidateSelector selector(map, forklift_map.shelf_blocks(), {}, input_builder);
    sandbox_msgs::RecoverySnapshot snapshot;
    snapshot.header.frame_id = "map";
    snapshot.vehicles.push_back(vehicle(1, {1.25, 2.25, M_PI * 0.5},
        {{1.25, 2.25, M_PI * 0.5}, {1.25, 2.80, M_PI * 0.5},
         {1.25, 3.40, M_PI * 0.5}}));
    snapshot.vehicles.push_back(vehicle(2, {0.50, map.corridor4_cy(), 0.0},
        {{0.50, map.corridor4_cy(), 0.0}, {1.20, map.corridor4_cy(), 0.0},
         {2.45, map.corridor4_cy(), 0.0}}));
    const auto result = selector.evaluate(snapshot, {1, 2});
    ASSERT_TRUE(result.complete_snapshot) << result.reason;
    const auto found = std::find_if(result.ranked.begin(), result.ranked.end(),
        [](const fnr::CandidateEvaluation& value) { return value.vehicle_id == 1; });
    ASSERT_NE(found, result.ranked.end());
    EXPECT_FALSE(found->eligible);
    EXPECT_NE(found->reason.find("row-2 bridge"), std::string::npos);

    const auto bridge = selector.evaluateBridgeEscapes(snapshot, {1, 2});
    ASSERT_TRUE(bridge.complete_snapshot) << bridge.reason;
    const auto escape = std::find_if(bridge.ranked.begin(), bridge.ranked.end(),
        [](const fnr::BridgeEscapeEvaluation& value) { return value.vehicle_id == 1; });
    ASSERT_NE(escape, bridge.ranked.end());
    EXPECT_GT(escape->escape_path_s, snapshot.vehicles.front().path_s);
    EXPECT_FALSE(escape->eligible);
    EXPECT_NE(escape->reason.find("no effective positive spatial gain"),
              std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
