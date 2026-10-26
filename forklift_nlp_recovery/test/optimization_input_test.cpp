#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <ros/time.h>

#include "forklift_map/forklift_map.h"
#include "forklift_nlp_recovery/optimization_input.h"

namespace fnr = forklift_nlp_recovery;

namespace {

sandbox_msgs::RecoveryVehicleState makeVehicle(int id, double y, int path_gen) {
    sandbox_msgs::RecoveryVehicleState vehicle;
    vehicle.id = id;
    vehicle.path_gen = path_gen;
    vehicle.path_s = 0.0;
    vehicle.current_speed = 0.07;
    vehicle.pose_valid = true;
    vehicle.pose.x = 0.46;
    vehicle.pose.y = y;
    vehicle.pose.theta = 0.0;
    for (const double x : {0.45, 0.85, 1.25, 1.65, 2.05, 2.45}) {
        geometry_msgs::Pose2D pose;
        pose.x = x;
        pose.y = y;
        pose.theta = 0.0;
        vehicle.path.push_back(pose);
    }
    vehicle.path_types = {0, 0, 0, 0, 0, 0};
    return vehicle;
}

sandbox_msgs::RecoveryVehicleState makeStaticVehicle(
    int id, const fnr::Pose2d& rear_axle_pose) {
    sandbox_msgs::RecoveryVehicleState vehicle;
    vehicle.id = id;
    vehicle.pose_valid = true;
    vehicle.pose.x = rear_axle_pose.x;
    vehicle.pose.y = rear_axle_pose.y;
    vehicle.pose.theta = rear_axle_pose.yaw;
    return vehicle;
}

fnr::VehicleParameters makeParameters(const MapParam& map,
                                      const fnr::CorridorBuilder& corridor) {
    fnr::VehicleParameters parameters;
    parameters.length = map.vehicle_length;
    parameters.width = map.vehicle_width;
    parameters.rear_axle_to_center = map.rear_axle_to_center;
    parameters.disc_radius = corridor.vehicleRadius();
    parameters.wheel_base = map.wheel_base;
    parameters.max_steer_angle = map.max_steer_angle;
    parameters.max_steer_rate = map.max_steer_rate;
    parameters.nominal_speed = 0.20;
    parameters.max_speed = 0.26;
    parameters.max_accel = 0.20;
    parameters.max_decel = 0.30;
    parameters.fixed_retreat_speed = 0.10;
    return parameters;
}

}  // namespace

class OptimizationInputTest : public ::testing::Test {
protected:
    OptimizationInputTest()
        : map(map_param), corridor(map_param, map.shelf_blocks(), corridor_config),
          input_builder(corridor, makeParameters(map_param, corridor)) {}

    sandbox_msgs::RecoverySnapshot baseSnapshot() const {
        sandbox_msgs::RecoverySnapshot snapshot;
        snapshot.header.frame_id = "map";
        snapshot.header.stamp = ros::Time(12, 34);
        snapshot.reason = "test_deadlock";
        snapshot.vehicles.push_back(makeVehicle(2, map_param.corridor4_cy(), 12));
        snapshot.vehicles.push_back(makeVehicle(5, map_param.corridor3_cy(), 15));
        snapshot.vehicles.push_back(makeVehicle(9, map_param.corridor2_cy(), 19));
        return snapshot;
    }

    MapParam map_param;
    ForkliftMap map;
    fnr::CorridorConfig corridor_config;
    fnr::CorridorBuilder corridor;
    fnr::OptimizationInputBuilder input_builder;
};

TEST_F(OptimizationInputTest, BuildsTwoVehicleInputAndPreservesMetadata) {
    auto snapshot = baseSnapshot();
    const auto preliminary = input_builder.build(snapshot, {2, 9}, "debug");
    ASSERT_TRUE(preliminary.complete) << preliminary.reason;
    ASSERT_TRUE(preliminary.input.valid) << preliminary.reason;
    ASSERT_EQ(preliminary.input.vehicles.size(), 2U);

    EXPECT_EQ(preliminary.input.vehicles.front().corridor.size(), 1U);
    const fnr::Pose2d near_center = corridor.bodyCenter(
        preliminary.input.vehicles.front().initial_state.rear_axle_pose);
    const fnr::Pose2d near_rear{
        near_center.x - map_param.rear_axle_to_center, near_center.y, 0.0};
    snapshot.vehicles.push_back(makeStaticVehicle(41, near_rear));
    snapshot.vehicles.push_back(makeStaticVehicle(42, {10.0, 10.0, 0.0}));
    auto parked = makeStaticVehicle(43, near_rear);
    parked.parked_in_slot = true;
    snapshot.vehicles.push_back(parked);

    const auto result = input_builder.build(snapshot, {2, 9}, "debug");
    ASSERT_TRUE(result.complete) << result.reason;
    ASSERT_TRUE(result.input.valid) << result.reason;
    ASSERT_EQ(result.input.participant_ids, std::vector<int>({2, 9}));
    ASSERT_EQ(result.input.vehicles.size(), 2U);
    EXPECT_EQ(result.input.vehicles[0].id, 2);
    EXPECT_EQ(result.input.vehicles[1].id, 9);
    EXPECT_DOUBLE_EQ(result.input.vehicles[0].initial_state.longitudinal_speed,
                     0.07);
    EXPECT_EQ(result.input.vehicles[0].path_gen, 12);
    EXPECT_EQ(result.input.vehicles[0].terminal.path_gen, 12);
    EXPECT_EQ(result.input.vehicles[0].original_path.size(), 6U);
    EXPECT_EQ(result.input.vehicles[0].corridor.size(), 1U);
    ASSERT_EQ(result.input.static_vehicles.size(), 4U);
    EXPECT_EQ(result.input.static_vehicles.front().id, 5);
    EXPECT_DOUBLE_EQ(result.input.static_vehicles.front().length,
                     result.input.vehicles.front().parameters.length);
    EXPECT_DOUBLE_EQ(result.input.static_vehicles.front().width,
                     result.input.vehicles.front().parameters.width);
    EXPECT_DOUBLE_EQ(result.input.static_vehicles.front().rear_axle_to_center,
                     result.input.vehicles.front().parameters.rear_axle_to_center);
    EXPECT_EQ(result.input.trigger_reason, "test_deadlock");
}

TEST_F(OptimizationInputTest, BuildsThreeNonContiguousVehicleIds) {
    const auto result = input_builder.build(baseSnapshot(), {2, 5, 9}, "auto");
    ASSERT_TRUE(result.complete) << result.reason;
    ASSERT_TRUE(result.input.valid) << result.reason;
    ASSERT_EQ(result.input.vehicles.size(), 3U);
    EXPECT_EQ(result.input.participant_ids, std::vector<int>({2, 5, 9}));
    EXPECT_EQ(result.input.vehicles[0].id, 2);
    EXPECT_EQ(result.input.vehicles[1].id, 5);
    EXPECT_EQ(result.input.vehicles[2].id, 9);
    EXPECT_EQ(result.input.source_mode, "auto");
}

TEST_F(OptimizationInputTest, StaticVehicleInfluenceIncludesExactBoundary) {
    fnr::NlpVehicleInput participant;
    participant.parameters.disc_radius = 0.20;
    participant.corridor.push_back({0.0, 1.0, 0.0, 1.0});

    EXPECT_TRUE(fnr::OptimizationInputBuilder::discMayAffectCorridor(
        {1.50, 0.50, 0.0}, 0.30, participant));
    EXPECT_FALSE(fnr::OptimizationInputBuilder::discMayAffectCorridor(
        {1.50001, 0.50, 0.0}, 0.30, participant));
    EXPECT_TRUE(fnr::OptimizationInputBuilder::discMayAffectCorridor(
        {1.30, 1.40, 0.0}, 0.30, participant));
}

TEST_F(OptimizationInputTest, ReportsMissingParticipantWithoutInventingState) {
    const auto result = input_builder.build(baseSnapshot(), {2, 77}, "debug");
    EXPECT_FALSE(result.complete);
    EXPECT_FALSE(result.input.valid);
    EXPECT_NE(result.reason.find("missing participant V77"), std::string::npos);
    ASSERT_EQ(result.input.vehicles.size(), 1U);
    EXPECT_EQ(result.input.vehicles.front().id, 2);
}

TEST_F(OptimizationInputTest, WritesInspectableDebugFile) {
    const auto result = input_builder.build(baseSnapshot(), {2, 5, 9}, "auto");
    ASSERT_TRUE(result.complete);
    const std::string path = "/tmp/forklift_nlp_recovery_input_test.yaml";
    std::string error;
    ASSERT_TRUE(fnr::writeOptimizationInputDebugFile(result, path, error)) << error;
    std::ifstream input(path);
    ASSERT_TRUE(input.good());
    const std::string text((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    EXPECT_NE(text.find("participant_ids: [2, 5, 9]"), std::string::npos);
    EXPECT_NE(text.find("original_path:"), std::string::npos);
    EXPECT_NE(text.find("aabbs:"), std::string::npos);
    EXPECT_NE(text.find("static_vehicles:"), std::string::npos);
    std::remove(path.c_str());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

