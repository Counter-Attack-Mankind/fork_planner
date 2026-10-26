#include <gtest/gtest.h>

#include "forklift_nlp_recovery/hybrid_astar.h"

namespace fnr = forklift_nlp_recovery;

namespace {

fnr::NlpVehicleInput straightVehicle() {
    fnr::NlpVehicleInput vehicle;
    vehicle.id = 3;
    vehicle.initial_state.rear_axle_pose = {0.50, 1.00, 0.0};
    vehicle.terminal = {{1.50, 1.00, 0.0}, 1.0, 8, 0};
    vehicle.path_gen = 8;
    vehicle.parameters.length = 0.211;
    vehicle.parameters.width = 0.191;
    vehicle.parameters.rear_axle_to_center = 0.0735;
    vehicle.parameters.disc_radius = 0.15;
    vehicle.parameters.wheel_base = 0.143;
    vehicle.parameters.max_steer_angle = 0.40;
    vehicle.corridor = {{0.20, 1.90, 0.55, 1.45}};
    return vehicle;
}

}  // namespace

TEST(HybridAstarTest, FindsForwardPathToFixedPoseInsideSingleStc) {
    fnr::HybridAstarConfig config;
    config.max_expansions = 20000;
    const auto result = fnr::HybridAstarPlanner(config).plan(straightVehicle(), {});
    ASSERT_TRUE(result.success) << result.reason;
    ASSERT_GE(result.path.size(), 2U);
    EXPECT_NEAR(result.path.front().pose.x, 0.50, 1e-12);
    EXPECT_NEAR(result.path.back().pose.x, 1.50, 1e-12);
    EXPECT_NEAR(result.path.back().pose.yaw, 0.0, 1e-12);
    EXPECT_EQ(result.path.back().type, uint8_t{0});
}

TEST(HybridAstarTest, ReportsNoPathWhenStaticObbCutsTheStc) {
    fnr::HybridAstarConfig config;
    config.max_expansions = 5000;
    fnr::StaticVehicleObstacle wall;
    wall.id = 9;
    wall.rear_axle_pose = {0.93, 1.00, 0.0};
    wall.rear_axle_to_center = 0.07;
    wall.disc_center = {1.00, 1.00, 0.0};
    wall.length = 0.10;
    wall.width = 0.90;
    const auto result = fnr::HybridAstarPlanner(config).plan(straightVehicle(), {wall});
    EXPECT_FALSE(result.success);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
