#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "forklift_map/forklift_map.h"
#include "forklift_nlp_recovery/corridor_builder.h"

namespace fnr = forklift_nlp_recovery;

class CorridorBuilderTest : public ::testing::Test {
protected:
    CorridorBuilderTest() : map(param), builder(param, map.shelf_blocks(), config) {}

    MapParam param;
    ForkliftMap map;
    fnr::CorridorConfig config;
    fnr::CorridorBuilder builder;
};

TEST_F(CorridorBuilderTest, UsesBodyCenteredFallbackRadius) {
    const double expected = std::hypot(param.vehicle_length * 0.5,
                                       param.vehicle_width * 0.5);
    EXPECT_NEAR(builder.vehicleRadius(), expected, 1e-12);
    EXPECT_NEAR(builder.boundaryClearance(), expected + config.safety_margin, 1e-12);
    EXPECT_NEAR(builder.shelfInflation(), expected + config.safety_margin, 1e-12);
}

TEST_F(CorridorBuilderTest, AppliesRadiusOverrideAndShelfInflationReduction) {
    fnr::CorridorConfig configured = config;
    configured.vehicle_radius = 0.12;
    configured.safety_margin = 0.01;
    configured.shelf_inflation_reduction = 0.03;
    const fnr::CorridorBuilder configured_builder(
        param, map.shelf_blocks(), configured);
    EXPECT_NEAR(configured_builder.vehicleRadius(), 0.12, 1e-12);
    EXPECT_NEAR(configured_builder.boundaryClearance(), 0.13, 1e-12);
    EXPECT_NEAR(configured_builder.shelfInflation(), 0.10, 1e-12);
    ASSERT_FALSE(configured_builder.originalShelves().empty());
    ASSERT_GT(configured_builder.originalShelves().size(), 2U);
    const auto& shelf = configured_builder.originalShelves()[2];
    const double shelf_mid_y = (shelf.min_y + shelf.max_y) * 0.5;
    EXPECT_FALSE(configured_builder.pointLegal(
        {shelf.min_x - 0.099, shelf_mid_y, 0.0}));
    EXPECT_TRUE(configured_builder.pointLegal(
        {shelf.min_x - 0.101, shelf_mid_y, 0.0}));

    configured.shelf_inflation_reduction = 1.0;
    const fnr::CorridorBuilder zero_inflation_builder(
        param, map.shelf_blocks(), configured);
    EXPECT_DOUBLE_EQ(zero_inflation_builder.shelfInflation(), 0.0);
}

TEST_F(CorridorBuilderTest, ConvertsRearAxlePoseToBodyCenter) {
    const fnr::Pose2d x_aligned{1.0, 2.0, 0.0};
    const fnr::Pose2d x_center = builder.bodyCenter(x_aligned);
    EXPECT_NEAR(x_center.x, x_aligned.x + param.rear_axle_to_center, 1e-12);
    EXPECT_NEAR(x_center.y, x_aligned.y, 1e-12);
    EXPECT_NEAR(x_center.yaw, x_aligned.yaw, 1e-12);

    const fnr::Pose2d y_aligned{1.0, 2.0, M_PI * 0.5};
    const fnr::Pose2d y_center = builder.bodyCenter(y_aligned);
    EXPECT_NEAR(y_center.x, y_aligned.x, 1e-12);
    EXPECT_NEAR(y_center.y, y_aligned.y + param.rear_axle_to_center, 1e-12);
}

TEST_F(CorridorBuilderTest, ExpansionRemainsLegalAndContainsSeed) {
    const fnr::Pose2d seed{1.25, param.corridor4_cy(), 0.0};
    fnr::Aabb box;
    std::string reason;
    ASSERT_TRUE(builder.generateAabb(seed, box, reason)) << reason;
    EXPECT_TRUE(builder.aabbLegal(box));
    EXPECT_LE(box.min_x, seed.x);
    EXPECT_GE(box.max_x, seed.x);
    EXPECT_LE(box.min_y, seed.y);
    EXPECT_GE(box.max_y, seed.y);
}

TEST_F(CorridorBuilderTest, SingleDiscRejectsNarrowRowTwoGap) {
    const fnr::Pose2d row_two_gap{1.25, (param.y4() + param.y5()) * 0.5, 0.0};
    EXPECT_FALSE(builder.pointLegal(row_two_gap));
}

TEST_F(CorridorBuilderTest, BuildsConnectedCorridorAndPreservesEndpointMetadata) {
    std::vector<fnr::Pose2d> poses;
    std::vector<uint8_t> types;
    for (int i = 0; i <= 30; ++i) {
        poses.push_back({0.45 + 0.04 * i, param.corridor4_cy(), 0.0});
        types.push_back(i < 15 ? uint8_t{0} : uint8_t{1});
    }
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, types);
    const fnr::Pose2d actual_start{0.46, param.corridor4_cy() + 0.01, 0.02};
    const auto result = builder.build(actual_start, path, 0.0, 17);
    ASSERT_TRUE(result.success) << result.reason;
    ASSERT_FALSE(result.boxes.empty());
    const fnr::Pose2d expected_center = builder.bodyCenter(actual_start);
    EXPECT_NEAR(result.seeds.front().x, expected_center.x, 1e-12);
    EXPECT_NEAR(result.seeds.front().y, expected_center.y, 1e-12);
    EXPECT_EQ(result.terminal.path_gen, 17);
    EXPECT_NEAR(result.terminal.path_s, config.rejoin_distance, 1e-9);
    EXPECT_NEAR(result.terminal.pose.x, 1.25, 1e-9);
    EXPECT_NEAR(result.terminal.pose.y, param.corridor4_cy(), 1e-9);
    EXPECT_NEAR(result.terminal.pose.yaw, 0.0, 1e-12);
    EXPECT_EQ(result.terminal.type, uint8_t{1});
    for (size_t i = 0; i < result.boxes.size(); ++i) {
        EXPECT_TRUE(builder.aabbLegal(result.boxes[i]));
        if (i > 0) {
            EXPECT_TRUE(builder.connected(result.boxes[i - 1], result.boxes[i]));
        }
    }
}

TEST_F(CorridorBuilderTest, DoesNotMoveIllegalActualStart) {
    const std::vector<fnr::Pose2d> poses{{0.5, param.corridor4_cy(), 0.0},
                                         {1.5, param.corridor4_cy(), 0.0}};
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, {});
    const fnr::Pose2d illegal_start{0.05, param.corridor4_cy(), 0.0};
    const auto result = builder.build(illegal_start, path, 0.0, 3);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.reason.find("actual rear-axle start"), std::string::npos);
}

TEST_F(CorridorBuilderTest, ShortensRejoinAtLastLegalPathIntersection) {
    const std::vector<fnr::Pose2d> poses{{0.80, param.corridor4_cy(), M_PI},
                                         {0.50, param.corridor4_cy(), M_PI},
                                         {0.20, param.corridor4_cy(), M_PI},
                                         {0.10, param.corridor4_cy(), M_PI}};
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, {});
    const auto result = builder.build(poses.front(), path, 0.0, 4);
    ASSERT_TRUE(result.success) << result.reason;
    EXPECT_GE(result.terminal.path_s, config.min_rejoin_distance);
    EXPECT_LT(result.terminal.path_s, path.back().s);
    EXPECT_TRUE(builder.pointLegal(builder.bodyCenter(result.terminal.pose)));
    EXPECT_NE(result.reason.find("shortened"), std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
