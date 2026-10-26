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

TEST_F(CorridorBuilderTest, UsesSingleDiscInflationAndBodyCenter) {
    const double expected = std::hypot(param.vehicle_length * 0.5, param.vehicle_width * 0.5);
    EXPECT_NEAR(builder.vehicleRadius(), expected, 1e-12);
    const fnr::Pose2d rear{1.0, 2.0, M_PI * 0.5};
    const fnr::Pose2d center = builder.bodyCenter(rear);
    EXPECT_NEAR(center.x, rear.x, 1e-12);
    EXPECT_NEAR(center.y, rear.y + param.rear_axle_to_center, 1e-12);
}

TEST_F(CorridorBuilderTest, ExpandsZeroAreaSeedIndependentlyAndLegally) {
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

TEST_F(CorridorBuilderTest, FindsFirstExitAndBuildsOneInsetStc) {
    std::vector<fnr::Pose2d> poses;
    std::vector<uint8_t> types;
    for (int i = 0; i <= 60; ++i) {
        poses.push_back({0.45 + 0.04 * i, param.corridor4_cy(), 0.0});
        types.push_back(i < 15 ? uint8_t{0} : uint8_t{1});
    }
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, types);
    const auto result = builder.build({0.46, param.corridor4_cy(), 0.0}, path, 0.0, 17);
    ASSERT_TRUE(result.success) << result.reason;
    ASSERT_EQ(result.boxes.size(), 1U);
    ASSERT_EQ(result.seeds.size(), 1U);
    EXPECT_GT(result.terminal.path_s, 0.0);
    EXPECT_LT(result.terminal.path_s, path.back().s);
    EXPECT_EQ(result.terminal.path_gen, 17);
    EXPECT_EQ(result.terminal.type, uint8_t{1});
}

TEST_F(CorridorBuilderTest, RejectsPathThatNeverExitsStc) {
    const std::vector<fnr::Pose2d> poses{{0.5, param.corridor4_cy(), 0.0},
                                         {0.9, param.corridor4_cy(), 0.0}};
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, {});
    const auto result = builder.build(poses.front(), path, 0.2, 3);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.reason.find("no body-center exit"), std::string::npos);
}

TEST_F(CorridorBuilderTest, RejectsPathProgressOutsideFrozenPath) {
    const std::vector<fnr::Pose2d> poses{{0.5, 1.0, 0.0}, {1.0, 1.0, 0.0}};
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, {});
    EXPECT_FALSE(builder.build(poses.front(), path, -0.1, 2).success);
    EXPECT_FALSE(builder.build(poses.front(), path, 0.6, 2).success);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
