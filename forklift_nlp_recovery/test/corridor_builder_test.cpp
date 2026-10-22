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

TEST(CorridorBuilderSeedSearchTest, FindsCardinalOffsetWithoutChangingSearchRange) {
    MapParam param;
    param.rear_axle_to_center = 0.0;
    fnr::CorridorConfig config;
    config.vehicle_radius = 0.05;
    config.seed_search_step = 0.01;
    config.seed_search_max_offset = 0.10;
    ShelfBlock shelf;
    shelf.x = 1.0; shelf.y = 1.0; shelf.w = 0.02; shelf.h = 0.02;
    fnr::CorridorBuilder builder(param, {shelf}, config);
    fnr::Pose2d legal;
    std::string reason;
    ASSERT_FALSE(builder.pointLegal({0.99, 1.01, 0.0}));
    ASSERT_TRUE(builder.findLegalSeed({0.99, 1.01, 0.0}, legal, reason)) << reason;
    EXPECT_TRUE(builder.pointLegal(legal));
    EXPECT_LE(std::hypot(legal.x - 0.99, legal.y - 1.01), config.seed_search_max_offset + 1e-9);
}

TEST_F(CorridorBuilderTest, SelectsFixedRejoinDistanceWithoutBuildingCorridor) {
    std::vector<fnr::Pose2d> poses;
    std::vector<uint8_t> types;
    for (int i = 0; i <= 30; ++i) {
        poses.push_back({0.45 + 0.04 * i, param.corridor4_cy(), 0.0});
        types.push_back(i < 15 ? uint8_t{0} : uint8_t{1});
    }
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, types);
    const auto result = builder.build({0.46, param.corridor4_cy(), 0.0}, path, 0.0, 17);
    ASSERT_TRUE(result.success) << result.reason;
    EXPECT_TRUE(result.boxes.empty());
    EXPECT_TRUE(result.seeds.empty());
    EXPECT_NEAR(result.terminal.path_s, config.rejoin_distance, 1e-9);
    EXPECT_EQ(result.terminal.path_gen, 17);
    EXPECT_EQ(result.terminal.type, uint8_t{1});
}

TEST_F(CorridorBuilderTest, ClampsTerminalOnlyAtFrozenPathEnd) {
    const std::vector<fnr::Pose2d> poses{{0.5, param.corridor4_cy(), 0.0},
                                         {0.9, param.corridor4_cy(), 0.0}};
    const auto path = fnr::CorridorBuilder::parameterizePath(poses, {});
    const auto result = builder.build(poses.front(), path, 0.2, 3);
    ASSERT_TRUE(result.success) << result.reason;
    EXPECT_NEAR(result.terminal.path_s, path.back().s, 1e-12);
    EXPECT_NEAR(result.terminal.pose.x, poses.back().x, 1e-12);
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
