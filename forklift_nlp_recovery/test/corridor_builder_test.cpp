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

TEST_F(CorridorBuilderTest, UsesRearAxleCircumscribedRadius) {
    const double expected = std::hypot(param.vehicle_length - param.rear_hang,
                                       param.vehicle_width * 0.5);
    EXPECT_NEAR(builder.vehicleRadius(), expected, 1e-12);
    EXPECT_NEAR(builder.clearance(), expected + config.safety_margin, 1e-12);
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
    EXPECT_NEAR(result.seeds.front().x, actual_start.x, 1e-12);
    EXPECT_NEAR(result.seeds.front().y, actual_start.y, 1e-12);
    EXPECT_EQ(result.terminal.path_gen, 17);
    EXPECT_NEAR(result.terminal.path_s, config.rejoin_distance, 1e-9);
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
    EXPECT_TRUE(builder.pointLegal(result.terminal.pose));
    EXPECT_NE(result.reason.find("shortened"), std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
