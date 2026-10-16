#include "forklift_nlp_recovery/coordinate_mapper.h"

#include <cmath>

#include <gtest/gtest.h>

namespace forklift_nlp_recovery {
namespace {

CorridorBuilder makeBuilder(const MapParam& map_param,
                            const std::vector<ShelfBlock>& shelves = {}) {
    CorridorConfig config;
    config.vehicle_radius = 0.15;
    config.shelf_inflation_reduction = 0.05;
    config.boundary_inset_reduction = 0.05;
    return CorridorBuilder(map_param, shelves, config);
}

TEST(CoordinateMapperTest, PreservesSourceMetadataAndMapsWithoutOverwritingRearPose) {
    MapParam map_param;
    const CorridorBuilder builder = makeBuilder(map_param);
    const CoordinateMapper mapper(map_param, builder);
    const std::vector<PathPoint> rear_path{
        {{1.0, 2.0, 0.0}, 0.0, 0},
        {{1.1, 2.1, M_PI * 0.5}, 0.2, 1}};

    const auto mapped = mapper.mapPath(rear_path);
    ASSERT_EQ(mapped.size(), 2u);
    EXPECT_EQ(mapped[1].source_index, 1u);
    EXPECT_DOUBLE_EQ(mapped[1].source_s, 0.2);
    EXPECT_EQ(mapped[1].path_type, 1u);
    EXPECT_DOUBLE_EQ(mapped[0].rear_axle_pose.x, rear_path[0].pose.x);
    EXPECT_DOUBLE_EQ(mapped[0].rear_axle_pose.y, rear_path[0].pose.y);
    EXPECT_NEAR(mapped[0].body_center_pose.x,
                1.0 + map_param.rear_axle_to_center, 1e-12);
    EXPECT_NEAR(mapped[0].body_center_pose.y, 2.0, 1e-12);
    EXPECT_NEAR(mapped[1].body_center_pose.x, 1.1, 1e-12);
    EXPECT_NEAR(mapped[1].body_center_pose.y,
                2.1 + map_param.rear_axle_to_center, 1e-12);
}

TEST(CoordinateMapperTest, DistinguishesInflatedShelfCollisionFromAabbGap) {
    MapParam map_param;
    const std::vector<ShelfBlock> shelves{{"test", 1.0, 1.0, 0.2, 0.2, true, false}};
    const CorridorBuilder builder = makeBuilder(map_param, shelves);
    const CoordinateMapper mapper(map_param, builder);
    const std::vector<Aabb> corridor{{0.2, 0.8, 0.2, 0.8}};

    PathPoint collision{{0.95 - map_param.rear_axle_to_center, 1.1, 0.0}, 0.0, 0};
    const auto collision_diagnostic = mapper.diagnosePoint(
        mapper.mapPoint(collision, 0), corridor);
    EXPECT_TRUE(collision_diagnostic.disc_shelf_collision);
    EXPECT_TRUE(collision_diagnostic.containing_aabbs.empty());

    PathPoint gap{{2.0 - map_param.rear_axle_to_center, 2.0, 0.0}, 1.0, 0};
    const auto gap_diagnostic = mapper.diagnosePoint(mapper.mapPoint(gap, 1), corridor);
    EXPECT_FALSE(gap_diagnostic.disc_shelf_collision);
    EXPECT_TRUE(gap_diagnostic.containing_aabbs.empty());
}

TEST(CoordinateMapperTest, SamplesBetweenSourceNodes) {
    MapParam map_param;
    const CorridorBuilder builder = makeBuilder(map_param);
    const CoordinateMapper mapper(map_param, builder);
    const std::vector<PathPoint> path{
        {{0.5, 0.5, 0.0}, 0.0, 0}, {{0.6, 0.5, 0.0}, 0.1, 0}};
    const auto swept = mapper.diagnoseSweptPath(path, {}, 0.02);
    EXPECT_GE(swept.size(), 4u);
    for (const auto& sample : swept) {
        EXPECT_EQ(sample.segment_begin_index, 0u);
        EXPECT_GT(sample.segment_ratio, 0.0);
        EXPECT_LT(sample.segment_ratio, 1.0);
    }
}

}  // namespace
}  // namespace forklift_nlp_recovery
