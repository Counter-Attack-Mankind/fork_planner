#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "forklift_nlp_recovery/initial_guess.h"

namespace fnr = forklift_nlp_recovery;

namespace {

fnr::VehicleParameters parameters() {
    fnr::VehicleParameters value;
    value.length = 0.20;
    value.width = 0.16;
    value.rear_axle_to_center = 0.05;
    value.disc_radius = 0.05;
    value.wheel_base = 0.14;
    value.max_steer_angle = 0.6;
    value.max_steer_rate = 0.8;
    value.max_speed = 0.5;
    value.max_accel = 0.4;
    value.max_decel = 0.4;
    return value;
}

fnr::NlpVehicleInput vehicle(int id, double y, uint8_t second_type = 0) {
    fnr::NlpVehicleInput value;
    value.id = id;
    value.parameters = parameters();
    value.path_gen = 10 + id;
    value.path_s = 0.0;
    value.initial_state.rear_axle_pose = {1.0, y, 0.0};
    value.initial_state.longitudinal_speed = 0.2;
    value.original_path = {{{1.0, y, 0.0}, 0.0, 0},
                           {{1.5, y, 0.0}, 0.5, 0},
                           {{2.0, y, second_type == 1 ? M_PI : 0.0}, 1.0, second_type}};
    value.terminal = {value.original_path.back().pose, 1.0, value.path_gen, second_type};
    value.corridor_valid = true;
    value.corridor_reason = "terminal selected";
    return value;
}

fnr::NlpOptimizationInput inputFor(const std::vector<fnr::NlpVehicleInput>& vehicles) {
    fnr::NlpOptimizationInput input;
    input.valid = true;
    input.reason = "ok";
    input.source_mode = "test";
    input.frame_id = "map";
    input.vehicles = vehicles;
    for (const auto& value : vehicles) input.participant_ids.push_back(value.id);
    return input;
}

fnr::CorridorBuilder builder(const std::vector<ShelfBlock>& shelves = {}) {
    MapParam map;
    fnr::CorridorConfig config;
    config.vehicle_radius = 0.05;
    config.seed_search_step = 0.01;
    config.seed_search_max_offset = 0.10;
    return fnr::CorridorBuilder(map, shelves, config);
}

}  // namespace

TEST(InitialGuessTest, UsesFixed51NodeCommonTimeGridAndOneToOneAabbs) {
    fnr::CorridorBuilder corridor = builder();
    fnr::InitialGuessConfig config;
    config.node_count = 51;
    fnr::NlpOptimizationInput input = inputFor({vehicle(2, 1.0), vehicle(9, 1.5)});
    const auto prepared = fnr::InitialGuessGenerator(config, corridor).prepare(input);
    ASSERT_TRUE(prepared.actual_guess.generated) << prepared.actual_guess.reason;
    EXPECT_EQ(prepared.actual_guess.node_count, 51U);
    EXPECT_EQ(prepared.actual_guess.time_grid.size(), 51U);
    ASSERT_EQ(input.vehicles.size(), 2U);
    for (size_t vehicle_index = 0; vehicle_index < input.vehicles.size(); ++vehicle_index) {
        EXPECT_EQ(prepared.actual_guess.vehicles[vehicle_index].nodes.size(), 51U);
        EXPECT_EQ(input.vehicles[vehicle_index].corridor.size(), 51U);
        EXPECT_EQ(input.vehicles[vehicle_index].corridor_seeds.size(), 51U);
        for (size_t node = 0; node < 51; ++node)
            EXPECT_EQ(prepared.actual_guess.vehicles[vehicle_index].nodes[node].aabb_index,
                      static_cast<int>(node));
    }
    EXPECT_DOUBLE_EQ(prepared.actual_guess.vehicles.front().nodes.front().signed_speed, 0.0);
    EXPECT_DOUBLE_EQ(prepared.actual_guess.vehicles.front().nodes.back().signed_speed, 0.0);
}

TEST(InitialGuessTest, ShorterProfileArrivesEarlyAndWaitsOnCommonTimeGrid) {
    fnr::CorridorBuilder corridor = builder();
    fnr::InitialGuessConfig config;
    fnr::NlpVehicleInput slow = vehicle(1, 1.0);
    fnr::NlpVehicleInput fast = vehicle(3, 1.5);
    slow.parameters.max_speed = 0.2;
    fast.parameters.max_speed = 0.6;
    fnr::NlpOptimizationInput input = inputFor({slow, fast});
    const auto prepared = fnr::InitialGuessGenerator(config, corridor).prepare(input);
    ASSERT_TRUE(prepared.actual_guess.generated) << prepared.actual_guess.reason;
    EXPECT_GT(prepared.actual_guess.vehicles[0].diagnostics.estimated_travel_time,
              prepared.actual_guess.vehicles[1].diagnostics.estimated_travel_time);
    for (size_t i = 0; i < input.vehicles.size(); ++i) {
        EXPECT_NEAR(prepared.actual_guess.vehicles[i].nodes.back().rear_axle_pose.x,
                    input.vehicles[i].terminal.pose.x, 1e-12);
        EXPECT_NEAR(prepared.actual_guess.vehicles[i].nodes.back().time,
                    prepared.actual_guess.total_time, 1e-12);
    }
    const auto& fast_nodes = prepared.actual_guess.vehicles[1].nodes;
    const auto first_waiting = std::find_if(fast_nodes.begin(), fast_nodes.end(),
        [&](const fnr::InitialGuessNode& node) {
            return node.index > 0 &&
                std::abs(node.rear_axle_pose.x - fast.terminal.pose.x) < 1e-12 &&
                std::abs(node.rear_axle_pose.y - fast.terminal.pose.y) < 1e-12;
        });
    ASSERT_NE(first_waiting, fast_nodes.end());
    ASSERT_LT(first_waiting->index, fast_nodes.size() - 1);
    const double waiting_steer = first_waiting->front_steer;
    for (auto node = first_waiting; node != fast_nodes.end(); ++node) {
        EXPECT_NEAR(node->rear_axle_pose.x, fast.terminal.pose.x, 1e-12);
        EXPECT_NEAR(node->rear_axle_pose.y, fast.terminal.pose.y, 1e-12);
        EXPECT_NEAR(node->rear_axle_pose.yaw, fast.terminal.pose.yaw, 1e-12);
        EXPECT_NEAR(node->path_s, fast.terminal.path_s, 1e-12);
        EXPECT_EQ(node->path_type, fast.terminal.type);
        EXPECT_DOUBLE_EQ(node->signed_speed, 0.0);
        EXPECT_DOUBLE_EQ(node->acceleration, 0.0);
        EXPECT_DOUBLE_EQ(node->steer_rate, 0.0);
        EXPECT_DOUBLE_EQ(node->front_steer, waiting_steer);
    }
}

TEST(InitialGuessTest, OffsetsOnlyIllegalInternalSeedsAndRecomputesState) {
    ShelfBlock shelf;
    shelf.x = 1.49; shelf.y = 0.99; shelf.w = 0.02; shelf.h = 0.02;
    fnr::CorridorBuilder corridor = builder({shelf});
    fnr::NlpOptimizationInput input = inputFor({vehicle(4, 1.0)});
    const auto prepared = fnr::InitialGuessGenerator({}, corridor).prepare(input);
    ASSERT_TRUE(prepared.actual_guess.generated) << prepared.actual_guess.reason;
    const auto& original = prepared.original_guess.vehicles.front();
    const auto& actual = prepared.actual_guess.vehicles.front();
    EXPECT_GT(actual.diagnostics.shifted_seed_nodes, 0U);
    EXPECT_EQ(actual.nodes.front().rear_axle_pose.x, original.nodes.front().rear_axle_pose.x);
    EXPECT_EQ(actual.nodes.back().rear_axle_pose.x, original.nodes.back().rear_axle_pose.x);
    for (size_t node = 0; node < actual.nodes.size(); ++node) {
        EXPECT_NEAR(actual.nodes[node].rear_axle_pose.yaw,
                    original.nodes[node].rear_axle_pose.yaw, 1e-12);
        EXPECT_TRUE(corridor.pointLegal(actual.nodes[node].disc_center));
    }
}

TEST(InitialGuessTest, RejectsIllegalFixedEndpointWithoutOffset) {
    fnr::CorridorBuilder corridor = builder();
    fnr::NlpVehicleInput source = vehicle(5, 1.0);
    source.initial_state.rear_axle_pose.x = 0.0;
    fnr::NlpOptimizationInput input = inputFor({source});
    const auto prepared = fnr::InitialGuessGenerator({}, corridor).prepare(input);
    EXPECT_FALSE(prepared.actual_guess.generated);
    EXPECT_FALSE(input.valid);
    EXPECT_NE(prepared.actual_guess.reason.find("fixed endpoint seed is illegal"), std::string::npos);
}

TEST(InitialGuessTest, SavesOnlyPreOffsetAndActualCsvFiles) {
    fnr::CorridorBuilder corridor = builder();
    fnr::NlpOptimizationInput input = inputFor({vehicle(6, 1.0)});
    const auto prepared = fnr::InitialGuessGenerator({}, corridor).prepare(input);
    ASSERT_TRUE(prepared.actual_guess.generated);
    fnr::OptimizationInputBuildResult build;
    build.complete = true;
    build.input = input;
    build.reason = "ok";
    const auto root = std::filesystem::temp_directory_path() / "fnr_initial_guess_test";
    std::filesystem::remove_all(root);
    const auto saved = fnr::saveInitialGuessCase(build, prepared.original_guess,
        prepared.actual_guess, root.string());
    ASSERT_TRUE(saved.success) << saved.reason;
    EXPECT_TRUE(std::filesystem::is_regular_file(
        std::filesystem::path(saved.case_directory) / "original_initial_guess.csv"));
    EXPECT_TRUE(std::filesystem::is_regular_file(
        std::filesystem::path(saved.case_directory) / "initial_guess.csv"));
    EXPECT_FALSE(std::filesystem::exists(
        std::filesystem::path(saved.case_directory) / "repaired_initial_guess.csv"));
    std::filesystem::remove_all(root);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
