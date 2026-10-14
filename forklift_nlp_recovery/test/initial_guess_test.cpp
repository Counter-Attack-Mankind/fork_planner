#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "forklift_nlp_recovery/initial_guess.h"

namespace fnr = forklift_nlp_recovery;

namespace {

fnr::VehicleParameters parameters() {
    fnr::VehicleParameters value;
    value.length = 0.211;
    value.width = 0.191;
    value.rear_axle_to_center = 0.0735;
    value.disc_radius = 0.15;
    value.wheel_base = 0.143;
    value.max_steer_angle = 0.50;
    value.max_steer_rate = 0.25;
    value.nominal_speed = 0.20;
    value.max_speed = 0.60;
    value.max_accel = 1.0;
    value.max_decel = 1.0;
    return value;
}

fnr::NlpVehicleInput makeVehicle(int id, double y, const std::vector<uint8_t>& types,
                                 double start_offset_y = 0.0) {
    fnr::NlpVehicleInput vehicle;
    vehicle.id = id;
    vehicle.parameters = parameters();
    vehicle.path_gen = 100 + id;
    vehicle.path_s = 0.0;
    vehicle.initial_state.rear_axle_pose = {0.0, y + start_offset_y, 0.0};
    vehicle.initial_state.longitudinal_speed = 0.08;
    for (size_t i = 0; i < types.size(); ++i) {
        vehicle.original_path.push_back({{0.4 * static_cast<double>(i), y, 0.0},
                                         0.4 * static_cast<double>(i), types[i]});
    }
    vehicle.terminal.pose = vehicle.original_path.back().pose;
    vehicle.terminal.path_s = vehicle.original_path.back().s;
    vehicle.terminal.path_gen = vehicle.path_gen;
    vehicle.terminal.type = vehicle.original_path.back().type;
    vehicle.corridor_valid = true;
    vehicle.corridor_reason = "ok";
    vehicle.corridor.push_back({-0.25, 2.0, y - 0.4, y + 0.4});
    return vehicle;
}

fnr::NlpOptimizationInput makeInput(const std::vector<int>& ids) {
    fnr::NlpOptimizationInput input;
    input.valid = true;
    input.reason = "ok";
    input.source_mode = "test";
    input.frame_id = "map";
    input.participant_ids = ids;
    for (size_t i = 0; i < ids.size(); ++i) {
        input.vehicles.push_back(makeVehicle(ids[i], 0.8 * static_cast<double>(i),
                                             {0, 0, 0, 0}));
    }
    return input;
}

fnr::InitialGuessGenerator generator(size_t nodes = 9, double total_time = 8.0) {
    fnr::InitialGuessConfig config;
    config.node_count = nodes;
    config.total_time = total_time;
    return fnr::InitialGuessGenerator(config);
}

}  // namespace

TEST(InitialGuessTest, BuildsTwoAndThreeVehicleCommonTimeGridWithExactBounds) {
    for (const std::vector<int> ids : {std::vector<int>{2, 9},
                                       std::vector<int>{2, 5, 9}}) {
        const fnr::NlpOptimizationInput input = makeInput(ids);
        const fnr::NlpInitialGuess guess = generator().generate(input);
        ASSERT_TRUE(guess.generated) << guess.reason;
        ASSERT_EQ(guess.vehicles.size(), ids.size());
        ASSERT_EQ(guess.time_grid.size(), 9U);
        for (size_t i = 0; i < ids.size(); ++i) {
            EXPECT_EQ(guess.vehicles[i].id, ids[i]);
            ASSERT_EQ(guess.vehicles[i].nodes.size(), guess.time_grid.size());
            for (size_t node = 0; node < guess.time_grid.size(); ++node) {
                EXPECT_DOUBLE_EQ(guess.vehicles[i].nodes[node].time,
                                 guess.time_grid[node]);
            }
            EXPECT_DOUBLE_EQ(guess.vehicles[i].nodes.front().rear_axle_pose.x,
                             input.vehicles[i].initial_state.rear_axle_pose.x);
            EXPECT_DOUBLE_EQ(guess.vehicles[i].nodes.front().rear_axle_pose.y,
                             input.vehicles[i].initial_state.rear_axle_pose.y);
            EXPECT_DOUBLE_EQ(guess.vehicles[i].nodes.back().rear_axle_pose.x,
                             input.vehicles[i].terminal.pose.x);
            EXPECT_DOUBLE_EQ(guess.vehicles[i].nodes.back().rear_axle_pose.yaw,
                             input.vehicles[i].terminal.pose.yaw);
        }
    }
}

TEST(InitialGuessTest, PreservesOffPathStartAndInterpolatesYawAcrossPi) {
    fnr::NlpOptimizationInput input = makeInput({7});
    auto& vehicle = input.vehicles.front();
    vehicle.initial_state.rear_axle_pose = {0.0, -0.2, 3.10};
    vehicle.original_path[0].pose.yaw = 3.10;
    vehicle.original_path[1].pose.yaw = 3.13;
    vehicle.original_path[2].pose.yaw = -3.13;
    vehicle.original_path[3].pose.yaw = -3.10;
    vehicle.terminal.pose = vehicle.original_path.back().pose;
    vehicle.corridor = {{-0.3, 2.0, -0.5, 0.5}};

    const fnr::NlpInitialGuess guess = generator(11, 10.0).generate(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    const auto& nodes = guess.vehicles.front().nodes;
    EXPECT_DOUBLE_EQ(nodes.front().rear_axle_pose.y, -0.2);
    EXPECT_DOUBLE_EQ(nodes.back().rear_axle_pose.yaw, -3.10);
    EXPECT_GT(std::abs(nodes[nodes.size() / 2].rear_axle_pose.yaw), 3.0);
}

TEST(InitialGuessTest, KeepsForwardReverseSignsAndStopsAtSampledCusp) {
    fnr::NlpOptimizationInput input = makeInput({3});
    input.vehicles.front() = makeVehicle(3, 0.0, {0, 0, 1, 1});
    const fnr::NlpInitialGuess guess = generator(13, 12.0).generate(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    bool found_positive = false;
    bool found_negative = false;
    bool checked_transition = false;
    const auto& nodes = guess.vehicles.front().nodes;
    for (size_t i = 0; i < nodes.size(); ++i) {
        found_positive = found_positive || nodes[i].signed_speed > 0.0;
        found_negative = found_negative || nodes[i].signed_speed < 0.0;
        if (i + 1 < nodes.size() && nodes[i].path_type != nodes[i + 1].path_type) {
            EXPECT_DOUBLE_EQ(nodes[i].signed_speed, 0.0);
            EXPECT_DOUBLE_EQ(nodes[i + 1].signed_speed, 0.0);
            checked_transition = true;
        }
    }
    EXPECT_TRUE(found_positive);
    EXPECT_TRUE(found_negative);
    EXPECT_TRUE(checked_transition);
}

TEST(InitialGuessTest, AssignsOrderedConnectedAabbsWithoutEnvelope) {
    fnr::NlpOptimizationInput input = makeInput({11});
    auto& vehicle = input.vehicles.front();
    vehicle.corridor = {{-0.2, 0.55, -0.3, 0.3},
                        {0.45, 1.05, -0.3, 0.3},
                        {0.95, 1.5, -0.3, 0.3}};
    const fnr::NlpInitialGuess guess = generator(13, 12.0).generate(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    int previous = 0;
    for (const auto& node : guess.vehicles.front().nodes) {
        EXPECT_GE(node.aabb_index, previous);
        EXPECT_LE(node.aabb_index, previous + 1);
        previous = node.aabb_index;
    }
    EXPECT_EQ(guess.vehicles.front().nodes.back().aabb_index, 2);
}

TEST(InitialGuessTest, AssignsAnLShapedConnectedCorridorInPathOrder) {
    fnr::NlpOptimizationInput input = makeInput({13});
    auto& vehicle = input.vehicles.front();
    vehicle.original_path = {{{0.0, 0.0, 0.0}, 0.0, 0},
                             {{0.4, 0.0, 0.0}, 0.4, 0},
                             {{0.8, 0.0, 0.0}, 0.8, 0},
                             {{0.8, 0.4, M_PI_2}, 1.2, 0}};
    vehicle.terminal.pose = vehicle.original_path.back().pose;
    vehicle.terminal.path_s = 1.2;
    vehicle.terminal.type = 0;
    vehicle.corridor = {{-0.2, 0.65, -0.3, 0.3},
                        {0.50, 1.10, -0.25, 0.28},
                        {0.65, 1.05, 0.15, 0.60}};
    const fnr::NlpInitialGuess guess = generator(17, 12.0).generate(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    const auto& nodes = guess.vehicles.front().nodes;
    EXPECT_EQ(nodes.front().aabb_index, 0);
    EXPECT_EQ(nodes.back().aabb_index, 2);
    for (const auto& node : nodes) {
        const fnr::Aabb& box = vehicle.corridor[static_cast<size_t>(node.aabb_index)];
        EXPECT_GE(node.disc_center.x, box.min_x - 1e-8);
        EXPECT_LE(node.disc_center.x, box.max_x + 1e-8);
        EXPECT_GE(node.disc_center.y, box.min_y - 1e-8);
        EXPECT_LE(node.disc_center.y, box.max_y + 1e-8);
    }
}

TEST(InitialGuessTest, RejectsInvalidInputAndIllegalAabbAssignment) {
    fnr::NlpOptimizationInput invalid = makeInput({1});
    invalid.valid = false;
    EXPECT_FALSE(generator().generate(invalid).generated);

    fnr::NlpOptimizationInput uncovered = makeInput({1});
    uncovered.vehicles.front().corridor = {{5.0, 6.0, 5.0, 6.0}};
    const fnr::NlpInitialGuess guess = generator().generate(uncovered);
    EXPECT_FALSE(guess.generated);
    EXPECT_NE(guess.reason.find("AABB assignment failed"), std::string::npos);
}

TEST(InitialGuessTest, ReportsParticipantAndSelectedStaticCollisions) {
    fnr::NlpOptimizationInput input = makeInput({2, 8});
    input.vehicles[1] = makeVehicle(8, 0.10, {0, 0, 0, 0});
    fnr::StaticVehicleObstacle obstacle;
    obstacle.id = 42;
    obstacle.disc_center = {0.50, 0.0, 0.0};
    obstacle.disc_radius = 0.15;
    input.static_vehicles.push_back(obstacle);
    const fnr::NlpInitialGuess guess = generator().generate(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    EXPECT_GT(guess.diagnostics.participant_collision_nodes, 0U);
    EXPECT_GT(guess.diagnostics.static_collision_nodes, 0U);
    EXPECT_LT(guess.diagnostics.minimum_participant_clearance, 0.0);
    EXPECT_LT(guess.diagnostics.minimum_static_clearance, 0.0);
}

TEST(InitialGuessTest, SavesConsecutiveCasesWithoutOverwriteAndMatchesMemory) {
    const fnr::NlpOptimizationInput input = makeInput({2, 9});
    const fnr::NlpInitialGuess guess = generator(7, 6.0).generate(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    fnr::OptimizationInputBuildResult build;
    build.complete = true;
    build.reason = "ok";
    build.input = input;
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path root = std::filesystem::temp_directory_path() /
        ("forklift_initial_guess_test_" + std::to_string(nonce));

    const fnr::CaseSaveResult first = fnr::saveInitialGuessCase(build, guess, root.string());
    const fnr::CaseSaveResult second = fnr::saveInitialGuessCase(build, guess, root.string());
    ASSERT_TRUE(first.success) << first.reason;
    ASSERT_TRUE(second.success) << second.reason;
    EXPECT_EQ(first.case_id, "case_0001");
    EXPECT_EQ(second.case_id, "case_0002");
    EXPECT_NE(first.case_directory, second.case_directory);
    EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(first.case_directory) /
                                        "optimization_input.yaml"));
    const std::filesystem::path csv_path = std::filesystem::path(first.case_directory) /
                                           "initial_guess.csv";
    std::ifstream csv(csv_path);
    ASSERT_TRUE(csv.good());
    const std::string text((std::istreambuf_iterator<char>(csv)),
                           std::istreambuf_iterator<char>());
    EXPECT_NE(text.find("vehicle_id,node_index,time"), std::string::npos);
    EXPECT_NE(text.find("case_0001,"), std::string::npos);
    size_t line_count = static_cast<size_t>(std::count(text.begin(), text.end(), '\n'));
    EXPECT_EQ(line_count, 1U + guess.vehicles.size() * guess.node_count);
    std::istringstream rows(text);
    std::string header;
    std::string first_row;
    ASSERT_TRUE(static_cast<bool>(std::getline(rows, header)));
    ASSERT_TRUE(static_cast<bool>(std::getline(rows, first_row)));
    std::vector<std::string> fields;
    std::istringstream field_stream(first_row);
    std::string field;
    while (std::getline(field_stream, field, ',')) fields.push_back(field);
    ASSERT_GE(fields.size(), 17U);
    const auto& memory_node = guess.vehicles.front().nodes.front();
    EXPECT_EQ(std::stoi(fields[2]), guess.vehicles.front().id);
    EXPECT_EQ(std::stoul(fields[3]), memory_node.index);
    EXPECT_NEAR(std::stod(fields[4]), memory_node.time, 1e-9);
    EXPECT_NEAR(std::stod(fields[7]), memory_node.rear_axle_pose.x, 1e-9);
    EXPECT_NEAR(std::stod(fields[10]), memory_node.signed_speed, 1e-9);
    EXPECT_EQ(std::stoi(fields[14]), memory_node.aabb_index);
    std::filesystem::remove_all(root);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
