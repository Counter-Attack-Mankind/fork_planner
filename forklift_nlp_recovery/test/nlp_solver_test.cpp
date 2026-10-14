#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "forklift_nlp_recovery/nlp_solver.h"

namespace fnr = forklift_nlp_recovery;

namespace {

fnr::VehicleParameters parameters() {
    fnr::VehicleParameters value;
    value.length = 0.211;
    value.width = 0.191;
    value.rear_axle_to_center = 0.0735;
    value.disc_radius = 0.08;
    value.wheel_base = 0.143;
    value.max_steer_angle = 0.50;
    value.max_steer_rate = 0.50;
    value.nominal_speed = 0.30;
    value.max_speed = 0.60;
    value.max_accel = 1.0;
    value.max_decel = 1.0;
    return value;
}

fnr::NlpVehicleInput vehicle(int id, double y, bool multiple_boxes = false) {
    fnr::NlpVehicleInput result;
    result.id = id;
    result.parameters = parameters();
    result.path_gen = 100 + id;
    result.initial_state.rear_axle_pose = {0.0, y, 0.0};
    result.path_s = 0.0;
    for (int index = 0; index <= 10; ++index) {
        const double x = 0.1 * index;
        result.original_path.push_back({{x, y, 0.0}, x, 0});
    }
    result.terminal.pose = result.original_path.back().pose;
    result.terminal.path_s = result.original_path.back().s;
    result.terminal.path_gen = result.path_gen;
    result.terminal.type = 0;
    result.corridor_valid = true;
    result.corridor_reason = "ok";
    if (multiple_boxes) {
        result.corridor = {{-0.15, 0.62, y - 0.2, y + 0.2},
                           {0.38, 1.2, y - 0.2, y + 0.2}};
    } else {
        result.corridor = {{-0.15, 1.2, y - 0.2, y + 0.2}};
    }
    return result;
}

fnr::NlpOptimizationInput inputFor(const std::vector<int>& ids,
                                   bool multiple_boxes = false) {
    fnr::NlpOptimizationInput input;
    input.valid = true;
    input.reason = "ok";
    input.source_mode = "test";
    input.frame_id = "map";
    input.participant_ids = ids;
    for (size_t index = 0; index < ids.size(); ++index) {
        input.vehicles.push_back(vehicle(ids[index], 0.5 * index,
                                         multiple_boxes && index == 0));
    }
    return input;
}

fnr::NlpInitialGuess initialGuess(const fnr::NlpOptimizationInput& input) {
    fnr::InitialGuessConfig config;
    config.automatic = false;
    config.node_count = 11;
    config.total_time = 4.0;
    return fnr::InitialGuessGenerator(config).generate(input);
}

fnr::NlpSolver solver() {
    fnr::NlpSolverConfig config;
    config.min_time_scale = 0.5;
    config.max_time_scale = 2.0;
    config.max_iterations = 400;
    config.print_level = 0;
    config.validation_substeps = 5;
    config.validation_tolerance = 2e-4;
    return fnr::NlpSolver(config);
}

}  // namespace

TEST(NlpSolverTest, SolvesTwoVehiclesWithSingleAndMultipleAabbs) {
    const fnr::NlpOptimizationInput input = inputFor({2, 9}, true);
    const fnr::NlpInitialGuess guess = initialGuess(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    ASSERT_TRUE(guess.vehicles.front().aabb_assignment_valid);
    const fnr::NlpSolveResult result = solver().solve(input, guess);
    EXPECT_TRUE(result.problem_built) << result.reason;
    EXPECT_TRUE(result.solver_success) << result.reason;
    EXPECT_TRUE(result.validation_passed) << result.validation.reason;
    ASSERT_EQ(result.vehicles.size(), 2U);
    EXPECT_EQ(result.vehicles[0].id, 2);
    EXPECT_EQ(result.vehicles[1].id, 9);
    EXPECT_EQ(result.node_count, guess.node_count);
    EXPECT_GT(result.total_time, 0.0);
    std::cout << "two_vehicle_result T=" << result.total_time
              << " iterations=" << result.statistics.iterations
              << " wall=" << result.statistics.wall_time
              << " max_violation=" << result.statistics.max_constraint_violation
              << " min_clear=" << result.validation.minimum_participant_clearance
              << std::endl;
}

TEST(NlpSolverTest, SolvesThreeVehiclesWithNonContiguousIdsOnCommonGrid) {
    const fnr::NlpOptimizationInput input = inputFor({1, 5, 12});
    const fnr::NlpInitialGuess guess = initialGuess(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    const fnr::NlpSolveResult result = solver().solve(input, guess);
    EXPECT_TRUE(result.solver_success) << result.reason;
    EXPECT_TRUE(result.validation_passed) << result.validation.reason;
    ASSERT_EQ(result.vehicles.size(), 3U);
    for (const auto& vehicle_result : result.vehicles) {
        ASSERT_EQ(vehicle_result.nodes.size(), result.node_count);
        for (size_t node = 0; node < result.node_count; ++node) {
            EXPECT_NEAR(vehicle_result.nodes[node].time,
                        result.average_dt * node, 1e-12);
        }
    }
    std::cout << "three_vehicle_result T=" << result.total_time
              << " iterations=" << result.statistics.iterations
              << " wall=" << result.statistics.wall_time
              << " max_violation=" << result.statistics.max_constraint_violation
              << " min_clear=" << result.validation.minimum_participant_clearance
              << std::endl;
}

TEST(NlpSolverTest, RejectsDisconnectedFixedAabbAssignmentBeforeSolve) {
    fnr::NlpOptimizationInput input = inputFor({7});
    input.vehicles.front().corridor = {{-0.2, 0.4, -0.2, 0.2},
                                       {0.6, 1.2, -0.2, 0.2}};
    fnr::NlpInitialGuess guess = initialGuess(input);
    ASSERT_FALSE(guess.generated);
    const fnr::NlpSolveResult result = solver().solve(input, guess);
    EXPECT_FALSE(result.problem_built);
    EXPECT_FALSE(result.solver_success);
}

TEST(NlpSolverTest, OutsideStartSkipsOnlyStartAndEntryIntervalAabbChecks) {
    fnr::NlpOptimizationInput input = inputFor({2});
    auto& source = input.vehicles.front();
    source.initial_state.rear_axle_pose.x = -0.25;
    source.initial_state.longitudinal_speed = 0.0;
    source.corridor = {{-0.10, 1.20, -0.20, 0.20}};
    source.corridor_seeds = {{-0.1765, 0.0, 0.0},
                             {-0.05, 0.0, 0.0},
                             {0.50, 0.0, 0.0}};
    const fnr::NlpInitialGuess guess = initialGuess(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    ASSERT_TRUE(guess.vehicles.front().start_aabb_exempt);
    const fnr::NlpSolveResult result = solver().solve(input, guess);
    EXPECT_TRUE(result.problem_built) << result.reason;
    EXPECT_TRUE(result.solver_success) << result.reason;
    EXPECT_TRUE(result.validation_passed) << result.validation.reason;
    ASSERT_EQ(result.vehicles.size(), 1U);
    EXPECT_TRUE(result.vehicles.front().start_aabb_exempt);
    EXPECT_EQ(result.validation.aabb_violation_samples, 0U);
    EXPECT_EQ(result.validation.invalid_aabb_switches, 0U);
    EXPECT_LT(result.vehicles.front().nodes.front().disc_center.x,
              source.corridor.front().min_x);
    EXPECT_GE(result.vehicles.front().nodes[1].disc_center.x,
              source.corridor.front().min_x - 2e-4);
}

TEST(NlpSolverTest, ReportsInitialParticipantCollisionAsNotSafe) {
    fnr::NlpOptimizationInput input = inputFor({3, 8});
    input.vehicles[1] = vehicle(8, 0.0);
    const fnr::NlpInitialGuess guess = initialGuess(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    const fnr::NlpSolveResult result = solver().solve(input, guess);
    EXPECT_TRUE(result.problem_built);
    EXPECT_FALSE(result.validation_passed);
    EXPECT_FALSE(result.statistics.return_status.empty());
}

TEST(NlpSolverTest, ReportsStaticObstacleAtFixedTerminalAsInfeasible) {
    fnr::NlpOptimizationInput input = inputFor({6});
    fnr::StaticVehicleObstacle obstacle;
    obstacle.id = 42;
    obstacle.disc_radius = input.vehicles.front().parameters.disc_radius;
    obstacle.rear_axle_pose = input.vehicles.front().terminal.pose;
    obstacle.disc_center = {
        obstacle.rear_axle_pose.x +
            input.vehicles.front().parameters.rear_axle_to_center,
        obstacle.rear_axle_pose.y, obstacle.rear_axle_pose.yaw};
    input.static_vehicles.push_back(obstacle);
    const fnr::NlpInitialGuess guess = initialGuess(input);
    ASSERT_TRUE(guess.generated) << guess.reason;
    const fnr::NlpSolveResult result = solver().solve(input, guess);
    EXPECT_TRUE(result.problem_built);
    EXPECT_FALSE(result.validation_passed);
    EXPECT_FALSE(result.solver_success);
    EXPECT_FALSE(result.statistics.return_status.empty());
    EXPECT_GE(result.statistics.iterations, 0);
}

TEST(NlpSolverTest, SavesTrajectoryAndReportInExistingCaseDirectory) {
    const fnr::NlpOptimizationInput input = inputFor({4});
    const fnr::NlpInitialGuess guess = initialGuess(input);
    const fnr::NlpSolveResult result = solver().solve(input, guess);
    ASSERT_TRUE(result.solver_success) << result.reason;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        ("forklift_nlp_solver_test_" + std::to_string(stamp));
    ASSERT_TRUE(std::filesystem::create_directory(directory));
    std::string error;
    EXPECT_TRUE(fnr::saveNlpSolveResult(result, directory.string(), error)) << error;
    EXPECT_TRUE(std::filesystem::exists(directory / "optimized_trajectory.csv"));
    EXPECT_TRUE(std::filesystem::exists(directory / "solver_report.yaml"));
    std::ifstream report(directory / "solver_report.yaml");
    const std::string content((std::istreambuf_iterator<char>(report)),
                              std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("validation_passed: true"), std::string::npos);
    std::filesystem::remove_all(directory);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
