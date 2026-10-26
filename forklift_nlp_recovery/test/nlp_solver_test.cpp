#include <gtest/gtest.h>

#include "forklift_nlp_recovery/nlp_solver.h"

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

fnr::NlpVehicleInput vehicle(int id, double y) {
    fnr::NlpVehicleInput value;
    value.id = id;
    value.parameters = parameters();
    value.path_gen = id + 20;
    value.initial_state.rear_axle_pose = {1.0, y, 0.0};
    value.original_path = {{{1.0, y, 0.0}, 0.0, 0},
                           {{1.5, y, 0.0}, 0.5, 0},
                           {{2.0, y, 0.0}, 1.0, 0}};
    value.terminal = {value.original_path.back().pose, 1.0, value.path_gen, 0};
    value.corridor_valid = true;
    value.corridor_reason = "terminal selected";
    value.corridor = {{0.20, 2.30, 0.30, 4.20}};
    value.corridor_seeds = {{1.05, y, 0.0}};
    return value;
}

fnr::NlpOptimizationInput inputFor(const std::vector<int>& ids) {
    fnr::NlpOptimizationInput input;
    input.valid = true;
    input.reason = "ok";
    for (size_t i = 0; i < ids.size(); ++i) {
        input.participant_ids.push_back(ids[i]);
        input.vehicles.push_back(vehicle(ids[i], 1.0 + 0.5 * i));
    }
    return input;
}

struct FixtureData {
    MapParam map;
    fnr::CorridorConfig corridor_config;
    fnr::CorridorBuilder corridor;
    fnr::NlpSolverConfig solver_config;
    fnr::NlpSolver solver;

    FixtureData()
        : corridor(map, {}, corridor_config), solver_config(),
          solver(solver_config, corridor) {}
};

}  // namespace

TEST(NlpSolverTest, RejectsMissingOneToOneAabbsBeforeCallingIpopt) {
    FixtureData fixture;
    fnr::NlpOptimizationInput input = inputFor({2});
    const auto prepared = fnr::InitialGuessGenerator({}, fixture.corridor).prepare(input);
    ASSERT_TRUE(prepared.actual_guess.generated) << prepared.actual_guess.reason;
    input.vehicles.front().corridor.pop_back();
    const fnr::NlpSolveResult result = fixture.solver.solve(input, prepared.actual_guess);
    EXPECT_FALSE(result.problem_built);
    EXPECT_FALSE(result.solver_success);
    EXPECT_NE(result.reason.find("one-to-one node AABBs"), std::string::npos);
}

TEST(NlpSolverTest, BuildsSingleVehicleProblemWith51NodeGrid) {
    FixtureData fixture;
    fixture.solver_config.max_iterations = 400;
    fixture.solver_config.print_level = 0;
    fnr::NlpOptimizationInput input = inputFor({2});
    const auto prepared = fnr::InitialGuessGenerator({}, fixture.corridor).prepare(input);
    ASSERT_TRUE(prepared.actual_guess.generated) << prepared.actual_guess.reason;
    ASSERT_EQ(prepared.actual_guess.node_count, 51U);
    const fnr::NlpSolveResult result = fixture.solver.solve(input, prepared.actual_guess);
    EXPECT_TRUE(result.problem_built) << result.reason;
    EXPECT_EQ(result.node_count, 51U);
    EXPECT_GE(result.constraint_diagnostics.categories.size(), 3U);
    ASSERT_EQ(result.vehicles.size(), 1U);
    EXPECT_EQ(result.vehicles[0].nodes.size(), 51U);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
