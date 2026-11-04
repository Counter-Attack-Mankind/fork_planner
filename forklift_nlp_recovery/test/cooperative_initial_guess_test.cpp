#include <gtest/gtest.h>

#include "forklift_nlp_recovery/cooperative/cooperative_initial_guess.h"

namespace fnr = forklift_nlp_recovery;

namespace {

fnr::NlpVehicleInput vehicle(int id, double y) {
    fnr::NlpVehicleInput value;
    value.id = id;
    value.stc_valid = true;
    value.terminal_valid = true;
    value.safe_stc = {0.0, 2.0, y - 0.4, y + 0.4};
    value.frozen_initial_state.rear_axle_pose = {0.2, y, 0.0};
    value.frozen_initial_state.steering_valid = true;
    value.frozen_initial_state.steering_angle = 0.1;
    value.frozen_path_s = 0.2;
    value.terminal = {{1.0, y, 0.0}, 1.0, id + 10, 0};
    value.parameters.nominal_speed = 0.2;
    value.parameters.max_speed = 0.3;
    value.parameters.rear_axle_to_center = 0.1;
    return value;
}

fnr::NlpOptimizationInput cooperativeInput() {
    fnr::NlpOptimizationInput input;
    input.valid = true;
    input.participant_ids = {2, 5, 9};
    input.vehicles = {vehicle(2, 0.5), vehicle(5, 1.5), vehicle(9, 2.5)};
    return input;
}

}  // namespace

TEST(CooperativeInitialGuessTest, ProducesOnlyStationaryMotionValues) {
    fnr::InitialGuessConfig config;
    config.node_count = 7;
    const fnr::NlpInitialGuess guess =
        fnr::CooperativeInitialGuessGenerator(config).generate(cooperativeInput());
    ASSERT_TRUE(guess.generated) << guess.reason;
    ASSERT_EQ(guess.vehicles.size(), 3U);
    ASSERT_EQ(guess.node_count, 7U);
    EXPECT_GT(guess.total_time, 0.0);
    for (const fnr::VehicleInitialGuess& vehicle_guess : guess.vehicles) {
        ASSERT_EQ(vehicle_guess.nodes.size(), 7U);
        const fnr::Pose2d start = vehicle_guess.nodes.front().rear_axle_pose;
        for (const fnr::InitialGuessNode& node : vehicle_guess.nodes) {
            EXPECT_DOUBLE_EQ(node.rear_axle_pose.x, start.x);
            EXPECT_DOUBLE_EQ(node.rear_axle_pose.y, start.y);
            EXPECT_DOUBLE_EQ(node.rear_axle_pose.yaw, start.yaw);
            EXPECT_DOUBLE_EQ(node.signed_speed, 0.0);
            EXPECT_DOUBLE_EQ(node.acceleration, 0.0);
            EXPECT_DOUBLE_EQ(node.steer_rate, 0.0);
            EXPECT_DOUBLE_EQ(node.front_steer, 0.1);
            EXPECT_EQ(node.aabb_index, -1);
        }
    }
}

TEST(CooperativeInitialGuessTest, RejectsAStartCorrectedAwayFromFrozenState) {
    fnr::NlpOptimizationInput input = cooperativeInput();
    input.vehicles.front().initial_state_corrected = true;
    const fnr::NlpInitialGuess guess =
        fnr::CooperativeInitialGuessGenerator({}).generate(input);
    EXPECT_FALSE(guess.generated);
    EXPECT_NE(guess.reason.find("corrected start"), std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
