#include "forklift_nlp_recovery/replay_data.h"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

namespace forklift_nlp_recovery {
namespace {

ReplayVehicle vehicle(int id, double x, double y, double speed = 0.0) {
    ReplayVehicle value;
    value.id = id;
    value.parameters = {0.20, 0.10, 0.05, 0.50};
    value.corridor = {{-10.0, 10.0, -10.0, 10.0}};
    value.nodes = {{0, 0.0, {x, y, 0.0}, speed, 0.0, 0.0, 0.0, 0},
                   {1, 1.0, {x + speed, y, 0.0}, speed, 0.0, 0.0, 0.0, 0}};
    return value;
}

std::filesystem::path writeCase(bool inconsistent_time = false) {
    const auto directory = std::filesystem::temp_directory_path() /
        ("forklift_replay_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
         (inconsistent_time ? "_bad" : "_valid"));
    std::filesystem::create_directories(directory);
    std::ofstream input(directory / "optimization_input.yaml");
    input << "frame_id: map\nparticipant_ids: [2, 9]\nvehicles:\n"
          << "  - id: 2\n    parameters: {length: 0.2, width: 0.1, rear_axle_to_center: 0.05, disc_radius: 0.12, wheel_base: 0.5}\n"
          << "    aabbs: [{min_x: -1, max_x: 3, min_y: -1, max_y: 1}]\n"
          << "  - id: 9\n    parameters: {length: 0.2, width: 0.1, rear_axle_to_center: 0.05, disc_radius: 0.12, wheel_base: 0.5}\n"
          << "    aabbs: [{min_x: -1, max_x: 3, min_y: 1, max_y: 3}]\n"
          << "static_vehicles: []\n";
    input.close();
    std::ofstream csv(directory / "optimized_trajectory.csv");
    csv << "vehicle_id,node_index,time,rear_x,rear_y,heading,signed_speed,front_steer,acceleration,steer_rate,aabb_index\n"
        << "2,0,0,0,0,0,1,0,0,0,0\n2,1,1,1,0,0,1,0,0,0,0\n"
        << "9,0,0,0,2,0,1,0,0,0,0\n9,1," << (inconsistent_time ? 1.1 : 1.0) << ",1,2,0,1,0,0,0,0\n";
    csv.close();
    std::ofstream report(directory / "solver_report.yaml");
    report << "solver_success: true\nvalidation_passed: true\nreturn_status: Solve_Succeeded\nnode_count: 2\ntotal_time: 1.0\n"
           << "validation:\n  reason: ok\n  minimum_participant_clearance: 1.76\n  minimum_static_clearance: 0\n"
           << "  participant_collision_samples: 0\n  static_collision_samples: 0\n  aabb_violation_samples: 0\n";
    return directory;
}

TEST(ReplayDataTest, LoadsNonContiguousVehicleIdsAndCommonGrid) {
    const auto directory = writeCase();
    ReplayCase replay_case;
    std::string error;
    ASSERT_TRUE(loadReplayCase(directory.string(), replay_case, error)) << error;
    ASSERT_EQ(replay_case.participant_ids, (std::vector<int>{2, 9}));
    EXPECT_EQ(replay_case.node_count, 2u);
    EXPECT_DOUBLE_EQ(replay_case.total_time, 1.0);
    EXPECT_EQ(replay_case.report.collision_geometry,
              "legacy_body_center_discs");
    EXPECT_FALSE(replay_case.report.has_obb_validation_metrics);
    std::filesystem::remove_all(directory);
}

TEST(ReplayDataTest, RejectsDifferentParticipantTimeAxes) {
    const auto directory = writeCase(true);
    ReplayCase replay_case;
    std::string error;
    EXPECT_FALSE(loadReplayCase(directory.string(), replay_case, error));
    EXPECT_NE(error.find("time axes differ"), std::string::npos);
    std::filesystem::remove_all(directory);
}

TEST(ReplayDataTest, RequiresAllThreeSavedFiles) {
    const auto directory = std::filesystem::temp_directory_path() / "forklift_replay_missing";
    std::filesystem::create_directories(directory);
    ReplayCase replay_case;
    std::string error;
    EXPECT_FALSE(loadReplayCase(directory.string(), replay_case, error));
    EXPECT_NE(error.find("required file is missing"), std::string::npos);
    std::filesystem::remove_all(directory);
}

TEST(ReplayDataTest, UsesSolverRk4ForIntermediatePose) {
    ReplayVehicle value = vehicle(5, 0.0, 0.0, 1.0);
    value.nodes.front().steer = 0.2;
    const ReplayNode half = sampleReplayVehicle(value, 0.5);
    const ReplayNode direct = integrateReplayNode(value.nodes.front(), 0.5, value.parameters.wheel_base);
    EXPECT_NEAR(half.rear_axle.x, direct.rear_axle.x, 1e-12);
    EXPECT_NEAR(half.rear_axle.y, direct.rear_axle.y, 1e-12);
    EXPECT_NEAR(half.rear_axle.yaw, direct.rear_axle.yaw, 1e-12);
    EXPECT_GT(std::abs(half.rear_axle.y), 1e-4);
}

TEST(ReplayDataTest, IgnoresFormerDiscOverlapWhenObbsAreSeparated) {
    ReplayCase replay_case;
    replay_case.vehicles = {vehicle(2, 0.0, 0.0), vehicle(9, 0.0, 0.11)};
    replay_case.participant_ids = {2, 9};
    replay_case.node_count = 2;
    replay_case.total_time = 1.0;
    EXPECT_TRUE(checkReplayInstant(replay_case, 0.5, 1e-6).empty());
}

TEST(ReplayDataTest, ReportsOnlyObbSatAndSeparateAabbViolations) {
    ReplayCase replay_case;
    replay_case.vehicles = {vehicle(2, 0.0, 0.0), vehicle(9, 0.0, 0.05)};
    replay_case.participant_ids = {2, 9};
    replay_case.node_count = 2;
    replay_case.total_time = 1.0;
    replay_case.vehicles[0].corridor = {{0.20, 1.0, -1.0, 1.0}};
    replay_case.static_vehicles.push_back({17, {0.0, -0.05, 0.0},
                                           0.20, 0.10, 0.05});
    const auto violations = checkReplayInstant(replay_case, 0.5, 1e-6);
    bool participant_obb = false, static_obb = false, aabb = false;
    for (const auto& violation : violations) {
        aabb |= violation.type == ReplayViolationType::AABB;
        participant_obb |= violation.type == ReplayViolationType::OBB &&
                           !violation.other_is_static;
        static_obb |= violation.type == ReplayViolationType::OBB &&
                      violation.other_is_static;
    }
    EXPECT_TRUE(participant_obb);
    EXPECT_TRUE(static_obb);
    EXPECT_TRUE(aabb);
}

TEST(ReplayDataTest, ScansIntervalBoundariesAndWrapsClock) {
    ReplayCase replay_case;
    replay_case.vehicles = {vehicle(2, 0.0, 0.0), vehicle(9, 2.0, 0.0)};
    replay_case.participant_ids = {2, 9}; replay_case.node_count = 2; replay_case.total_time = 1.0;
    const auto diagnostics = scanReplayCase(replay_case, 5, 1e-6);
    EXPECT_TRUE(diagnostics.violations.empty());
    EXPECT_NEAR(wrapReplayTime(0.95, 0.1, 1.0, 1.0), 0.05, 1e-12);
    EXPECT_NEAR(sampleReplayVehicle(replay_case.vehicles.front(), 1.0).time, 1.0, 1e-12);
}

}  // namespace
}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
