#include "forklift_nlp_recovery/case_manager.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <unistd.h>

namespace forklift_nlp_recovery {
namespace {

sandbox_msgs::RecoverySnapshot snapshot() {
    sandbox_msgs::RecoverySnapshot value;
    value.header.frame_id = "map";
    value.recovery_id = 42;
    value.reason = "test";
    value.frozen_vehicle_ids = {3};
    sandbox_msgs::RecoveryVehicleState vehicle;
    vehicle.id = 3;
    vehicle.path_gen = 7;
    vehicle.path_s = 0.25;
    vehicle.pose_valid = true;
    vehicle.pose_fresh = true;
    vehicle.pose.x = 1.0;
    vehicle.pose.y = 2.0;
    vehicle.pose.theta = 0.3;
    geometry_msgs::Pose2D next = vehicle.pose;
    next.x += 0.1;
    vehicle.path = {vehicle.pose, next};
    vehicle.path_types = {0, 0};
    value.vehicles.push_back(vehicle);
    return value;
}

TEST(CaseManagerTest, NumberingPersistsAndFilesAreNotOverwritten) {
    const std::filesystem::path root = std::filesystem::temp_directory_path() /
        ("nlp_case_manager_" + std::to_string(::getpid()));
    std::filesystem::remove_all(root);
    const std::string cases = (root / "case").string();
    const std::string results = (root / "results").string();
    CaseManager first(cases, results);
    const CaseDirectory case1 = first.createCase(snapshot(), {3});
    const CaseDirectory case2 = first.createCase(snapshot(), {3});
    ASSERT_TRUE(case1.success) << case1.reason;
    ASSERT_TRUE(case2.success) << case2.reason;
    EXPECT_EQ(case1.number, 1u);
    EXPECT_EQ(case2.number, 2u);

    CaseManager restarted(cases, results);
    const CaseDirectory case3 = restarted.createCase(snapshot(), {3});
    ASSERT_TRUE(case3.success) << case3.reason;
    EXPECT_EQ(case3.number, 3u);
    const ResultDirectory run1 = restarted.createResult(1);
    const ResultDirectory run2 = restarted.createResult(1);
    ASSERT_TRUE(run1.success);
    ASSERT_TRUE(run2.success);
    EXPECT_EQ(run1.run_number, 1u);
    EXPECT_EQ(run2.run_number, 2u);

    sandbox_msgs::RecoverySnapshot loaded;
    std::vector<int> participants;
    std::string error;
    ASSERT_TRUE(loadRecoveryCase(case1.path, loaded, participants, error)) << error;
    EXPECT_EQ(loaded.recovery_id, 42u);
    ASSERT_EQ(loaded.vehicles.size(), 1u);
    EXPECT_DOUBLE_EQ(loaded.vehicles.front().current_speed, 0.0);
    EXPECT_EQ(participants, std::vector<int>({3}));
    std::filesystem::remove_all(root);
}

}  // namespace
}  // namespace forklift_nlp_recovery

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
