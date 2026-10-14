#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace forklift_nlp_recovery {

struct ReplayPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct ReplayAabb {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

struct ReplayParameters {
    double length = 0.0;
    double width = 0.0;
    double rear_axle_to_center = 0.0;
    double disc_radius = 0.0;
    double wheel_base = 0.0;
};

struct ReplayNode {
    int node_index = -1;
    double time = 0.0;
    ReplayPose rear_axle;
    double speed = 0.0;
    double steer = 0.0;
    double acceleration = 0.0;
    double steer_rate = 0.0;
    int aabb_index = -1;
};

struct ReplayVehicle {
    int id = -1;
    ReplayParameters parameters;
    std::vector<ReplayAabb> corridor;
    std::vector<ReplayNode> nodes;
};

struct ReplayStaticVehicle {
    int id = -1;
    ReplayPose rear_axle;
    ReplayPose disc_center;
    double disc_radius = 0.0;
};

struct ReplaySolverReport {
    bool solver_success = false;
    bool validation_passed = false;
    std::string return_status;
    std::string validation_reason;
    std::size_t node_count = 0;
    double total_time = 0.0;
    double minimum_participant_obb_separation_margin = 0.0;
    double minimum_static_obb_separation_margin = 0.0;
    std::size_t participant_obb_collision_samples = 0;
    std::size_t static_obb_collision_samples = 0;
    std::size_t aabb_violation_samples = 0;
};

struct ReplayCase {
    std::string case_id;
    std::string frame_id = "map";
    std::vector<int> participant_ids;
    std::vector<ReplayVehicle> vehicles;
    std::vector<ReplayStaticVehicle> static_vehicles;
    ReplaySolverReport report;
    std::size_t node_count = 0;
    double total_time = 0.0;
};

enum class ReplayViolationType { PARTICIPANT_DISC, STATIC_DISC, AABB, OBB };

struct ReplayViolation {
    ReplayViolationType type = ReplayViolationType::PARTICIPANT_DISC;
    double time = 0.0;
    int vehicle_id = -1;
    int other_id = -1;
    double actual = 0.0;
    double required = 0.0;
    double magnitude = 0.0;
    ReplayPose first_center;
    ReplayPose second_center;
};

struct ReplayViolationInterval {
    ReplayViolationType type = ReplayViolationType::PARTICIPANT_DISC;
    int vehicle_id = -1;
    int other_id = -1;
    double start_time = 0.0;
    double end_time = 0.0;
    double max_violation = 0.0;
};

struct ReplayDiagnostics {
    double minimum_participant_clearance = 0.0;
    double minimum_static_clearance = 0.0;
    std::vector<ReplayViolation> violations;
    std::vector<ReplayViolationInterval> intervals;
};

bool loadReplayCase(const std::string& case_dir, ReplayCase& replay_case, std::string& error);
ReplayNode integrateReplayNode(const ReplayNode& source, double dt, double wheel_base);
ReplayNode sampleReplayVehicle(const ReplayVehicle& vehicle, double time, std::size_t* interval_index = nullptr);
ReplayPose replayBodyCenter(const ReplayPose& rear_axle, double rear_axle_to_center);
double replayAabbViolation(const ReplayPose& center, const ReplayAabb& box);
std::vector<ReplayViolation> checkReplayInstant(const ReplayCase& replay_case, double time,
                                                double tolerance);
ReplayDiagnostics scanReplayCase(const ReplayCase& replay_case, std::size_t substeps,
                                 double tolerance);
double wrapReplayTime(double current, double wall_dt, double playback_rate, double total_time);
const char* replayViolationTypeName(ReplayViolationType type);

}  // namespace forklift_nlp_recovery
