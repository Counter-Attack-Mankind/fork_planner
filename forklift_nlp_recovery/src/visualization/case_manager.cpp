#include "forklift_nlp_recovery/case_manager.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <type_traits>
#include <utility>

namespace forklift_nlp_recovery {
namespace {

constexpr unsigned int kSchemaVersion = 1;

template <typename Directory>
Directory createNumberedDirectory(const std::filesystem::path& root,
                                  unsigned int case_number = 0) {
    (void)case_number;
    Directory result;
    std::error_code error;
    std::filesystem::create_directories(root, error);
    if (error) {
        result.reason = "cannot create directory root: " + error.message();
        return result;
    }
    for (unsigned int number = 1; number < 1000000; ++number) {
        const std::filesystem::path candidate = root / std::to_string(number);
        error.clear();
        if (std::filesystem::create_directory(candidate, error)) {
            result.success = true;
            result.path = candidate.string();
            if constexpr (std::is_same<Directory, CaseDirectory>::value) {
                result.number = number;
            } else {
                result.case_number = case_number;
                result.run_number = number;
            }
            result.reason = "ok";
            return result;
        }
        if (error) {
            result.reason = "cannot atomically create numbered directory: " + error.message();
            return result;
        }
    }
    result.reason = "no available numeric directory";
    return result;
}

void emitPose(YAML::Emitter& out, const geometry_msgs::Pose2D& pose) {
    out << YAML::Flow << YAML::BeginMap << YAML::Key << "x" << YAML::Value << pose.x
        << YAML::Key << "y" << YAML::Value << pose.y << YAML::Key << "yaw"
        << YAML::Value << pose.theta << YAML::EndMap;
}

bool writeScene(const std::filesystem::path& directory,
                const sandbox_msgs::RecoverySnapshot& snapshot,
                const std::vector<int>& participant_ids, std::string& error) {
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value
        << kSchemaVersion << YAML::Key << "recovery_id" << YAML::Value
        << snapshot.recovery_id << YAML::Key << "frame_id" << YAML::Value
        << snapshot.header.frame_id << YAML::Key << "stamp" << YAML::Value
        << YAML::Flow << YAML::BeginMap << YAML::Key << "sec" << YAML::Value
        << snapshot.header.stamp.sec << YAML::Key << "nsec" << YAML::Value
        << snapshot.header.stamp.nsec << YAML::EndMap << YAML::Key << "reason"
        << YAML::Value << snapshot.reason << YAML::Key << "participant_ids"
        << YAML::Value << YAML::Flow << participant_ids << YAML::Key
        << "frozen_vehicle_ids" << YAML::Value << YAML::Flow
        << snapshot.frozen_vehicle_ids << YAML::Key << "vehicles" << YAML::Value
        << YAML::BeginSeq;
    for (const auto& vehicle : snapshot.vehicles) {
        out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << vehicle.id
            << YAML::Key << "path_gen" << YAML::Value << vehicle.path_gen
            << YAML::Key << "path_s" << YAML::Value << vehicle.path_s
            << YAML::Key << "nlp_initial_speed" << YAML::Value << 0.0
            << YAML::Key << "frozen_speed" << YAML::Value << vehicle.current_speed
            << YAML::Key << "pose_valid" << YAML::Value << (vehicle.pose_valid != 0)
            << YAML::Key << "pose_fresh" << YAML::Value << (vehicle.pose_fresh != 0)
            << YAML::Key << "pose" << YAML::Value;
        emitPose(out, vehicle.pose);
        out << YAML::Key << "steering_valid" << YAML::Value << (vehicle.steering_valid != 0)
            << YAML::Key << "steering_estimated" << YAML::Value << (vehicle.steering_estimated != 0)
            << YAML::Key << "steering_angle" << YAML::Value << vehicle.steering_angle
            << YAML::Key << "parked_in_slot" << YAML::Value << (vehicle.parked_in_slot != 0)
            << YAML::Key << "mode" << YAML::Value << vehicle.mode
            << YAML::Key << "mission_phase" << YAML::Value << vehicle.mission_phase
            << YAML::Key << "action" << YAML::Value << vehicle.action
            << YAML::Key << "blocker_id" << YAML::Value << vehicle.blocker_id
            << YAML::Key << "blocker_reason" << YAML::Value << vehicle.blocker_reason
            << YAML::Key << "loaded" << YAML::Value << (vehicle.loaded != 0)
            << YAML::Key << "slot_departure_clear_s" << YAML::Value
            << vehicle.slot_departure_clear_s << YAML::Key << "departed_source_slot"
            << YAML::Value << (vehicle.departed_source_slot != 0) << YAML::Key << "path"
            << YAML::Value << YAML::BeginSeq;
        for (const auto& pose : vehicle.path) emitPose(out, pose);
        out << YAML::EndSeq << YAML::Key << "path_types" << YAML::Value
            << YAML::Flow << YAML::BeginSeq;
        for (uint8_t type : vehicle.path_types) out << static_cast<unsigned int>(type);
        out << YAML::EndSeq << YAML::EndMap;
    }
    out << YAML::EndSeq << YAML::EndMap;
    const std::filesystem::path path = directory / "scene.yaml";
    std::ofstream file(path, std::ios::trunc);
    if (!file) { error = "cannot open scene.yaml"; return false; }
    file << out.c_str() << '\n';
    if (!file) { error = "failed writing scene.yaml"; return false; }
    error = "ok";
    return true;
}

geometry_msgs::Pose2D readPose(const YAML::Node& node) {
    if (!node || !node["x"] || !node["y"] || !node["yaw"]) {
        throw std::runtime_error("pose lacks x, y, or yaw");
    }
    geometry_msgs::Pose2D pose;
    pose.x = node["x"].as<double>();
    pose.y = node["y"].as<double>();
    pose.theta = node["yaw"].as<double>();
    return pose;
}

}  // namespace

CaseManager::CaseManager(std::string case_root, std::string results_root)
    : case_root_(std::move(case_root)), results_root_(std::move(results_root)) {}

CaseDirectory CaseManager::createCase(
    const sandbox_msgs::RecoverySnapshot& snapshot,
    const std::vector<int>& participant_ids) const {
    if (snapshot.recovery_id == 0 || participant_ids.empty()) {
        return {false, 0, "", "recovery_id and participant_ids are required"};
    }
    std::set<int> available;
    for (const auto& vehicle : snapshot.vehicles) available.insert(vehicle.id);
    for (int id : participant_ids) {
        if (available.count(id) == 0) {
            return {false, 0, "", "participant V" + std::to_string(id) + " is missing"};
        }
    }
    CaseDirectory result = createNumberedDirectory<CaseDirectory>(case_root_);
    if (!result.success) return result;
    std::string error;
    if (!writeScene(result.path, snapshot, participant_ids, error)) {
        std::ofstream failure(std::filesystem::path(result.path) / "write_error.txt");
        failure << error << '\n';
        result.success = false;
        result.reason = error;
        return result;
    }
    return result;
}

ResultDirectory CaseManager::createResult(unsigned int case_number) const {
    if (case_number == 0) return {false, 0, 0, "", "case number must be positive"};
    return createNumberedDirectory<ResultDirectory>(
        std::filesystem::path(results_root_) / std::to_string(case_number), case_number);
}

bool CaseManager::writeRunStatus(const ResultDirectory& directory,
                                 const std::string& stage, bool complete,
                                 bool success, const std::string& reason,
                                 std::string& error) const {
    if (!directory.success || directory.path.empty()) {
        error = "valid result directory is required";
        return false;
    }
    std::ofstream file(std::filesystem::path(directory.path) / "run_status.yaml",
                       std::ios::trunc);
    if (!file) { error = "cannot open run_status.yaml"; return false; }
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value
        << kSchemaVersion << YAML::Key << "case" << YAML::Value
        << directory.case_number << YAML::Key << "run" << YAML::Value
        << directory.run_number << YAML::Key << "stage" << YAML::Value << stage
        << YAML::Key << "complete" << YAML::Value << complete << YAML::Key
        << "success" << YAML::Value << success << YAML::Key << "reason"
        << YAML::Value << reason << YAML::EndMap;
    file << out.c_str() << '\n';
    if (!file) { error = "failed writing run_status.yaml"; return false; }
    error = "ok";
    return true;
}

bool CaseManager::writeVirtualBridgeEscape(
    const ResultDirectory& directory, int vehicle_id, int path_gen,
    const geometry_msgs::Pose2D& frozen_pose,
    const geometry_msgs::Pose2D& target_pose, double distance, int direction,
    std::string& error) const {
    if (!directory.success || directory.path.empty()) {
        error = "valid result directory is required";
        return false;
    }
    const std::filesystem::path path =
        std::filesystem::path(directory.path) / "virtual_bridge_escape.yaml";
    if (std::filesystem::exists(path)) {
        error = "virtual_bridge_escape.yaml already exists";
        return false;
    }
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value
        << kSchemaVersion << YAML::Key << "mode" << YAML::Value
        << "offline_virtual_bridge_escape" << YAML::Key << "vehicle_id"
        << YAML::Value << vehicle_id << YAML::Key << "path_gen" << YAML::Value
        << path_gen << YAML::Key << "distance" << YAML::Value << distance
        << YAML::Key << "direction" << YAML::Value << direction
        << YAML::Key << "frozen_pose" << YAML::Value;
    emitPose(out, frozen_pose);
    out << YAML::Key << "target_pose" << YAML::Value;
    emitPose(out, target_pose);
    out << YAML::EndMap;
    std::ofstream file(path);
    if (!file) { error = "cannot open virtual_bridge_escape.yaml"; return false; }
    file << out.c_str() << '\n';
    if (!file) { error = "failed writing virtual_bridge_escape.yaml"; return false; }
    error = "ok";
    return true;
}

bool loadRecoveryCase(const std::string& case_directory,
                      sandbox_msgs::RecoverySnapshot& snapshot,
                      std::vector<int>& participant_ids, std::string& error) {
    try {
        const YAML::Node root = YAML::LoadFile(
            (std::filesystem::path(case_directory) / "scene.yaml").string());
        if (!root["schema_version"] || root["schema_version"].as<unsigned int>() != kSchemaVersion ||
            !root["recovery_id"] || !root["frame_id"] || !root["participant_ids"] ||
            !root["vehicles"] || !root["vehicles"].IsSequence()) {
            throw std::runtime_error("scene.yaml lacks required schema fields");
        }
        snapshot = sandbox_msgs::RecoverySnapshot{};
        snapshot.header.frame_id = root["frame_id"].as<std::string>();
        if (root["stamp"]) {
            snapshot.header.stamp.sec = root["stamp"]["sec"].as<uint32_t>();
            snapshot.header.stamp.nsec = root["stamp"]["nsec"].as<uint32_t>();
        }
        snapshot.recovery_id = root["recovery_id"].as<uint64_t>();
        snapshot.recovery_phase = sandbox_msgs::RecoverySnapshot::PHASE_NLP_PREP;
        snapshot.reason = root["reason"] ? root["reason"].as<std::string>() : "offline_case";
        participant_ids = root["participant_ids"].as<std::vector<int>>();
        snapshot.deadlock_cycle_ids = participant_ids;
        snapshot.nlp_vehicle_ids = participant_ids;
        if (root["frozen_vehicle_ids"]) {
            snapshot.frozen_vehicle_ids = root["frozen_vehicle_ids"].as<std::vector<int>>();
        }
        for (const YAML::Node& source : root["vehicles"]) {
            if (!source["id"] || !source["path_gen"] || !source["path_s"] ||
                !source["pose"] || !source["path"] || !source["path_types"]) {
                throw std::runtime_error("vehicle entry lacks required identity, pose, or path fields");
            }
            sandbox_msgs::RecoveryVehicleState vehicle;
            vehicle.id = source["id"].as<int>();
            vehicle.path_gen = source["path_gen"].as<int>();
            vehicle.path_s = source["path_s"].as<double>();
            vehicle.current_speed = 0.0;
            vehicle.pose_valid = source["pose_valid"] ? source["pose_valid"].as<bool>() : true;
            vehicle.pose_fresh = source["pose_fresh"] ? source["pose_fresh"].as<bool>() : true;
            vehicle.pose = readPose(source["pose"]);
            vehicle.steering_valid = source["steering_valid"] && source["steering_valid"].as<bool>();
            vehicle.steering_estimated = source["steering_estimated"] && source["steering_estimated"].as<bool>();
            vehicle.steering_angle = source["steering_angle"] ? source["steering_angle"].as<double>() : 0.0;
            vehicle.parked_in_slot = source["parked_in_slot"] && source["parked_in_slot"].as<bool>();
            vehicle.mode = source["mode"] ? source["mode"].as<int>() : 0;
            vehicle.mission_phase = source["mission_phase"] ? source["mission_phase"].as<int>() : 0;
            vehicle.action = source["action"] ? source["action"].as<int>() : 0;
            vehicle.blocker_id = source["blocker_id"] ? source["blocker_id"].as<int>() : -1;
            vehicle.blocker_reason = source["blocker_reason"] ? source["blocker_reason"].as<std::string>() : "";
            vehicle.loaded = source["loaded"] && source["loaded"].as<bool>();
            vehicle.slot_departure_clear_s = source["slot_departure_clear_s"] ? source["slot_departure_clear_s"].as<double>() : 0.0;
            vehicle.departed_source_slot = source["departed_source_slot"] && source["departed_source_slot"].as<bool>();
            for (const YAML::Node& pose : source["path"]) vehicle.path.push_back(readPose(pose));
            for (const YAML::Node& type : source["path_types"]) {
                vehicle.path_types.push_back(static_cast<uint8_t>(type.as<unsigned int>()));
            }
            if (vehicle.path.size() != vehicle.path_types.size()) {
                throw std::runtime_error("vehicle path and path_types sizes differ");
            }
            snapshot.vehicles.push_back(std::move(vehicle));
        }
        error = "ok";
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

}  // namespace forklift_nlp_recovery
