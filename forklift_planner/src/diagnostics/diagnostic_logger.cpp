#include "forklift_planner/diagnostics/diagnostic_logger.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "forklift_planner/diagnostics/diagnostic_snapshot.h"

namespace forklift_planner {
namespace diagnostics {

DiagnosticLogger::DiagnosticLogger(std::string log_dir, std::string coordination_file,
                                   bool enabled)
    : log_dir_(std::move(log_dir)), coordination_file_(std::move(coordination_file)),
      rolling_file_(log_dir_ + "/rolling_planning.log"),
      event_file_(log_dir_ + "/exception_events.log"),
      summary_file_(log_dir_ + "/run_summary.log"), coordination_enabled_(enabled) {}

bool DiagnosticLogger::ensureParentDirectory(const std::string& path) {
    if (path.empty()) return false;
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (parent.empty()) return true;
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
        ROS_ERROR("[diagnostics] cannot create directory %s: %s",
                  parent.string().c_str(), error.message().c_str());
        return false;
    }
    return true;
}

bool DiagnosticLogger::initialize(int vehicle_count, int seed, bool one_shot,
                                  bool use_a1_cycle) {
    if ((coordination_enabled_ && !ensureParentDirectory(coordination_file_)) ||
        !ensureParentDirectory(rolling_file_) || !ensureParentDirectory(event_file_) ||
        !ensureParentDirectory(summary_file_)) {
        return false;
    }
    if (coordination_enabled_) {
        coordination_log_.open(coordination_file_, std::ios::out | std::ios::trunc);
    }
    rolling_log_.open(rolling_file_, std::ios::out | std::ios::trunc);
    event_log_.open(event_file_, std::ios::out | std::ios::app);
    summary_log_.open(summary_file_, std::ios::out | std::ios::trunc);
    if ((coordination_enabled_ && !coordination_log_) || !rolling_log_ ||
        !event_log_ || !summary_log_) {
        ROS_WARN("[diagnostics] failed to open one or more classified logs under %s",
                 log_dir_.c_str());
        return false;
    }
    if (coordination_enabled_) {
        coordination_log_ << "[multi_patrol] coordination log started\n"
                          << "vehicle_count=" << vehicle_count
                          << " one_shot=" << (one_shot ? 1 : 0)
                          << " use_a1_cycle=" << (use_a1_cycle ? 1 : 0) << "\n";
    }
    rolling_log_ << "[multi_patrol] rolling planning log started\n";
    summary_log_ << "seed=" << seed << "\nvehicle_count=" << vehicle_count
                 << "\none_shot=" << (one_shot ? 1 : 0)
                 << "\nuse_a1_cycle=" << (use_a1_cycle ? 1 : 0) << "\n";
    if (coordination_enabled_) coordination_log_.flush();
    rolling_log_.flush();
    summary_log_.flush();
    if (coordination_enabled_) {
        ROS_WARN("[diagnostics] coordination log: %s", coordination_file_.c_str());
    }
    ROS_WARN("[diagnostics] rolling log: %s", rolling_file_.c_str());
    ROS_WARN("[diagnostics] event log: %s", event_file_.c_str());
    ROS_WARN("[diagnostics] summary log: %s", summary_file_.c_str());
    return true;
}

std::string DiagnosticLogger::contextualize(const std::string& line) const {
    return contextualize(line, context_);
}

std::string DiagnosticLogger::contextualize(const std::string& line,
                                            const LogContext& context) const {
    std::ostringstream out;
    out << "[SOURCE=" << context.source << "] [plan=" << context.plan_id
        << "] [frame=" << context.frame_id << "] [rollout_step="
        << context.rollout_step << "] [tick=" << context.tick << "] [sim_t="
        << std::fixed << std::setprecision(3) << context.sim_time << "] " << line;
    return out.str();
}

bool DiagnosticLogger::isRollingLine(const std::string& line,
                                     const LogContext& context) {
    return context.source == "ROLLOUT" || line.rfind("[ROLLING", 0) == 0 ||
           line.rfind("[TIMELINE_", 0) == 0;
}

void DiagnosticLogger::write(std::ofstream& stream, const std::string& line) {
    if (suppressed_ || !stream) return;
    stream << line << "\n";
    stream.flush();
}

void DiagnosticLogger::coordination(const std::string& line) {
    coordination(line, context_);
}

void DiagnosticLogger::coordination(const std::string& line, const LogContext& context) {
    if (isRollingLine(line, context)) {
        write(rolling_log_, contextualize(line, context));
    } else if (line.find("HARD_GUARD") != std::string::npos ||
        line.find("PATH_FAILURE") != std::string::npos ||
        line.find("FAILED") != std::string::npos ||
        line.find("[STRESS_FAILURE]") != std::string::npos) {
        write(event_log_, contextualize(line, context));
    } else {
        write(coordination_log_, contextualize(line, context));
    }
}

void DiagnosticLogger::rolling(const std::string& line) {
    write(rolling_log_, contextualize(line));
}

void DiagnosticLogger::event(const std::string& line) {
    write(event_log_, contextualize(line));
}

void DiagnosticLogger::eventHistory(const std::string& header,
                                    const std::deque<std::string>& history) {
    event("========== " + header + " ==========");
    for (const auto& snapshot : history) event(snapshot);
}

void DiagnosticLogger::summary(const std::string& line) {
    write(summary_log_, line);
}

void DiagnosticLogger::runSummary(
    const std::string& status, const std::string& failure_type,
    unsigned long long tick, double sim_time,
    unsigned long long hard_guard_events, unsigned long long wedge_episodes,
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const std::vector<double>& max_wait) {
    summary("status=" + status);
    summary("failure_type=" + (failure_type.empty() ? std::string("none") : failure_type));
    summary("ticks=" + std::to_string(tick));
    summary("sim_time_s=" + std::to_string(sim_time));
    summary("hard_guard_events=" + std::to_string(hard_guard_events));
    summary("wedge_episodes=" + std::to_string(wedge_episodes));
    for (size_t i = 0; i < vehicles.size(); ++i) {
        summary("V" + std::to_string(vehicles[i].id) +
                "_tasks=" + std::to_string(vehicles[i].task_count));
        summary("V" + std::to_string(vehicles[i].id) + "_max_wait_s=" +
                std::to_string(i < max_wait.size() ? max_wait[i] : 0.0));
    }
}

std::ofstream& DiagnosticLogger::vehicleStream(int vehicle_id) {
    auto inserted = vehicle_logs_.try_emplace(vehicle_id);
    std::ofstream& stream = inserted.first->second;
    if (!inserted.second) return stream;
    const std::string path = log_dir_ + "/vehicle_state_V" +
                             std::to_string(vehicle_id) + ".csv";
    if (!ensureParentDirectory(path)) return stream;
    stream.open(path, std::ios::out | std::ios::trunc);
    if (stream) {
        stream << "sim_time,tick,vehicle_id,plan_id,frame_id,mode,mission_phase,"
               << "leg_target,current_slot,target_slot,task_count,path_gen,path_s,"
               << "path_length,remaining_s,x,y,yaw,real_pose,speed,action,requested_action,"
               << "blocker_id,wait_time,dwell_remaining,reason\n";
    } else {
        ROS_WARN("[diagnostics] failed to open vehicle state log: %s", path.c_str());
    }
    return stream;
}

void DiagnosticLogger::vehicleState(const multi_vehicle::VehicleAgent& vehicle) {
    if (suppressed_) return;
    double x = std::numeric_limits<double>::quiet_NaN();
    double y = x;
    double yaw = x;
    if (vehicle.real_pose_valid) {
        x = vehicle.real_x;
        y = vehicle.real_y;
        yaw = vehicle.real_yaw;
    } else if (!vehicle.track.empty()) {
        const double s = vehicle.mode == multi_vehicle::VehicleMode::DWELL
            ? vehicle.track.length() : std::min(vehicle.path_s, vehicle.track.length());
        const RoughWp pose = vehicle.track.poseAtS(s);
        x = pose.x;
        y = pose.y;
        yaw = pose.theta;
    }
    const double length = vehicle.track.empty() ? 0.0 : vehicle.track.length();
    std::ofstream& stream = vehicleStream(vehicle.id);
    if (!stream) return;
    stream << std::setprecision(15) << context_.sim_time << "," << context_.tick << ","
           << vehicle.id << "," << context_.plan_id << "," << context_.frame_id << ","
           << modeName(vehicle.mode) << "," << missionPhaseName(vehicle.mission_phase) << ","
           << legTargetName(vehicle.leg_target) << "," << vehicle.current_slot << ","
           << vehicle.target_slot << "," << vehicle.task_count << "," << vehicle.path_gen
           << "," << vehicle.path_s << "," << length << ","
           << (vehicle.track.empty() ? 0.0 : vehicle.remainingS()) << "," << x << "," << y
           << "," << yaw << "," << (vehicle.real_pose_valid ? 1 : 0) << ","
           << vehicle.current_speed << "," << multi_vehicle::actionName(vehicle.action) << ","
           << multi_vehicle::actionName(vehicle.requested_action) << "," << vehicle.blocker_id
           << "," << vehicle.wait_time << "," << vehicle.dwell_remaining << ",\""
           << vehicle.reason << "\"\n";
    stream.flush();
}

std::ofstream& DiagnosticLogger::projectionStream(int vehicle_id) {
    auto inserted = projection_logs_.try_emplace(vehicle_id);
    std::ofstream& stream = inserted.first->second;
    if (!inserted.second) return stream;
    const std::string path = log_dir_ + "/real_projection_V" +
                             std::to_string(vehicle_id) + ".csv";
    if (!ensureParentDirectory(path)) return stream;
    stream.open(path, std::ios::out | std::ios::trunc);
    if (stream) {
        stream << "wall_time,sim_time,tick,vehicle_id,real_x,real_y,real_yaw,"
               << "path_gen,mode,mission_phase,leg_target,previous_path_s,"
               << "projected_path_s,delta_s,projected_x,projected_y,projected_yaw,"
               << "projection_distance,yaw_error_to_projected_path,wp_type,current_speed,"
               << "action,real_plan_id,frame_id,search_s_min,search_s_max,"
               << "raw_single_step_speed,window_speed,speed_window_duration,"
               << "speed_window_samples,best_xy_distance,selected_heading_error,"
               << "selected_continuity_error,projection_candidate_count\n";
        ROS_WARN("[real_projection] V%d log: %s", vehicle_id, path.c_str());
    } else {
        ROS_WARN("[real_projection] failed to open %s", path.c_str());
    }
    return stream;
}

void DiagnosticLogger::realProjection(
    const multi_vehicle::VehicleAgent& vehicle, double real_x, double real_y,
    double real_yaw, double previous_path_s, double projected_path_s,
    double search_s_min, double search_s_max,
    const multi_vehicle::ArcLengthSpeedResult& speed,
    const multi_vehicle::RealProjectionResult& projection, double wall_time) {
    if (suppressed_) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    RoughWp projected{nan, nan, nan, WpType::FORWARD};
    const char* wp_type = "NONE";
    if (!vehicle.track.empty()) {
        const double s = std::max(0.0, std::min(projected_path_s, vehicle.track.length()));
        projected = vehicle.track.poseAtS(s);
        wp_type = vehicle.track.typeAtS(s) == WpType::REVERSE ? "REVERSE" : "FORWARD";
    }
    const double projection_distance = std::hypot(projected.x - real_x, projected.y - real_y);
    const double yaw_error = std::atan2(std::sin(real_yaw - projected.theta),
                                        std::cos(real_yaw - projected.theta));
    std::ofstream& stream = projectionStream(vehicle.id);
    if (!stream) return;
    stream << std::setprecision(15) << wall_time << "," << context_.sim_time << ","
           << context_.tick << "," << vehicle.id << "," << real_x << "," << real_y << ","
           << real_yaw << "," << vehicle.path_gen << "," << static_cast<int>(vehicle.mode)
           << "," << static_cast<int>(vehicle.mission_phase) << ","
           << static_cast<int>(vehicle.leg_target) << "," << previous_path_s << ","
           << projected_path_s << "," << (projected_path_s - previous_path_s) << ","
           << projected.x << "," << projected.y << "," << projected.theta << ","
           << projection_distance << "," << yaw_error << "," << wp_type << ","
           << vehicle.current_speed << "," << multi_vehicle::actionName(vehicle.action) << ","
           << context_.plan_id << "," << context_.frame_id << "," << search_s_min << ","
           << search_s_max << "," << speed.raw_single_step_speed << "," << speed.window_speed
           << "," << speed.window_duration << "," << speed.window_samples << ","
           << projection.best_xy_distance << "," << projection.selected_heading_error << ","
           << projection.selected_continuity_error << "," << projection.candidate_count << "\n";
    stream.flush();
}

bool DiagnosticLogger::writeStressResult(
    const std::string& path, const std::string& status, const std::string& failure_type,
    int seed, unsigned long long tick, double sim_time,
    unsigned long long hard_guard_events, unsigned long long wedge_episodes,
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const std::vector<double>& max_wait) const {
    if (path.empty() || !ensureParentDirectory(path)) return false;
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    out << "status=" << status << "\nfailure_type="
        << (failure_type.empty() ? "none" : failure_type) << "\nseed=" << seed
        << "\nvehicle_count=" << vehicles.size() << "\nticks=" << tick
        << "\nsim_time_s=" << std::fixed << std::setprecision(3) << sim_time
        << "\nhard_guard_events=" << hard_guard_events
        << "\nwedge_episodes=" << wedge_episodes << "\n";
    for (size_t i = 0; i < vehicles.size(); ++i) {
        out << "V" << vehicles[i].id << "_tasks=" << vehicles[i].task_count
            << "\nV" << vehicles[i].id << "_max_wait_s="
            << (i < max_wait.size() ? max_wait[i] : 0.0) << "\n";
    }
    return true;
}

bool DiagnosticLogger::writeStressFailure(
    const std::string& path, const std::string& failure_type, int seed,
    double progress_timeout, const std::deque<std::string>& ring,
    const std::string& failure_snapshot) const {
    if (path.empty() || !ensureParentDirectory(path)) return false;
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    out << "failure_type=" << failure_type << "\nseed=" << seed
        << "\nring_seconds=120\nprogress_timeout_s=" << progress_timeout
        << "\n\n===== PRE-FAILURE RING =====\n";
    for (const auto& frame : ring) out << frame << "\n---\n";
    out << "===== FAILURE GEOMETRY =====\n" << failure_snapshot << "\n";
    return true;
}

void DiagnosticLogger::truncateEventLog() {
    event_log_.close();
    event_log_.open(event_file_, std::ios::out | std::ios::trunc);
}

}  // namespace diagnostics
}  // namespace forklift_planner
