#include "forklift_planner/diagnostics/diagnostic_logger.h"

#include <ros/ros.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include "forklift_planner/diagnostics/diagnostic_snapshot.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"

#ifndef FORKLIFT_PLANNER_GIT_COMMIT
#define FORKLIFT_PLANNER_GIT_COMMIT "unknown"
#endif

namespace forklift_planner {
namespace diagnostics {
namespace {

constexpr double kHistorySeconds = 300.0;

std::string blockerName(int id) {
    return id < 0 ? "none" : "V" + std::to_string(id);
}

std::string reasonModule(const std::string& reason) {
    std::string lower = reason;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower.find("deadlock") != std::string::npos ||
        lower.find("retreat") != std::string::npos) return "deadlock";
    if (lower.find("a1") != std::string::npos ||
        lower.find("departure") != std::string::npos) return "A1";
    if (lower.find("hard") != std::string::npos ||
        lower.find("guard") != std::string::npos ||
        lower.find("invariant") != std::string::npos) return "safety";
    if (lower == "clear" || lower.empty()) return "none";
    return "coordination";
}

const char* recoveryMotionName(multi_vehicle::RecoveryMotion motion) {
    switch (motion) {
        case multi_vehicle::RecoveryMotion::NORMAL: return "NORMAL";
        case multi_vehicle::RecoveryMotion::HOLD: return "HOLD";
        case multi_vehicle::RecoveryMotion::RETREAT: return "RETREAT";
    }
    return "UNKNOWN";
}

}  // namespace

DiagnosticLogger::DiagnosticLogger(std::string log_dir, bool enabled)
    : log_dir_(std::move(log_dir)), coordination_enabled_(enabled) {}

DiagnosticLogger::~DiagnosticLogger() {
    finalize(context_.sim_time);
    if (initialized_ && !run_summary_written_ && summary_log_) {
        summary_log_ << "[" << readableSimTime(context_.sim_time)
                     << "] [END] run_id=" << run_id_
                     << " result=INTERRUPTED tick=" << context_.tick << "\n";
        summary_log_.flush();
    }
    if (incident_active_) closeIncident("INTERRUPTED");
}

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

std::string DiagnosticLogger::makeRunDirectory() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t stamp = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &stamp);
#else
    localtime_r(&stamp, &tm);
#endif
    std::ostringstream base;
    base << std::put_time(&tm, "%Y%m%d_%H%M%S");
    for (int suffix = 0; suffix < 10000; ++suffix) {
        run_id_ = base.str();
        if (suffix != 0) run_id_ += "_" + std::to_string(suffix);
        const std::filesystem::path candidate =
            std::filesystem::path(log_dir_) / ("run_" + run_id_);
        std::error_code error;
        if (std::filesystem::create_directories(candidate, error)) {
            return candidate.string();
        }
        if (error && !std::filesystem::exists(candidate)) return "";
    }
    return "";
}

bool DiagnosticLogger::initialize(int vehicle_count, int seed, bool one_shot,
                                  bool use_a1_cycle, bool real_mode) {
    if (!coordination_enabled_) return true;
    real_mode_ = real_mode;
    run_dir_ = makeRunDirectory();
    if (run_dir_.empty()) {
        ROS_ERROR("[diagnostics] cannot create unique run directory below %s",
                  log_dir_.c_str());
        coordination_enabled_ = false;
        return false;
    }
    const std::filesystem::path root(run_dir_);
    summary_file_ = (root / "summary.log").string();
    coordination_file_ = (root / "coordination.log").string();
    safety_file_ = (root / "safety.log").string();
    summary_log_.open(summary_file_, std::ios::out | std::ios::trunc);
    if (!summary_log_) {
        ROS_ERROR("[diagnostics] cannot open %s", summary_file_.c_str());
        coordination_enabled_ = false;
        return false;
    }
    std::error_code error;
    std::filesystem::create_directories(root / "incidents", error);
    if (error) ROS_ERROR("[diagnostics] cannot create incidents directory: %s",
                         error.message().c_str());

    module_streams_[StreamKind::A1].logical_name = "A1";
    module_streams_[StreamKind::COORDINATION].logical_name = "coordination";
    module_streams_[StreamKind::DEADLOCK].logical_name = "deadlock";
    module_streams_[StreamKind::SAFETY].logical_name = "safety";
    for (auto& item : module_streams_) rotate(item.second, 0.0, false);

    summary_log_ << "[0min00.0s] [START] run_id=" << run_id_ << " seed=" << seed
                 << " vehicles=" << vehicle_count << " commit="
                 << FORKLIFT_PLANNER_GIT_COMMIT << " one_shot=" << (one_shot ? 1 : 0)
                 << " use_a1_cycle=" << (use_a1_cycle ? 1 : 0) << "\n";
    summary_log_.flush();
    initialized_ = true;
    ROS_INFO("[diagnostics] run=%s directory=%s", run_id_.c_str(), run_dir_.c_str());
    return true;
}

void DiagnosticLogger::registerVehicles(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles) {
    if (!initialized_) return;
    std::ostringstream ids;
    for (size_t i = 0; i < vehicles.size(); ++i) {
        if (i != 0) ids << ",";
        ids << "V" << vehicles[i].id;
        rotate(vehicleStream(vehicles[i].id, false), context_.sim_time, false);
        if (real_mode_) rotate(vehicleStream(vehicles[i].id, true), context_.sim_time, true);
    }
    summary("VEHICLES ids=" + ids.str());
}

std::string DiagnosticLogger::contextualize(const std::string& line) const {
    return contextualize(line, context_);
}

std::string DiagnosticLogger::contextualize(const std::string& line,
                                            const LogContext& context) const {
    return formatRecord(eventName(line), line, context, "-");
}

std::string DiagnosticLogger::sanitizeValue(const std::string& value) {
    std::string output = value;
    for (char& c : output) {
        if (c == '\n' || c == '\r' || c == '\t') c = ' ';
    }
    return output;
}

std::string DiagnosticLogger::eventName(const std::string& line) {
    const size_t event = line.find("event=");
    if (event != std::string::npos) {
        const size_t begin = event + 6;
        const size_t end = line.find_first_of(" \t]", begin);
        return line.substr(begin, end == std::string::npos ? end : end - begin);
    }
    const size_t open = line.find('[');
    const size_t close = open == std::string::npos ? std::string::npos : line.find(']', open);
    if (open != std::string::npos && close != std::string::npos) {
        std::string name = line.substr(open + 1, close - open - 1);
        const size_t nested = line.find('[', close + 1);
        const size_t nested_close = nested == std::string::npos
            ? std::string::npos : line.find(']', nested + 1);
        if ((name == "multi_patrol" || name == "SOURCE=REAL") &&
            nested != std::string::npos && nested_close != std::string::npos) {
            name = line.substr(nested + 1, nested_close - nested - 1);
        }
        for (char& c : name) {
            if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
        }
        return name.empty() ? "EVENT" : name;
    }
    return "EVENT";
}

std::string DiagnosticLogger::nextEventId() {
    return "E" + std::to_string(++event_id_);
}

std::string DiagnosticLogger::formatRecord(const std::string& event,
                                           const std::string& details,
                                           const LogContext& context,
                                           const std::string& event_id) const {
    std::ostringstream out;
    out << "[" << readableSimTime(context.sim_time) << "] [" << event << "]";
    if (event_id != "-") out << " event_id=" << event_id;
    out << " tick=" << context.tick << " plan_id=" << context.plan_id;
    if (context.source == "ROLLOUT") {
        out << " source=PREDICTED";
    } else if (context.source == "ROLLOUT_BASIS") {
        out << " source=PREDICTED_BASIS";
    }
    if (context.frame_id >= 0) out << " frame=" << context.frame_id;
    if (context.rollout_step >= 0) out << " rollout_step=" << context.rollout_step;
    if (!details.empty()) out << " " << sanitizeValue(details);
    return out.str();
}

DiagnosticLogger::StreamKind DiagnosticLogger::classify(const std::string& line) const {
    if (line.find("[DEADLOCK]") != std::string::npos) return StreamKind::DEADLOCK;
    if (line.find("HARD_GUARD") != std::string::npos ||
        line.find("PATH_FAILURE") != std::string::npos ||
        line.find("ROLLING_PLAN_FAILURE") != std::string::npos ||
        line.find("STRESS_FAILURE") != std::string::npos ||
        line.find("INVARIANT_VIOLATION") != std::string::npos) {
        return StreamKind::SAFETY;
    }
    if (line.find("[A1") != std::string::npos ||
        line.find("A1_") != std::string::npos ||
        line.find("A1 ") != std::string::npos ||
        line.find("FUTURE_A1") != std::string::npos ||
        line.find("DEPARTURE_CLUSTER") != std::string::npos ||
        line.find("SLOT_DEPARTURE") != std::string::npos ||
        line.find("PREPARE_DROPOFF") != std::string::npos ||
        line.find("ACTIVATE_DROPOFF") != std::string::npos) {
        return StreamKind::A1;
    }
    return StreamKind::COORDINATION;
}

DiagnosticLogger::RotatingStream& DiagnosticLogger::moduleStream(StreamKind kind) {
    return module_streams_[kind];
}

DiagnosticLogger::RotatingStream& DiagnosticLogger::vehicleStream(int vehicle_id,
                                                                  bool trajectory) {
    auto& streams = trajectory ? trajectory_streams_ : vehicle_streams_;
    RotatingStream& stream = streams[vehicle_id];
    if (stream.logical_name.empty()) {
        stream.logical_name = "V" + std::to_string(vehicle_id) +
                              (trajectory ? "_trajectory" : "");
    }
    return stream;
}

void DiagnosticLogger::rotate(RotatingStream& stream, double sim_time, bool csv) {
    if (!initialized_ && run_dir_.empty()) return;
    if (!stream.path.empty()) return;
    stream.csv = csv;
    stream.vehicle = !csv && !stream.logical_name.empty() && stream.logical_name[0] == 'V';
    const std::string extension = csv ? ".csv" : ".log";
    stream.path = (std::filesystem::path(run_dir_) /
                   (stream.logical_name + extension)).string();
    std::ofstream file(stream.path, std::ios::out | std::ios::trunc);
    if (!file) {
        ROS_ERROR_THROTTLE(5.0, "[diagnostics] cannot open %s; logging degraded",
                           stream.path.c_str());
        stream.path.clear();
        return;
    }
    if (csv) file << "sim_t,tick,vehicle_id,path_gen,x,y,yaw,path_s,speed,mode,phase,action,reason,blocker_id\n";
}

void DiagnosticLogger::writeRotating(RotatingStream& stream, double sim_time,
                                     const std::string& record, bool csv,
                                     bool deduplicate) {
    if (suppressed_ || !coordination_enabled_) return;
    rotate(stream, sim_time, csv);
    if (stream.path.empty()) return;
    if (deduplicate && record == stream.last_record) return;
    stream.records.push_back(TimedRecord{sim_time, record});
    stream.dirty = true;
    stream.last_record = record;
}

void DiagnosticLogger::advanceWindow(double endpoint) {
    final_sim_time_ = std::max(final_sim_time_, endpoint);
    const double cutoff = std::max(0.0, final_sim_time_ - kHistorySeconds);
    auto prune = [cutoff](RotatingStream& stream) {
        while (!stream.records.empty() && stream.records.front().sim_time + 1e-9 < cutoff) {
            if (stream.vehicle &&
                stream.records.front().text.find("] [STATE]") != std::string::npos) {
                stream.baseline_state = stream.records.front().text;
                stream.baseline_state_time = stream.records.front().sim_time;
            }
            stream.records.pop_front();
            stream.dirty = true;
        }
    };
    for (auto& item : module_streams_) prune(item.second);
    for (auto& item : vehicle_streams_) prune(item.second);
    for (auto& item : trajectory_streams_) prune(item.second);
    while (!history_.empty() && history_.front().first + 1e-9 < cutoff) {
        history_.pop_front();
    }
}

void DiagnosticLogger::rewriteStream(RotatingStream& stream, double endpoint) {
    if (stream.path.empty()) return;
    std::ofstream file(stream.path, std::ios::out | std::ios::trunc);
    if (!file) {
        ROS_ERROR_THROTTLE(5.0, "[diagnostics] cannot rewrite %s",
                           stream.path.c_str());
        return;
    }
    if (stream.csv) {
        file << "sim_t,tick,vehicle_id,path_gen,x,y,yaw,path_s,speed,mode,phase,action,reason,blocker_id\n";
    } else if (stream.vehicle && !stream.baseline_state.empty() && endpoint > kHistorySeconds) {
        const double cutoff = endpoint - kHistorySeconds;
        const size_t state_marker = stream.baseline_state.find("] [STATE]");
        const std::string state = state_marker == std::string::npos
            ? stream.baseline_state
            : stream.baseline_state.substr(state_marker + 9);
        file << "[" << readableSimTime(cutoff)
             << "] [WINDOW_BASELINE] carried_from="
             << readableSimTime(stream.baseline_state_time) << " "
             << state << "\n";
    }
    for (const TimedRecord& record : stream.records) file << record.text << "\n";
    file.flush();
    stream.dirty = false;
}

void DiagnosticLogger::flushWindow(double endpoint, bool force) {
    advanceWindow(endpoint);
    if (!force && last_flush_time_ >= 0.0 &&
        endpoint - last_flush_time_ < 1.0 - 1e-9) {
        return;
    }
    for (auto& item : module_streams_) {
        if (force || item.second.dirty) rewriteStream(item.second, final_sim_time_);
    }
    for (auto& item : vehicle_streams_) {
        if (force || item.second.dirty) rewriteStream(item.second, final_sim_time_);
    }
    for (auto& item : trajectory_streams_) {
        if (force || item.second.dirty) rewriteStream(item.second, final_sim_time_);
    }
    for (auto& item : projection_logs_) {
        if (item.second) item.second.flush();
    }
    last_flush_time_ = endpoint;
}

void DiagnosticLogger::appendHistory(const std::string& record, double sim_time) {
    if (suppressed_) return;
    history_.emplace_back(sim_time, record);
    while (!history_.empty() && sim_time - history_.front().first > kHistorySeconds) {
        history_.pop_front();
    }
    while (history_.size() > 150000) history_.pop_front();
    if (incident_active_ && incident_log_) incident_log_ << record << "\n";
}

void DiagnosticLogger::writeModule(StreamKind kind, const std::string& event,
                                   const std::string& details,
                                   const LogContext& context) {
    RotatingStream& stream = moduleStream(kind);
    if (module_dedup_tick_ != context.tick) {
        module_dedup_tick_ = context.tick;
        current_tick_module_records_.clear();
    }
    const std::string dedup_key = std::to_string(static_cast<int>(kind)) + "\n" + details;
    if (!current_tick_module_records_.insert(dedup_key).second) return;
    const std::string record = formatRecord(event, details, context, nextEventId());
    writeRotating(stream, context.sim_time, record);
    stream.last_record = details;
    appendHistory(record, context.sim_time);
}

bool DiagnosticLogger::isIncidentLine(const std::string& line) {
    return line.find("HARD_GUARD") != std::string::npos ||
           line.find("PATH_FAILURE") != std::string::npos ||
           line.find("ROLLING_PLAN_FAILURE") != std::string::npos ||
           line.find("INVARIANT_VIOLATION") != std::string::npos ||
           line.find("event=UNRESOLVED") != std::string::npos ||
           line.find("event=ABORT") != std::string::npos ||
           line.find("STRESS_FAILURE") != std::string::npos;
}

bool DiagnosticLogger::keepRolloutEvidence(const std::string& line,
                                           const LogContext& context) const {
    if (context.source != "ROLLOUT") return true;
    if (context.frame_id != 0) return false;
    return line.find("[DYN-TTC]") != std::string::npos ||
           line.find("[DYN-SPEED]") != std::string::npos ||
           line.find("[DYN-PHYSICAL]") != std::string::npos ||
           line.find("[BRIDGE-TTC]") != std::string::npos ||
           line.find("[FORWARD_CLEARANCE]") != std::string::npos ||
           line.find("[TARGET_SLOT_OCCUPANCY]") != std::string::npos ||
           line.find("[CONFLICT_RESERVATION]") != std::string::npos ||
           line.find("[ACTION_REQUEST]") != std::string::npos ||
           line.find("[ACTION_HOLD]") != std::string::npos ||
           line.find("[A1") != std::string::npos ||
           line.find("[FUTURE_A1]") != std::string::npos ||
           line.find("[DEPARTURE_CLUSTER]") != std::string::npos ||
           line.find("[DEADLOCK]") != std::string::npos ||
           line.find("[PATH_FAILURE]") != std::string::npos;
}

bool DiagnosticLogger::suppressRepeatedDiagnostic(const std::string& line,
                                                  const LogContext& context) const {
    if (context.source != "REAL") return false;
    return line.rfind("[multi_patrol][state]", 0) == 0 ||
           (line.find("[ACTION_REQUEST]") != std::string::npos &&
            line.find("result=COVERED_BY_EXISTING") != std::string::npos);
}

void DiagnosticLogger::promoteRolloutEvidence(uint64_t plan_id,
                                              const LogContext& context) {
    const auto found = rollout_evidence_.find(plan_id);
    if (found == rollout_evidence_.end()) return;
    for (const RolloutEvidence& evidence : found->second) {
        LogContext committed = context;
        committed.source = "ROLLOUT_BASIS";
        committed.frame_id = evidence.context.frame_id;
        committed.rollout_step = evidence.context.rollout_step;
        writeModule(classify(evidence.line), eventName(evidence.line),
                    evidence.line, committed);
    }
    rollout_evidence_.erase(found);
}

void DiagnosticLogger::coordination(const std::string& line) {
    coordination(line, context_);
}

void DiagnosticLogger::coordination(const std::string& line,
                                    const LogContext& context) {
    if (suppressed_ || !coordination_enabled_ ||
        !keepRolloutEvidence(line, context) ||
        suppressRepeatedDiagnostic(line, context)) return;

    if (context.source == "ROLLOUT") {
        auto& evidence = rollout_evidence_[context.plan_id];
        const bool duplicate = std::any_of(
            evidence.begin(), evidence.end(), [&line](const RolloutEvidence& item) {
                return item.line == line;
            });
        if (!duplicate) evidence.push_back(RolloutEvidence{line, context});
        while (rollout_evidence_.size() > 8) rollout_evidence_.erase(rollout_evidence_.begin());
        return;
    }

    if (line.find("[ROLLING-DECISION]") != std::string::npos) {
        promoteRolloutEvidence(context.plan_id, context);
    }
    writeModule(classify(line), eventName(line), line, context);
    if (isIncidentLine(line)) {
        const LogContext previous = context_;
        context_ = context;
        startIncident("SAFETY_OR_SYSTEM_EVENT", line, "");
        context_ = previous;
    }
    if (real_mode_) flushWindow(context.sim_time, false);
}

void DiagnosticLogger::rolling(const std::string& line) {
    coordination(line, context_);
}

void DiagnosticLogger::event(const std::string& line) {
    coordination(line, context_);
}

void DiagnosticLogger::eventHistory(const std::string& header,
                                    const std::deque<std::string>& history) {
    event("[HISTORY] header=\"" + header + "\"");
    for (const auto& snapshot : history) event("[HISTORY_SAMPLE] " + snapshot);
}

void DiagnosticLogger::summary(const std::string& line) {
    if (!summary_log_) return;
    summary_log_ << "[" << readableSimTime(context_.sim_time)
                 << "] [SUMMARY] " << sanitizeValue(line) << "\n";
    summary_log_.flush();
}

void DiagnosticLogger::runSummary(
    const std::string& status, const std::string& failure_type,
    unsigned long long tick, double sim_time,
    unsigned long long hard_guard_events, unsigned long long wedge_episodes,
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const std::vector<double>& max_wait) {
    if (!summary_log_) return;
    summary_log_ << "[" << readableSimTime(sim_time)
                 << "] [END] run_id=" << run_id_ << " result=" << status
                 << " failure=" << (failure_type.empty() ? "none" : failure_type)
                 << " tick=" << tick << " hard_guard=" << hard_guard_events
                 << " deadlock=" << wedge_episodes << "\n";
    for (size_t i = 0; i < vehicles.size(); ++i) {
        summary_log_ << "[" << readableSimTime(sim_time)
                     << "] [VEHICLE_RESULT] vehicle_id=V"
                     << vehicles[i].id << " tasks=" << vehicles[i].task_count
                     << " max_wait=" << (i < max_wait.size() ? max_wait[i] : 0.0) << "\n";
    }
    summary_log_.flush();
    run_summary_written_ = true;
    if (incident_active_) closeIncident(status == "PASS" ? "RECOVERED" : "FAILED");
    finalize(sim_time);
}

std::string DiagnosticLogger::routeName(const multi_vehicle::VehicleAgent& vehicle) {
    if (vehicle.mission_phase == multi_vehicle::MissionPhase::TO_A1 ||
        vehicle.mission_phase == multi_vehicle::MissionPhase::PICKUP_DWELL) {
        return "B" + std::to_string(vehicle.current_slot) + "->A1";
    }
    if (vehicle.mission_phase == multi_vehicle::MissionPhase::TO_B ||
        vehicle.mission_phase == multi_vehicle::MissionPhase::UNLOAD_DWELL ||
        vehicle.mission_phase == multi_vehicle::MissionPhase::WAIT_DROPOFF_TASK) {
        return "A1->B" + std::to_string(vehicle.target_slot);
    }
    return "B" + std::to_string(vehicle.current_slot) + "->B" +
           std::to_string(vehicle.target_slot);
}

void DiagnosticLogger::writeVehicleSnapshot(
    const multi_vehicle::VehicleAgent& vehicle, VehicleHistoryState& previous,
    bool force) {
    const bool identity_changed = !previous.initialized || previous.mode != vehicle.mode ||
        previous.phase != vehicle.mission_phase || previous.task_count != vehicle.task_count ||
        previous.path_gen != vehicle.path_gen;
    const bool action_changed = !previous.initialized || previous.action != vehicle.action ||
        previous.reason != vehicle.reason || previous.blocker_id != vehicle.blocker_id;
    RotatingStream& stream = vehicleStream(vehicle.id, false);

    if (previous.initialized && identity_changed) {
        const std::string event = previous.mode == multi_vehicle::VehicleMode::ACTIVE
            ? "TASK_END" : "TASK_STATE_CHANGE";
        std::ostringstream details;
        details << "vehicle_id=V" << vehicle.id << " route=" << routeName(vehicle)
                << " task=" << vehicle.task_count << " path_gen=" << vehicle.path_gen
                << " mode=" << modeName(previous.mode) << "->" << modeName(vehicle.mode)
                << " phase=" << missionPhaseName(previous.phase) << "->"
                << missionPhaseName(vehicle.mission_phase);
        const std::string record = formatRecord(event, details.str(), context_, nextEventId());
        writeRotating(stream, context_.sim_time, record);
        appendHistory(record, context_.sim_time);
    }
    if (identity_changed && vehicle.mode == multi_vehicle::VehicleMode::ACTIVE) {
        std::ostringstream details;
        details << "vehicle_id=V" << vehicle.id << " route=" << routeName(vehicle)
                << " task=" << vehicle.task_count << " path_gen=" << vehicle.path_gen;
        const std::string record = formatRecord("TASK_START", details.str(), context_, nextEventId());
        writeRotating(stream, context_.sim_time, record);
        appendHistory(record, context_.sim_time);
    }
    if (action_changed && previous.initialized) {
        std::ostringstream details;
        details << "vehicle_id=V" << vehicle.id << " "
                << multi_vehicle::actionName(previous.action) << "->"
                << multi_vehicle::actionName(vehicle.action)
                << " reason=" << (vehicle.reason.empty() ? "clear" : vehicle.reason)
                << " blocker=" << blockerName(vehicle.blocker_id)
                << " path_gen=" << vehicle.path_gen;
        if (previous.action == multi_vehicle::VehicleAction::STOP) {
            details << " stop_duration=" << std::fixed << std::setprecision(3)
                    << (context_.sim_time - previous.constraint_since);
        }
        const std::string record = formatRecord("ACTION_CHANGE", details.str(), context_, nextEventId());
        writeRotating(stream, context_.sim_time, record);
        appendHistory(record, context_.sim_time);
    }
    if (vehicle.action == multi_vehicle::VehicleAction::STOP &&
        (!previous.initialized || previous.action != multi_vehicle::VehicleAction::STOP)) {
        previous.constraint_since = context_.sim_time;
        previous.constraint_id = "C" + std::to_string(++constraint_id_);
    }
    if (force || identity_changed || action_changed || previous.last_snapshot_time < 0.0 ||
        context_.sim_time - previous.last_snapshot_time >= 1.0 - 1e-9) {
        previous.last_snapshot_time = context_.sim_time;
        const double length = vehicle.track.empty() ? 0.0 : vehicle.track.length();
        const double no_progress = context_.sim_time - previous.last_progress_time;
        std::ostringstream details;
        details << std::fixed << std::setprecision(3)
                << "vehicle_id=V" << vehicle.id << " route=" << routeName(vehicle)
                << " mode=" << modeName(vehicle.mode)
                << " phase=" << missionPhaseName(vehicle.mission_phase)
                << " action=" << multi_vehicle::actionName(vehicle.action)
                << " reason=" << (vehicle.reason.empty() ? "none" : vehicle.reason)
                << " blocker=" << blockerName(vehicle.blocker_id)
                << " task=" << vehicle.task_count << " path_gen=" << vehicle.path_gen
                << " path_s=" << vehicle.path_s << "/" << length
                << " speed=" << vehicle.current_speed << " wait_time=" << vehicle.wait_time
                << " stop_duration="
                << (vehicle.action == multi_vehicle::VehicleAction::STOP
                        ? context_.sim_time - previous.constraint_since : 0.0)
                << " no_progress=" << no_progress
                << " dwell=" << vehicle.dwell_remaining;
        if (vehicle.action == multi_vehicle::VehicleAction::STOP) {
            details << " constraint_id=" << previous.constraint_id;
        }
        const std::string record = formatRecord("STATE", details.str(), context_, nextEventId());
        writeRotating(stream, context_.sim_time, record);
        appendHistory(record, context_.sim_time);
    }
}

void DiagnosticLogger::writeTrajectory(const multi_vehicle::VehicleAgent& vehicle) {
    if (!real_mode_) return;
    VehicleHistoryState& state = vehicle_history_[vehicle.id];
    if (state.last_trajectory_tick == context_.tick) return;
    state.last_trajectory_tick = context_.tick;
    double x = std::numeric_limits<double>::quiet_NaN();
    double y = x;
    double yaw = x;
    if (vehicle.real_pose_valid) {
        x = vehicle.real_x;
        y = vehicle.real_y;
        yaw = vehicle.real_yaw;
    } else if (!vehicle.track.empty()) {
        const RoughWp pose = vehicle.track.poseAtS(
            std::max(0.0, std::min(vehicle.path_s, vehicle.track.length())));
        x = pose.x;
        y = pose.y;
        yaw = pose.theta;
    }
    std::ostringstream row;
    row << std::setprecision(15) << context_.sim_time << "," << context_.tick << ","
        << vehicle.id << "," << vehicle.path_gen << "," << x << "," << y << ","
        << yaw << "," << vehicle.path_s << "," << vehicle.current_speed << ","
        << modeName(vehicle.mode) << "," << missionPhaseName(vehicle.mission_phase) << ","
        << multi_vehicle::actionName(vehicle.action) << ",\""
        << sanitizeValue(vehicle.reason) << "\"," << vehicle.blocker_id;
    writeRotating(vehicleStream(vehicle.id, true), context_.sim_time, row.str(), true);
}

void DiagnosticLogger::vehicleState(const multi_vehicle::VehicleAgent& vehicle) {
    if (suppressed_) return;
    VehicleHistoryState& previous = vehicle_history_[vehicle.id];
    const bool progressed = !previous.initialized ||
        std::abs(vehicle.path_s - previous.path_s) > 1e-4 ||
        vehicle.path_gen != previous.path_gen || vehicle.task_count != previous.task_count ||
        vehicle.mission_phase != previous.phase || vehicle.mode != previous.mode;
    if (progressed || vehicle.mode != multi_vehicle::VehicleMode::ACTIVE) {
        previous.last_progress_time = context_.sim_time;
        previous.no_progress_reported = false;
    }
    writeVehicleSnapshot(vehicle, previous, false);
    writeTrajectory(vehicle);
    previous.initialized = true;
    previous.mode = vehicle.mode;
    previous.phase = vehicle.mission_phase;
    previous.action = vehicle.action;
    previous.blocker_id = vehicle.blocker_id;
    previous.path_gen = vehicle.path_gen;
    previous.task_count = vehicle.task_count;
    previous.current_slot = vehicle.current_slot;
    previous.target_slot = vehicle.target_slot;
    previous.path_s = vehicle.path_s;
    previous.reason = vehicle.reason;
    if (real_mode_) flushWindow(context_.sim_time, false);
}

std::string DiagnosticLogger::waitCycleSignature(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles) const {
    std::unordered_map<int, int> edges;
    for (const auto& vehicle : vehicles) {
        if (vehicle.mode == multi_vehicle::VehicleMode::ACTIVE &&
            vehicle.action == multi_vehicle::VehicleAction::STOP && vehicle.blocker_id >= 0) {
            edges[vehicle.id] = vehicle.blocker_id;
        }
    }
    for (const auto& start : edges) {
        std::vector<int> chain;
        std::unordered_map<int, size_t> position;
        int current = start.first;
        while (edges.count(current) != 0) {
            const auto found = position.find(current);
            if (found != position.end()) {
                std::vector<int> cycle(chain.begin() + found->second, chain.end());
                if (cycle.size() < 2) break;
                std::sort(cycle.begin(), cycle.end());
                std::ostringstream out;
                for (size_t i = 0; i < cycle.size(); ++i) {
                    if (i != 0) out << "/";
                    out << "V" << cycle[i];
                }
                return out.str();
            }
            position[current] = chain.size();
            chain.push_back(current);
            current = edges.at(current);
        }
    }
    return "";
}

void DiagnosticLogger::startIncident(const std::string& type,
                                     const std::string& details,
                                     const std::string& snapshot) {
    if (!coordination_enabled_) return;
    if (!incident_active_) {
        incident_active_ = true;
        incident_start_time_ = context_.sim_time;
        last_incident_update_time_ = context_.sim_time;
        std::ostringstream id;
        id << "I" << std::setfill('0') << std::setw(3) << ++incident_id_;
        const std::string incident_id = id.str();
        std::string type_name = type;
        for (char& c : type_name) {
            if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
        }
        const std::string path = (std::filesystem::path(run_dir_) / "incidents" /
                                  (incident_id + "_" + type_name + ".log")).string();
        incident_log_.open(path, std::ios::out | std::ios::trunc);
        if (!incident_log_) {
            ROS_ERROR("[diagnostics] cannot archive incident %s", path.c_str());
            incident_active_ = false;
            return;
        }
        incident_log_ << "[" << readableSimTime(context_.sim_time)
                      << "] [INCIDENT_BEGIN] incident_id="
                      << incident_id << " type=" << type << " trigger=\""
                      << sanitizeValue(details) << "\" history_seconds=300\n";
        for (const auto& item : history_) incident_log_ << item.second << "\n";
        if (!snapshot.empty()) incident_log_ << sanitizeValue(snapshot) << "\n";
        incident_log_.flush();
        summary("INCIDENT incident_id=" + incident_id + " sim_t=" +
                std::to_string(context_.sim_time) + " archive=incidents/" +
                incident_id + "_" + type_name + ".log type=" + type);
    } else if (incident_log_) {
        incident_log_ << "[" << readableSimTime(context_.sim_time)
                      << "] [INCIDENT_TRIGGER] type="
                      << type << " details=\"" << sanitizeValue(details) << "\"\n";
    }
}

void DiagnosticLogger::closeIncident(const std::string& result) {
    if (!incident_active_) return;
    if (incident_log_) {
        incident_log_ << "[" << readableSimTime(context_.sim_time)
                      << "] [INCIDENT_END] result="
                      << result << "\n";
        incident_log_.close();
    }
    incident_active_ = false;
}

void DiagnosticLogger::triggerIncident(const std::string& type,
                                       const std::string& details,
                                       const std::string& snapshot) {
    startIncident(type, details, snapshot);
}

void DiagnosticLogger::printProgress(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const multi_vehicle::RuleEngine& rule_engine) {
    if (context_.sim_time - last_console_progress_time_ >= 10.0 - 1e-9) {
        last_console_progress_time_ = context_.sim_time;
        int tasks = 0;
        int active = 0;
        int stopped = 0;
        double max_wait = 0.0;
        for (const auto& vehicle : vehicles) {
            tasks += vehicle.task_count;
            active += vehicle.mode == multi_vehicle::VehicleMode::ACTIVE ? 1 : 0;
            stopped += vehicle.mode == multi_vehicle::VehicleMode::ACTIVE &&
                       vehicle.action == multi_vehicle::VehicleAction::STOP ? 1 : 0;
            max_wait = std::max(max_wait, vehicle.wait_time);
        }
        const auto& owner = rule_engine.futureA1Commitment();
        ROS_INFO("[SIM] t=%s tasks=%d active=%d stop=%d max_wait=%.1fs A1_owner=%s",
                 readableSimTime(context_.sim_time).c_str(), tasks, active, stopped,
                 max_wait, owner.valid() ? ("V" + std::to_string(owner.owner_id)).c_str() : "none");
    }
    if (context_.sim_time - last_console_table_time_ >= 60.0 - 1e-9) {
        last_console_table_time_ = context_.sim_time;
        std::ostringstream table;
        table << "[SIM] t=" << readableSimTime(context_.sim_time);
        for (const auto& vehicle : vehicles) {
            table << "\nV" << vehicle.id << " " << routeName(vehicle) << " "
                  << multi_vehicle::actionName(vehicle.action) << " wait="
                  << std::fixed << std::setprecision(1) << vehicle.wait_time << "s";
        }
        ROS_INFO("%s", table.str().c_str());
    }
}

void DiagnosticLogger::observeSimulationTick(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const multi_vehicle::RuleEngine& rule_engine,
    unsigned long long tick, double sim_time) {
    context_.tick = tick;
    context_.sim_time = sim_time;
    context_.source = "REAL";
    for (auto& item : module_streams_) rotate(item.second, sim_time, false);
    for (const auto& vehicle : vehicles) {
        VehicleHistoryState& previous = vehicle_history_[vehicle.id];
        const bool progressed = !previous.initialized ||
            std::abs(vehicle.path_s - previous.path_s) > 1e-4 ||
            vehicle.path_gen != previous.path_gen || vehicle.task_count != previous.task_count ||
            vehicle.mission_phase != previous.phase || vehicle.mode != previous.mode;
        if (progressed || vehicle.mode != multi_vehicle::VehicleMode::ACTIVE) {
            previous.last_progress_time = sim_time;
            previous.no_progress_reported = false;
        }
        if (previous.last_decision_plan_id != context_.plan_id) {
            const auto& recovery = rule_engine.recoveryDirective();
            const auto motion = rule_engine.motionOverrideFor(vehicle.id);
            const bool recovery_vehicle = recovery.active() &&
                (vehicle.id == recovery.retreat_vehicle_id ||
                 vehicle.id == recovery.pass_vehicle_id);
            std::ostringstream details;
            details << std::fixed << std::setprecision(3)
                    << "vehicle_id=V" << vehicle.id
                    << " route=" << routeName(vehicle)
                    << " mode=" << modeName(vehicle.mode)
                    << " phase=" << missionPhaseName(vehicle.mission_phase)
                    << " final_action=" << multi_vehicle::actionName(vehicle.action)
                    << " requested_action="
                    << multi_vehicle::actionName(vehicle.requested_action)
                    << " reason=" << (vehicle.reason.empty() ? "clear" : vehicle.reason)
                    << " module="
                    << (recovery_vehicle ? "deadlock" : reasonModule(vehicle.reason))
                    << " blocker=" << blockerName(vehicle.blocker_id)
                    << " task=" << vehicle.task_count
                    << " path_gen=" << vehicle.path_gen
                    << " path_s=" << vehicle.path_s << "/"
                    << (vehicle.track.empty() ? 0.0 : vehicle.track.length())
                    << " speed=" << vehicle.current_speed
                    << " motion=" << recoveryMotionName(motion.motion)
                    << " recovery_phase="
                    << multi_vehicle::recoveryPhaseName(recovery.phase);
            if (recovery_vehicle) {
                details << " recovery_attempt=" << recovery.retreat_attempt
                        << " retreat=V" << recovery.retreat_vehicle_id
                        << " pass=V" << recovery.pass_vehicle_id
                        << " retreat_target_s=" << recovery.retreat_target_s;
            }
            RotatingStream& stream = vehicleStream(vehicle.id, false);
            const std::string record = formatRecord(
                "DECISION_COMMIT", details.str(), context_, nextEventId());
            writeRotating(stream, context_.sim_time, record);
            appendHistory(record, context_.sim_time);
            previous.last_decision_plan_id = context_.plan_id;
        }
        writeVehicleSnapshot(vehicle, previous, false);
        writeTrajectory(vehicle);
        const double no_progress = sim_time - previous.last_progress_time;
        if (vehicle.mode == multi_vehicle::VehicleMode::ACTIVE && no_progress >= 60.0 &&
            !previous.no_progress_reported) {
            previous.no_progress_reported = true;
            std::ostringstream details;
            details << "vehicle_id=V" << vehicle.id << " duration=" << no_progress
                    << " action=" << multi_vehicle::actionName(vehicle.action)
                    << " blocker=" << blockerName(vehicle.blocker_id)
                    << " reason=" << vehicle.reason;
            ROS_WARN("[SIM] no progress: %s", details.str().c_str());
            startIncident("NO_PROGRESS", details.str(),
                          formatStressSnapshot(vehicles, rule_engine, tick, sim_time, true));
        }
        previous.initialized = true;
        previous.mode = vehicle.mode;
        previous.phase = vehicle.mission_phase;
        previous.action = vehicle.action;
        previous.blocker_id = vehicle.blocker_id;
        previous.path_gen = vehicle.path_gen;
        previous.task_count = vehicle.task_count;
        previous.current_slot = vehicle.current_slot;
        previous.target_slot = vehicle.target_slot;
        previous.path_s = vehicle.path_s;
        previous.reason = vehicle.reason;
    }

    const std::string cycle = waitCycleSignature(vehicles);
    if (cycle != wait_cycle_signature_) {
        if (!wait_cycle_signature_.empty()) {
            writeModule(StreamKind::DEADLOCK, "CANDIDATE_CLEAR",
                        "pair=" + wait_cycle_signature_ + " duration=" +
                        std::to_string(sim_time - wait_cycle_since_), context_);
        }
        wait_cycle_signature_ = cycle;
        wait_cycle_since_ = sim_time;
        if (!cycle.empty()) {
            writeModule(StreamKind::DEADLOCK, "CANDIDATE", "pair=" + cycle, context_);
        }
    }

    if (sim_time - last_progress_log_time_ >= 60.0 - 1e-9) {
        last_progress_log_time_ = sim_time;
        int tasks = 0;
        int stop_count = 0;
        double max_wait = 0.0;
        for (const auto& vehicle : vehicles) {
            tasks += vehicle.task_count;
            stop_count += vehicle.action == multi_vehicle::VehicleAction::STOP ? 1 : 0;
            max_wait = std::max(max_wait, vehicle.wait_time);
        }
        summary("PROGRESS tasks=" + std::to_string(tasks) + " stop_count=" +
                std::to_string(stop_count) + " max_wait=" + std::to_string(max_wait));
    }
    printProgress(vehicles, rule_engine);

    bool stopped = false;
    bool no_progress_active = false;
    for (const auto& vehicle : vehicles) {
        stopped = stopped || (vehicle.mode == multi_vehicle::VehicleMode::ACTIVE &&
                              vehicle.action == multi_vehicle::VehicleAction::STOP);
        const auto found = vehicle_history_.find(vehicle.id);
        no_progress_active = no_progress_active ||
            (found != vehicle_history_.end() && found->second.no_progress_reported);
    }
    if (incident_active_ && sim_time - last_incident_update_time_ >= 10.0) {
        last_incident_update_time_ = sim_time;
        if (incident_log_) {
            incident_log_ << "[" << readableSimTime(sim_time)
                          << "] [INCIDENT_STATUS] stopped="
                          << (stopped ? 1 : 0) << " wait_cycle="
                          << (cycle.empty() ? "none" : cycle) << " recovery="
                          << multi_vehicle::recoveryPhaseName(rule_engine.recoveryDirective().phase)
                          << "\n";
            incident_log_.flush();
        }
    }
    if (incident_active_ && sim_time - incident_start_time_ >= 20.0 && !stopped &&
        cycle.empty() && !no_progress_active && !rule_engine.recoveryDirective().active()) {
        closeIncident("RECOVERED");
    }
    flushWindow(sim_time, false);
}

std::ofstream& DiagnosticLogger::projectionStream(int vehicle_id) {
    auto inserted = projection_logs_.try_emplace(vehicle_id);
    std::ofstream& stream = inserted.first->second;
    if (!inserted.second) return stream;
    const std::string path = (std::filesystem::path(run_dir_) /
        ("real_projection_V" + std::to_string(vehicle_id) + ".csv")).string();
    if (!ensureParentDirectory(path)) return stream;
    stream.open(path, std::ios::out | std::ios::trunc);
    if (stream) {
        stream << "wall_time,sim_time,tick,vehicle_id,real_x,real_y,real_yaw,path_gen,mode,mission_phase,leg_target,previous_path_s,projected_path_s,delta_s,projected_x,projected_y,projected_yaw,projection_distance,yaw_error_to_projected_path,wp_type,current_speed,action,real_plan_id,frame_id,search_s_min,search_s_max,raw_single_step_speed,window_speed,speed_window_duration,speed_window_samples,best_xy_distance,selected_heading_error,selected_continuity_error,projection_candidate_count\n";
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
        << "\nring_seconds=300\nprogress_timeout_s=" << progress_timeout
        << "\n\n===== PRE-FAILURE RING =====\n";
    for (const auto& frame : ring) out << frame << "\n---\n";
    out << "===== FAILURE GEOMETRY =====\n" << failure_snapshot << "\n";
    return true;
}

void DiagnosticLogger::truncateEventLog() {
    flushWindow(context_.sim_time, true);
    if (summary_log_) summary_log_.flush();
}

void DiagnosticLogger::finalize(double final_sim_time) {
    if (!initialized_ || finalized_) return;
    context_.sim_time = std::max(context_.sim_time, final_sim_time);
    flushWindow(context_.sim_time, true);
    if (summary_log_) summary_log_.flush();
    for (auto& item : projection_logs_) {
        if (item.second) item.second.flush();
    }
    finalized_ = true;
}

}  // namespace diagnostics
}  // namespace forklift_planner
