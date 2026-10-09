#include "forklift_planner/diagnostics/diagnostic_logger.h"

#include <ros/ros.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include "forklift_planner/diagnostics/diagnostic_snapshot.h"

namespace forklift_planner {
namespace diagnostics {

DiagnosticLogger::DiagnosticLogger(std::string log_dir, std::string coordination_file,
                                   bool enabled)
    : log_dir_(std::move(log_dir)), coordination_file_(std::move(coordination_file)),
      rolling_file_(coordination_file_), event_file_(coordination_file_),
      summary_file_(coordination_file_), coordination_enabled_(enabled) {}

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
    if (!coordination_enabled_) return true;
    if (!ensureParentDirectory(coordination_file_)) {
        return false;
    }
    diagnosis_log_.open(coordination_file_, std::ios::out | std::ios::trunc);
    if (!diagnosis_log_) {
        ROS_WARN("[diagnostics] failed to open unified diagnosis log: %s",
                 coordination_file_.c_str());
        return false;
    }
    diagnosis_log_ << "schema=simulation_diagnosis_v1\n"
                   << "mode=MULTI_VEHICLE seed=" << seed
                   << " vehicle_count=" << vehicle_count
                   << " one_shot=" << (one_shot ? 1 : 0)
                   << " use_a1_cycle=" << (use_a1_cycle ? 1 : 0) << "\n";
    diagnosis_log_.flush();
    ROS_WARN("[diagnostics] unified diagnosis log: %s",
             coordination_file_.c_str());
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
        << std::fixed << std::setprecision(3) << context.sim_time << "] [scope="
        << (context.source == "ROLLOUT" ? "PREDICTED" : "EXECUTED")
        << "] " << line;
    return out.str();
}

bool DiagnosticLogger::isIncidentLine(const std::string& line) {
    return line.find("HARD_GUARD") != std::string::npos ||
           line.find("PATH_FAILURE") != std::string::npos ||
           line.find("ROLLING_PLAN_FAILURE") != std::string::npos ||
           line.find("A1_ADMISSION_INVARIANT_VIOLATION") != std::string::npos ||
           line.find("event=UNRESOLVED") != std::string::npos ||
           line.find("[DEADLOCK] event=ABORT") != std::string::npos ||
           line.find("[STRESS_FAILURE]") != std::string::npos;
}

bool DiagnosticLogger::persistDuringNormalRun(const std::string& line,
                                              const LogContext& context) {
    if (context.source != "REAL") return false;
    if (line.rfind("[multi_patrol][state]", 0) == 0) return false;
    if (line.find(" event=HOLD") != std::string::npos) return false;
    return line.find("[PATH_FAILURE]") != std::string::npos ||
           line.find("[PREPARE_DROPOFF]") != std::string::npos ||
           line.find("[ACTIVATE_DROPOFF]") != std::string::npos ||
           line.find("[FUTURE_A1]") != std::string::npos ||
           line.find("[A1_SERVICE]") != std::string::npos ||
           line.find("[A1_HANDOFF]") != std::string::npos ||
           line.find("[DEPARTURE_CLUSTER]") != std::string::npos ||
           line.find("[A1_STATE_COMMIT]") != std::string::npos ||
           line.find("[RESERVATION_COMMIT]") != std::string::npos ||
           line.find("[DEADLOCK]") != std::string::npos ||
           line.find("[HARD_GUARD]") != std::string::npos ||
           line.find("[ROLLING_PLAN_FAILURE]") != std::string::npos;
}

bool DiagnosticLogger::keepRolloutEvidence(const std::string& line,
                                           const LogContext& context) const {
    if (context.source != "ROLLOUT") return true;
    if (context.frame_id < 0 || context.frame_id >= rollout_commit_frames_) {
        return false;
    }
    return line.find("[DYN-TTC]") != std::string::npos ||
           line.find("[DYN-SPEED]") != std::string::npos ||
           line.find("[DYN-PHYSICAL]") != std::string::npos ||
           line.find("[BRIDGE-TTC]") != std::string::npos ||
           line.find("[FORWARD_CLEARANCE]") != std::string::npos ||
           line.find("[TARGET_SLOT_OCCUPANCY]") != std::string::npos ||
           line.find("[ACTION_HOLD]") != std::string::npos ||
           line.find("[ACTION_REQUEST]") != std::string::npos ||
           line.find("[A1_") != std::string::npos ||
           line.find("[A1-") != std::string::npos ||
           line.find("[FUTURE_A1]") != std::string::npos ||
           line.find("[SLOT_DEPARTURE]") != std::string::npos ||
           line.find("[DEPARTURE_CLUSTER]") != std::string::npos ||
           line.find("[CONFLICT_RESERVATION]") != std::string::npos ||
           line.find("[DEADLOCK]") != std::string::npos ||
           line.find("[PATH_FAILURE]") != std::string::npos;
}

void DiagnosticLogger::appendHistory(const std::string& line, double sim_time) {
    if (suppressed_) return;
    history_.emplace_back(sim_time, line);
    while (!history_.empty() && sim_time - history_.front().first > 120.0) {
        history_.pop_front();
    }
    while (history_.size() > 50000) history_.pop_front();
    if (incident_active_) writeDiagnosis(line);
}

void DiagnosticLogger::writeDiagnosis(const std::string& line, bool flush) {
    if (suppressed_ || !coordination_enabled_ || !diagnosis_log_) return;
    diagnosis_log_ << line << "\n";
    if (flush) diagnosis_log_.flush();
}

void DiagnosticLogger::startIncident(const std::string& type,
                                     const std::string& details,
                                     const std::string& snapshot) {
    if (!coordination_enabled_) return;
    if (!incident_active_) {
        incident_active_ = true;
        incident_snapshot_written_ = false;
        incident_start_time_ = context_.sim_time;
        last_incident_update_time_ = context_.sim_time;
        ++incident_id_;
        std::ostringstream begin;
        begin << "===== INCIDENT_BEGIN id=F" << incident_id_
              << " type=" << type << " tick=" << context_.tick
              << " sim_t=" << std::fixed << std::setprecision(3)
              << context_.sim_time << " details=\"" << details << "\" =====";
        writeDiagnosis(begin.str());
        writeDiagnosis("===== PRE_INCIDENT_HISTORY seconds=120 =====");
        for (const auto& item : history_) writeDiagnosis(item.second);
        writeDiagnosis("===== INCIDENT_STATE =====");
    } else {
        std::ostringstream update;
        update << "[INCIDENT_TRIGGER] id=F" << incident_id_ << " type=" << type
               << " tick=" << context_.tick << " sim_t=" << context_.sim_time
               << " details=\"" << details << "\"";
        writeDiagnosis(update.str());
    }
    if (!snapshot.empty()) {
        writeDiagnosis(snapshot);
        incident_snapshot_written_ = true;
    }
    diagnosis_log_.flush();
}

void DiagnosticLogger::triggerIncident(const std::string& type,
                                       const std::string& details,
                                       const std::string& snapshot) {
    startIncident(type, details, snapshot);
}

void DiagnosticLogger::coordination(const std::string& line) {
    coordination(line, context_);
}

void DiagnosticLogger::coordination(const std::string& line, const LogContext& context) {
    const std::string contextual = contextualize(line, context);
    const bool incident_line = isIncidentLine(line) && context.source == "REAL";
    if (keepRolloutEvidence(line, context)) {
        appendHistory(contextual, context.sim_time);
        if (context.source == "ROLLOUT") {
            for (size_t pos = 0; pos + 1 < line.size(); ++pos) {
                if (line[pos] != 'V' ||
                    !std::isdigit(static_cast<unsigned char>(line[pos + 1]))) {
                    continue;
                }
                size_t end = pos + 2;
                while (end < line.size() &&
                       std::isdigit(static_cast<unsigned char>(line[end]))) {
                    ++end;
                }
                const int vehicle_id = std::stoi(line.substr(pos + 1, end - pos - 1));
                rollout_evidence_vehicles_.insert(
                    {context.plan_id, context.frame_id, vehicle_id});
                pos = end - 1;
            }
            while (rollout_evidence_vehicles_.size() > 12000) {
                rollout_evidence_vehicles_.erase(
                    rollout_evidence_vehicles_.begin());
            }
        }
    }
    if (persistDuringNormalRun(line, context) && !incident_active_ &&
        !incident_line) {
        writeDiagnosis(contextual);
    }
    if (incident_line) {
        const LogContext previous = context_;
        context_ = context;
        startIncident("RULE_OR_PLANNING_FAILURE", line, "");
        context_ = previous;
    }
}

void DiagnosticLogger::rolling(const std::string& line) {
    const std::string contextual = contextualize(line);
    appendHistory(contextual, context_.sim_time);
}

void DiagnosticLogger::event(const std::string& line) {
    const std::string contextual = contextualize(line);
    appendHistory(contextual, context_.sim_time);
    if (context_.source == "REAL") {
        if (isIncidentLine(line)) {
            startIncident("SAFETY_OR_SYSTEM_EVENT", line, "");
        } else if (!incident_active_) {
            writeDiagnosis(contextual, true);
        }
    }
}

void DiagnosticLogger::eventHistory(const std::string& header,
                                    const std::deque<std::string>& history) {
    event("========== " + header + " ==========");
    for (const auto& snapshot : history) event(snapshot);
}

void DiagnosticLogger::summary(const std::string& line) {
    writeDiagnosis("[RUN_SUMMARY] " + line);
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

void DiagnosticLogger::vehicleState(const multi_vehicle::VehicleAgent& vehicle) {
    if (suppressed_) return;
    appendHistory(contextualize("[STATE_CHANGE] " + formatVehicleCompact(vehicle)),
                  context_.sim_time);
}

std::string DiagnosticLogger::waitCycleSignature(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles) const {
    std::unordered_map<int, int> edges;
    for (const auto& vehicle : vehicles) {
        if (vehicle.mode == multi_vehicle::VehicleMode::ACTIVE &&
            vehicle.action == multi_vehicle::VehicleAction::STOP &&
            vehicle.blocker_id >= 0) {
            edges[vehicle.id] = vehicle.blocker_id;
        }
    }
    std::set<int> globally_seen;
    for (const auto& start : edges) {
        std::vector<int> chain;
        std::unordered_map<int, size_t> position;
        int current = start.first;
        while (edges.count(current) != 0 && globally_seen.count(current) == 0) {
            const auto existing = position.find(current);
            if (existing != position.end()) {
                std::vector<int> cycle(chain.begin() + existing->second, chain.end());
                if (cycle.size() < 2) break;
                const auto minimum = std::min_element(cycle.begin(), cycle.end());
                std::rotate(cycle.begin(), minimum, cycle.end());
                std::ostringstream out;
                for (size_t i = 0; i < cycle.size(); ++i) {
                    if (i != 0) out << "->";
                    out << "V" << cycle[i];
                }
                out << "->V" << cycle.front();
                return out.str();
            }
            position[current] = chain.size();
            chain.push_back(current);
            current = edges.at(current);
        }
        globally_seen.insert(chain.begin(), chain.end());
    }
    return "";
}

void DiagnosticLogger::observeSimulationTick(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const multi_vehicle::RuleEngine& rule_engine,
    unsigned long long tick, double sim_time) {
    context_.tick = tick;
    context_.sim_time = sim_time;
    if (last_history_sample_time_ < 0.0 ||
        sim_time - last_history_sample_time_ >= 1.0 - 1e-9) {
        last_history_sample_time_ = sim_time;
        appendHistory(contextualize("[FLEET_SAMPLE] " +
                                    formatFleetSnapshot(vehicles, tick)),
                      sim_time);
    }
    bool no_progress_active = false;
    bool stopped_active = false;

    for (const auto& vehicle : vehicles) {
        VehicleHistoryState& previous = vehicle_history_[vehicle.id];
        if (previous.initialized &&
            (previous.mode != vehicle.mode || previous.phase != vehicle.mission_phase ||
             previous.task_count != vehicle.task_count ||
             previous.path_gen != vehicle.path_gen)) {
            std::ostringstream transition;
            transition << "[TASK_STATE_CHANGE] scope=EXECUTED vehicle=V"
                       << vehicle.id << " mode=" << modeName(previous.mode)
                       << "->" << modeName(vehicle.mode) << " phase="
                       << missionPhaseName(previous.phase) << "->"
                       << missionPhaseName(vehicle.mission_phase) << " task="
                       << previous.task_count << "->" << vehicle.task_count
                       << " path_gen=" << previous.path_gen << "->"
                       << vehicle.path_gen << " slot=" << vehicle.current_slot
                       << "->" << vehicle.target_slot;
            const std::string contextual = contextualize(transition.str());
            appendHistory(contextual, sim_time);
            if (!incident_active_) writeDiagnosis(contextual);
        }
        const bool constrained = vehicle.mode == multi_vehicle::VehicleMode::ACTIVE &&
            (vehicle.action == multi_vehicle::VehicleAction::STOP ||
             vehicle.action == multi_vehicle::VehicleAction::CREEP ||
             vehicle.action == multi_vehicle::VehicleAction::YIELD);
        const bool was_constrained = previous.initialized &&
            previous.mode == multi_vehicle::VehicleMode::ACTIVE &&
            (previous.action == multi_vehicle::VehicleAction::STOP ||
             previous.action == multi_vehicle::VehicleAction::CREEP ||
             previous.action == multi_vehicle::VehicleAction::YIELD);
        const bool signature_changed = !previous.initialized ||
            previous.action != vehicle.action ||
            previous.blocker_id != vehicle.blocker_id ||
            previous.reason != vehicle.reason ||
            previous.path_gen != vehicle.path_gen;

        if (was_constrained && (!constrained || signature_changed)) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[CONSTRAINT_END] scope=EXECUTED constraint_id="
                 << previous.constraint_id << " vehicle=V" << vehicle.id
                 << " action=" << multi_vehicle::actionName(previous.action)
                 << " blocker=V" << previous.blocker_id
                 << " reason=" << previous.reason
                 << " established_at=" << previous.constraint_since
                 << " duration=" << (sim_time - previous.constraint_since)
                 << " end=" << (constrained ? "COVERED" : "RELEASED");
            appendHistory(contextualize(line.str()), sim_time);
        }
        if (constrained && (!was_constrained || signature_changed)) {
            previous.constraint_since = sim_time;
            previous.last_constraint_heartbeat = sim_time;
            previous.last_evidence_plan = context_.plan_id;
            previous.constraint_id = "C" + std::to_string(++constraint_id_);
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[CONSTRAINT_BEGIN] scope=EXECUTED constraint_id="
                 << previous.constraint_id << " vehicle=V" << vehicle.id
                 << " plan=" << context_.plan_id << " frame=" << context_.frame_id
                 << " path_gen=" << vehicle.path_gen
                 << " phase=" << missionPhaseName(vehicle.mission_phase)
                 << " action=" << multi_vehicle::actionName(vehicle.action)
                 << " requested=" << multi_vehicle::actionName(vehicle.requested_action)
                 << " blocker=V" << vehicle.blocker_id
                 << " reason=" << vehicle.reason;
            const bool evidence_available = rollout_evidence_vehicles_.count(
                {context_.plan_id, context_.frame_id, vehicle.id}) != 0;
            line << " evidence=ROLLOUT(plan=" << context_.plan_id
                 << ",frame=" << context_.frame_id << ")"
                 << " evidence_status="
                 << (evidence_available ? "AVAILABLE" : "MISSING");
            if (!evidence_available) {
                line << " missing=rule_specific_ttc_priority_boundary";
            }
            appendHistory(contextualize(line.str()), sim_time);
        } else if (constrained && previous.last_evidence_plan != context_.plan_id) {
            previous.last_evidence_plan = context_.plan_id;
            const bool evidence_available = rollout_evidence_vehicles_.count(
                {context_.plan_id, context_.frame_id, vehicle.id}) != 0;
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[CONSTRAINT_REAFFIRMED] scope=EXECUTED constraint_id="
                 << previous.constraint_id << " vehicle=V" << vehicle.id
                 << " duration=" << (sim_time - previous.constraint_since)
                 << " plan=" << context_.plan_id << " frame=" << context_.frame_id
                 << " action=" << multi_vehicle::actionName(vehicle.action)
                 << " blocker=V" << vehicle.blocker_id
                 << " reason=" << vehicle.reason
                 << " evidence_status="
                 << (evidence_available ? "AVAILABLE" : "MISSING");
            if (!evidence_available) {
                line << " missing=rule_specific_ttc_priority_boundary";
            }
            appendHistory(contextualize(line.str()), sim_time);
        } else if (constrained &&
                   sim_time - previous.last_constraint_heartbeat >= 10.0) {
            previous.last_constraint_heartbeat = sim_time;
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[CONSTRAINT_CONTINUE] scope=EXECUTED constraint_id="
                 << previous.constraint_id << " vehicle=V" << vehicle.id
                 << " duration=" << (sim_time - previous.constraint_since)
                 << " plan=" << context_.plan_id << " frame=" << context_.frame_id
                 << " action=" << multi_vehicle::actionName(vehicle.action)
                 << " blocker=V" << vehicle.blocker_id
                 << " reason=" << vehicle.reason;
            appendHistory(contextualize(line.str()), sim_time);
        }

        const bool progressed = !previous.initialized ||
            std::abs(vehicle.path_s - previous.path_s) > 1e-4 ||
            vehicle.path_gen != previous.path_gen ||
            vehicle.task_count != previous.task_count ||
            vehicle.mission_phase != previous.phase ||
            vehicle.mode != previous.mode;
        if (progressed || vehicle.mode != multi_vehicle::VehicleMode::ACTIVE) {
            previous.last_progress_time = sim_time;
            previous.no_progress_reported = false;
        }
        const double no_progress_duration = sim_time - previous.last_progress_time;
        if (vehicle.mode == multi_vehicle::VehicleMode::ACTIVE &&
            no_progress_duration >= 60.0) {
            no_progress_active = true;
            if (!previous.no_progress_reported) {
                previous.no_progress_reported = true;
                std::ostringstream details;
                details << "vehicle=V" << vehicle.id
                        << " duration=" << no_progress_duration
                        << " action=" << multi_vehicle::actionName(vehicle.action)
                        << " blocker=V" << vehicle.blocker_id
                        << " reason=" << vehicle.reason;
                startIncident("NO_PROGRESS", details.str(),
                              formatStressSnapshot(vehicles, rule_engine, tick,
                                                   sim_time, true));
            }
        }
        stopped_active = stopped_active ||
            (vehicle.mode == multi_vehicle::VehicleMode::ACTIVE &&
             vehicle.action == multi_vehicle::VehicleAction::STOP);

        previous.initialized = true;
        previous.mode = vehicle.mode;
        previous.phase = vehicle.mission_phase;
        previous.action = vehicle.action;
        previous.blocker_id = vehicle.blocker_id;
        previous.path_gen = vehicle.path_gen;
        previous.task_count = vehicle.task_count;
        previous.path_s = vehicle.path_s;
        previous.reason = vehicle.reason;
    }

    const std::string cycle = waitCycleSignature(vehicles);
    if (cycle != wait_cycle_signature_) {
        if (!wait_cycle_signature_.empty()) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[WAIT_CYCLE_END] scope=EXECUTED cycle="
                 << wait_cycle_signature_ << " duration="
                 << (sim_time - wait_cycle_since_)
                 << " end=" << (cycle.empty() ? "RELEASED" : "CHANGED");
            appendHistory(contextualize(line.str()), sim_time);
        }
        wait_cycle_signature_ = cycle;
        wait_cycle_since_ = sim_time;
        if (!cycle.empty()) {
            appendHistory(contextualize("[WAIT_CYCLE_BEGIN] scope=EXECUTED cycle=" + cycle),
                          sim_time);
        }
    }
    const bool cycle_active = !cycle.empty() && sim_time - wait_cycle_since_ >= 8.0;
    if (cycle_active && (!incident_active_ ||
        sim_time - last_incident_update_time_ >= 30.0)) {
        startIncident("WAIT_CYCLE", "cycle=" + cycle,
                      formatStressSnapshot(vehicles, rule_engine, tick,
                                           sim_time, true));
        last_incident_update_time_ = sim_time;
    }

    if (!incident_snapshot_written_ && incident_active_) {
        writeDiagnosis(formatStressSnapshot(vehicles, rule_engine, tick,
                                            sim_time, true));
        incident_snapshot_written_ = true;
    }

    if (sim_time - last_progress_log_time_ >= 60.0) {
        last_progress_log_time_ = sim_time;
        std::ostringstream progress;
        progress << "[PROGRESS] tick=" << tick << " sim_t=" << std::fixed
                 << std::setprecision(1) << sim_time;
        for (const auto& vehicle : vehicles) {
            progress << " V" << vehicle.id << "=" << modeName(vehicle.mode)
                     << "/" << missionPhaseName(vehicle.mission_phase)
                     << "/task" << vehicle.task_count;
        }
        writeDiagnosis(progress.str());
    }

    const auto& recovery = rule_engine.recoveryDirective();
    const bool recovery_active = recovery.active();
    if (incident_active_ && sim_time - last_incident_update_time_ >= 10.0) {
        last_incident_update_time_ = sim_time;
        std::ostringstream update;
        update << "[INCIDENT_STATUS] id=F" << incident_id_
               << " tick=" << tick << " sim_t=" << sim_time
               << " stopped=" << (stopped_active ? 1 : 0)
               << " wait_cycle=" << (cycle.empty() ? "none" : cycle)
               << " recovery_phase="
               << multi_vehicle::recoveryPhaseName(recovery.phase)
               << " recovery_reason=" << recovery.reason;
        writeDiagnosis(update.str(), true);
    }
    if (incident_active_ && sim_time - incident_start_time_ >= 20.0 &&
        !stopped_active && !cycle_active && !no_progress_active &&
        !recovery_active) {
        std::ostringstream end;
        end << "===== INCIDENT_END id=F" << incident_id_
            << " tick=" << tick << " sim_t=" << sim_time
            << " result=RECOVERED =====";
        writeDiagnosis(end.str(), true);
        incident_active_ = false;
        incident_snapshot_written_ = false;
    }
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
    if (!diagnosis_log_) return;
    diagnosis_log_.flush();
}

}  // namespace diagnostics
}  // namespace forklift_planner
