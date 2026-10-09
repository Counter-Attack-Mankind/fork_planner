#pragma once

#include <cstdint>
#include <deque>
#include <fstream>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/real_state_estimation.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {
class RuleEngine;
}
namespace diagnostics {

struct LogContext {
    std::string source = "REAL";
    uint64_t plan_id = 0;
    int frame_id = -1;
    int rollout_step = -1;
    unsigned long long tick = 0;
    double sim_time = 0.0;
};

class DiagnosticLogger {
public:
    DiagnosticLogger(std::string log_dir, std::string coordination_file, bool enabled);

    bool initialize(int vehicle_count, int seed, bool one_shot, bool use_a1_cycle);
    void setRolloutCommitFrames(int frames) { rollout_commit_frames_ = frames; }
    void setContext(const LogContext& context) { context_ = context; }
    const LogContext& context() const { return context_; }
    void setSuppressed(bool suppressed) { suppressed_ = suppressed; }

    std::string contextualize(const std::string& line) const;
    std::string contextualize(const std::string& line, const LogContext& context) const;
    void coordination(const std::string& line);
    void coordination(const std::string& line, const LogContext& context);
    void rolling(const std::string& line);
    void event(const std::string& line);
    void eventHistory(const std::string& header, const std::deque<std::string>& history);
    void summary(const std::string& line);
    void runSummary(const std::string& status, const std::string& failure_type,
                    unsigned long long tick, double sim_time,
                    unsigned long long hard_guard_events,
                    unsigned long long wedge_episodes,
                    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
                    const std::vector<double>& max_wait);
    void vehicleState(const multi_vehicle::VehicleAgent& vehicle);
    void observeSimulationTick(
        const std::vector<multi_vehicle::VehicleAgent>& vehicles,
        const multi_vehicle::RuleEngine& rule_engine,
        unsigned long long tick, double sim_time);
    void triggerIncident(const std::string& type, const std::string& details,
                         const std::string& snapshot = std::string());
    void realProjection(const multi_vehicle::VehicleAgent& vehicle, double real_x,
                        double real_y, double real_yaw, double previous_path_s,
                        double projected_path_s, double search_s_min,
                        double search_s_max,
                        const multi_vehicle::ArcLengthSpeedResult& speed,
                        const multi_vehicle::RealProjectionResult& projection,
                        double wall_time);

    bool writeStressResult(const std::string& path, const std::string& status,
                           const std::string& failure_type, int seed,
                           unsigned long long tick, double sim_time,
                           unsigned long long hard_guard_events,
                           unsigned long long wedge_episodes,
                           const std::vector<multi_vehicle::VehicleAgent>& vehicles,
                           const std::vector<double>& max_wait) const;
    bool writeStressFailure(const std::string& path, const std::string& failure_type,
                            int seed, double progress_timeout,
                            const std::deque<std::string>& ring,
                            const std::string& failure_snapshot) const;
    void truncateEventLog();

    const std::string& coordinationPath() const { return coordination_file_; }
    const std::string& rollingPath() const { return rolling_file_; }
    const std::string& eventPath() const { return event_file_; }
    const std::string& summaryPath() const { return summary_file_; }

private:
    struct VehicleHistoryState {
        bool initialized = false;
        multi_vehicle::VehicleMode mode = multi_vehicle::VehicleMode::NEED_TASK;
        multi_vehicle::MissionPhase phase = multi_vehicle::MissionPhase::DIRECT_TO_B;
        multi_vehicle::VehicleAction action = multi_vehicle::VehicleAction::STOP;
        int blocker_id = -1;
        int path_gen = -1;
        int task_count = -1;
        double path_s = 0.0;
        double last_progress_time = 0.0;
        double constraint_since = 0.0;
        double last_constraint_heartbeat = 0.0;
        uint64_t last_evidence_plan = 0;
        std::string reason;
        std::string constraint_id;
        bool no_progress_reported = false;
    };

    static bool ensureParentDirectory(const std::string& path);
    static bool isIncidentLine(const std::string& line);
    static bool persistDuringNormalRun(const std::string& line,
                                       const LogContext& context);
    bool keepRolloutEvidence(const std::string& line,
                             const LogContext& context) const;
    void appendHistory(const std::string& line, double sim_time);
    void writeDiagnosis(const std::string& line, bool flush = false);
    void startIncident(const std::string& type, const std::string& details,
                       const std::string& snapshot);
    std::string waitCycleSignature(
        const std::vector<multi_vehicle::VehicleAgent>& vehicles) const;
    std::ofstream& projectionStream(int vehicle_id);

    std::string log_dir_;
    std::string coordination_file_;
    std::string rolling_file_;
    std::string event_file_;
    std::string summary_file_;
    bool coordination_enabled_ = true;
    bool suppressed_ = false;
    int rollout_commit_frames_ = 1;
    LogContext context_;
    std::ofstream diagnosis_log_;
    std::unordered_map<int, std::ofstream> projection_logs_;
    std::deque<std::pair<double, std::string>> history_;
    std::set<std::tuple<uint64_t, int, int>> rollout_evidence_vehicles_;
    std::unordered_map<int, VehicleHistoryState> vehicle_history_;
    bool incident_active_ = false;
    bool incident_snapshot_written_ = false;
    unsigned long long incident_id_ = 0;
    unsigned long long constraint_id_ = 0;
    double incident_start_time_ = 0.0;
    double last_incident_update_time_ = 0.0;
    double last_progress_log_time_ = -60.0;
    double last_history_sample_time_ = -1.0;
    std::string wait_cycle_signature_;
    double wait_cycle_since_ = 0.0;
};

}  // namespace diagnostics
}  // namespace forklift_planner
