#pragma once

#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/real_state_estimation.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle { class RuleEngine; }
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
    DiagnosticLogger(std::string log_dir, bool enabled);
    ~DiagnosticLogger();

    bool initialize(int vehicle_count, int seed, bool one_shot, bool use_a1_cycle,
                    bool real_mode = false);
    void registerVehicles(const std::vector<multi_vehicle::VehicleAgent>& vehicles);
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
                    unsigned long long hard_guard_events, unsigned long long wedge_episodes,
                    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
                    const std::vector<double>& max_wait);
    void vehicleState(const multi_vehicle::VehicleAgent& vehicle);
    void observeSimulationTick(const std::vector<multi_vehicle::VehicleAgent>& vehicles,
                               const multi_vehicle::RuleEngine& rule_engine,
                               unsigned long long tick, double sim_time);
    void triggerIncident(const std::string& type, const std::string& details,
                         const std::string& snapshot = std::string());
    void realProjection(const multi_vehicle::VehicleAgent& vehicle, double real_x,
                        double real_y, double real_yaw, double previous_path_s,
                        double projected_path_s, double search_s_min, double search_s_max,
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
    void finalize(double final_sim_time);

    const std::string& runDirectory() const { return run_dir_; }
    const std::string& runId() const { return run_id_; }
    const std::string& coordinationPath() const { return coordination_file_; }
    const std::string& rollingPath() const { return coordination_file_; }
    const std::string& eventPath() const { return safety_file_; }
    const std::string& summaryPath() const { return summary_file_; }

private:
    enum class StreamKind { A1, COORDINATION, DEADLOCK, SAFETY, VEHICLE, TRAJECTORY };

    struct VehicleHistoryState {
        bool initialized = false;
        multi_vehicle::VehicleMode mode = multi_vehicle::VehicleMode::NEED_TASK;
        multi_vehicle::MissionPhase phase = multi_vehicle::MissionPhase::DIRECT_TO_B;
        multi_vehicle::VehicleAction action = multi_vehicle::VehicleAction::STOP;
        int blocker_id = -1;
        int path_gen = -1;
        int task_count = -1;
        int current_slot = -1;
        int target_slot = -1;
        double path_s = 0.0;
        double last_progress_time = 0.0;
        double constraint_since = 0.0;
        double last_snapshot_time = -1.0;
        unsigned long long last_trajectory_tick = static_cast<unsigned long long>(-1);
        uint64_t last_decision_plan_id = static_cast<uint64_t>(-1);
        std::string reason;
        std::string constraint_id;
        bool no_progress_reported = false;
    };

    struct TimedRecord {
        double sim_time = 0.0;
        std::string text;
    };

    struct RotatingStream {
        std::string logical_name;
        std::string path;
        std::deque<TimedRecord> records;
        std::string last_record;
        std::string baseline_state;
        double baseline_state_time = 0.0;
        bool csv = false;
        bool vehicle = false;
        bool dirty = false;
    };

    struct RolloutEvidence {
        std::string line;
        LogContext context;
    };

    static bool ensureParentDirectory(const std::string& path);
    static bool isIncidentLine(const std::string& line);
    static std::string eventName(const std::string& line);
    static std::string sanitizeValue(const std::string& value);
    static std::string routeName(const multi_vehicle::VehicleAgent& vehicle);
    StreamKind classify(const std::string& line) const;
    bool keepRolloutEvidence(const std::string& line, const LogContext& context) const;
    bool suppressRepeatedDiagnostic(const std::string& line, const LogContext& context) const;
    std::string makeRunDirectory();
    std::string formatRecord(const std::string& event, const std::string& details,
                             const LogContext& context, const std::string& event_id) const;
    std::string nextEventId();
    RotatingStream& moduleStream(StreamKind kind);
    RotatingStream& vehicleStream(int vehicle_id, bool trajectory);
    void rotate(RotatingStream& stream, double sim_time, bool csv);
    void writeRotating(RotatingStream& stream, double sim_time,
                       const std::string& record, bool csv = false,
                       bool deduplicate = false);
    void writeModule(StreamKind kind, const std::string& event,
                     const std::string& details, const LogContext& context);
    void appendHistory(const std::string& record, double sim_time);
    void promoteRolloutEvidence(uint64_t plan_id, const LogContext& context);
    void advanceWindow(double endpoint);
    void flushWindow(double endpoint, bool force);
    void rewriteStream(RotatingStream& stream, double endpoint);
    void startIncident(const std::string& type, const std::string& details,
                       const std::string& snapshot);
    void closeIncident(const std::string& result);
    std::string waitCycleSignature(const std::vector<multi_vehicle::VehicleAgent>& vehicles) const;
    std::ofstream& projectionStream(int vehicle_id);
    void writeVehicleSnapshot(const multi_vehicle::VehicleAgent& vehicle,
                              VehicleHistoryState& previous, bool force);
    void writeTrajectory(const multi_vehicle::VehicleAgent& vehicle);
    void printProgress(const std::vector<multi_vehicle::VehicleAgent>& vehicles,
                       const multi_vehicle::RuleEngine& rule_engine);

    std::string log_dir_;
    std::string run_id_;
    std::string run_dir_;
    std::string coordination_file_;
    std::string safety_file_;
    std::string summary_file_;
    bool coordination_enabled_ = true;
    bool real_mode_ = false;
    bool suppressed_ = false;
    bool initialized_ = false;
    bool run_summary_written_ = false;
    LogContext context_;
    std::ofstream summary_log_;
    std::ofstream incident_log_;
    std::unordered_map<int, std::ofstream> projection_logs_;
    std::map<StreamKind, RotatingStream> module_streams_;
    std::unordered_map<int, RotatingStream> vehicle_streams_;
    std::unordered_map<int, RotatingStream> trajectory_streams_;
    std::deque<std::pair<double, std::string>> history_;
    std::map<uint64_t, std::vector<RolloutEvidence>> rollout_evidence_;
    std::set<std::string> current_tick_module_records_;
    unsigned long long module_dedup_tick_ = static_cast<unsigned long long>(-1);
    std::unordered_map<int, VehicleHistoryState> vehicle_history_;
    bool incident_active_ = false;
    unsigned long long event_id_ = 0;
    unsigned long long incident_id_ = 0;
    unsigned long long constraint_id_ = 0;
    double incident_start_time_ = 0.0;
    double last_incident_update_time_ = 0.0;
    double last_progress_log_time_ = -60.0;
    double last_console_progress_time_ = -10.0;
    double last_console_table_time_ = -60.0;
    double last_flush_time_ = -1.0;
    double final_sim_time_ = 0.0;
    bool finalized_ = false;
    std::string wait_cycle_signature_;
    double wait_cycle_since_ = 0.0;
};

}  // namespace diagnostics
}  // namespace forklift_planner
