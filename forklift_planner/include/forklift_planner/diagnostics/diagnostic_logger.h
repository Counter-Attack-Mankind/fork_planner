#pragma once

#include <cstdint>
#include <deque>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "forklift_planner/multi_vehicle/real_state_estimation.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
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
    static bool ensureParentDirectory(const std::string& path);
    static bool isRollingLine(const std::string& line, const LogContext& context);
    std::ofstream& vehicleStream(int vehicle_id);
    std::ofstream& projectionStream(int vehicle_id);
    void write(std::ofstream& stream, const std::string& line);

    std::string log_dir_;
    std::string coordination_file_;
    std::string rolling_file_;
    std::string event_file_;
    std::string summary_file_;
    bool coordination_enabled_ = true;
    bool suppressed_ = false;
    LogContext context_;
    std::ofstream coordination_log_;
    std::ofstream rolling_log_;
    std::ofstream event_log_;
    std::ofstream summary_log_;
    std::unordered_map<int, std::ofstream> vehicle_logs_;
    std::unordered_map<int, std::ofstream> projection_logs_;
};

}  // namespace diagnostics
}  // namespace forklift_planner
