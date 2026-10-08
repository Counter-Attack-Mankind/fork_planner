#pragma once

#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace diagnostics {

const char* modeName(multi_vehicle::VehicleMode mode);
const char* missionPhaseName(multi_vehicle::MissionPhase phase);
const char* legTargetName(multi_vehicle::LegTargetKind target);
std::string readableSimTime(double seconds);
std::string formatVehicleCompact(const multi_vehicle::VehicleAgent& vehicle);
std::string formatFleetSnapshot(const std::vector<multi_vehicle::VehicleAgent>& vehicles,
                                unsigned long long tick);
std::string formatStressSnapshot(
    const std::vector<multi_vehicle::VehicleAgent>& vehicles,
    const multi_vehicle::RuleEngine& rule_engine, unsigned long long tick,
    double sim_time, bool include_geometry);

}  // namespace diagnostics
}  // namespace forklift_planner
