#include "forklift_nlp_recovery/replay_data.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace forklift_nlp_recovery {
namespace {

constexpr double kTimeTolerance = 1e-8;

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> values;
    std::stringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) values.push_back(value);
    return values;
}

double requireDouble(const YAML::Node& node, const char* field) {
    if (!node[field]) throw std::runtime_error(std::string("missing field '") + field + "'");
    const double value = node[field].as<double>();
    if (!std::isfinite(value)) throw std::runtime_error(std::string("non-finite field '") + field + "'");
    return value;
}

ReplayPose readPose(const YAML::Node& node) {
    return {requireDouble(node, "x"), requireDouble(node, "y"), requireDouble(node, "yaw")};
}

double csvDouble(const std::vector<std::string>& values,
                 const std::unordered_map<std::string, std::size_t>& columns,
                 const std::string& name) {
    const auto found = columns.find(name);
    if (found == columns.end() || found->second >= values.size()) throw std::runtime_error("missing CSV column '" + name + "'");
    std::size_t used = 0;
    const double value = std::stod(values[found->second], &used);
    if (used != values[found->second].size() || !std::isfinite(value)) throw std::runtime_error("invalid CSV value in '" + name + "'");
    return value;
}

int csvInt(const std::vector<std::string>& values,
           const std::unordered_map<std::string, std::size_t>& columns,
           const std::string& name) {
    const double value = csvDouble(values, columns, name);
    if (std::floor(value) != value || value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
        throw std::runtime_error("invalid integer CSV value in '" + name + "'");
    }
    return static_cast<int>(value);
}

struct NumericState {
    double x;
    double y;
    double yaw;
    double speed;
    double steer;
};

NumericState derivative(const NumericState& state, double acceleration, double steer_rate,
                        double wheel_base) {
    return {state.speed * std::cos(state.yaw), state.speed * std::sin(state.yaw),
            state.speed * std::tan(state.steer) / wheel_base, acceleration, steer_rate};
}

NumericState addScaled(const NumericState& state, const NumericState& delta, double scale) {
    return {state.x + scale * delta.x, state.y + scale * delta.y,
            state.yaw + scale * delta.yaw, state.speed + scale * delta.speed,
            state.steer + scale * delta.steer};
}

double discClearance(const ReplayPose& first, double first_radius,
                     const ReplayPose& second, double second_radius) {
    return std::hypot(first.x - second.x, first.y - second.y) - first_radius - second_radius;
}

double unionAabbViolation(const ReplayVehicle& vehicle, const ReplayPose& center,
                          std::size_t interval, double time) {
    const ReplayNode& current = vehicle.nodes[interval];
    double violation = replayAabbViolation(center, vehicle.corridor.at(current.aabb_index));
    if (interval + 1 < vehicle.nodes.size() && time > current.time + kTimeTolerance) {
        const ReplayNode& next = vehicle.nodes[interval + 1];
        violation = std::min(violation, replayAabbViolation(center, vehicle.corridor.at(next.aabb_index)));
    }
    return violation;
}

std::array<ReplayPose, 4> obbCorners(const ReplayNode& state, const ReplayParameters& parameters) {
    const ReplayPose center = replayBodyCenter(state.rear_axle, parameters.rear_axle_to_center);
    const double c = std::cos(state.rear_axle.yaw);
    const double s = std::sin(state.rear_axle.yaw);
    const double hl = parameters.length * 0.5;
    const double hw = parameters.width * 0.5;
    return {{{center.x + c * hl - s * hw, center.y + s * hl + c * hw, 0.0},
             {center.x + c * hl + s * hw, center.y + s * hl - c * hw, 0.0},
             {center.x - c * hl + s * hw, center.y - s * hl - c * hw, 0.0},
             {center.x - c * hl - s * hw, center.y - s * hl + c * hw, 0.0}}};
}

double obbOverlap(const ReplayNode& first, const ReplayParameters& first_parameters,
                  const ReplayNode& second, const ReplayParameters& second_parameters) {
    const auto a = obbCorners(first, first_parameters);
    const auto b = obbCorners(second, second_parameters);
    const double angles[] = {first.rear_axle.yaw, first.rear_axle.yaw + M_PI_2,
                             second.rear_axle.yaw, second.rear_axle.yaw + M_PI_2};
    double minimum_overlap = std::numeric_limits<double>::infinity();
    for (double angle : angles) {
        const double ax = std::cos(angle);
        const double ay = std::sin(angle);
        double amin = std::numeric_limits<double>::infinity();
        double amax = -amin;
        double bmin = amin;
        double bmax = -amin;
        for (const ReplayPose& point : a) {
            const double projection = point.x * ax + point.y * ay;
            amin = std::min(amin, projection);
            amax = std::max(amax, projection);
        }
        for (const ReplayPose& point : b) {
            const double projection = point.x * ax + point.y * ay;
            bmin = std::min(bmin, projection);
            bmax = std::max(bmax, projection);
        }
        const double overlap = std::min(amax, bmax) - std::max(amin, bmin);
        if (overlap <= 0.0) return 0.0;
        minimum_overlap = std::min(minimum_overlap, overlap);
    }
    return minimum_overlap;
}

using ViolationKey = std::tuple<int, int, int>;

void buildIntervals(const std::vector<ReplayViolation>& violations, double maximum_gap,
                    std::vector<ReplayViolationInterval>& intervals) {
    std::map<ViolationKey, std::vector<const ReplayViolation*>> groups;
    for (const ReplayViolation& violation : violations) {
        groups[{static_cast<int>(violation.type), violation.vehicle_id, violation.other_id}].push_back(&violation);
    }
    for (auto& item : groups) {
        auto& samples = item.second;
        std::sort(samples.begin(), samples.end(), [](const auto* a, const auto* b) { return a->time < b->time; });
        ReplayViolationInterval interval{samples.front()->type, samples.front()->vehicle_id,
                                         samples.front()->other_id, samples.front()->time,
                                         samples.front()->time, samples.front()->magnitude};
        for (std::size_t i = 1; i < samples.size(); ++i) {
            if (samples[i]->time - interval.end_time > maximum_gap + kTimeTolerance) {
                intervals.push_back(interval);
                interval = {samples[i]->type, samples[i]->vehicle_id, samples[i]->other_id,
                            samples[i]->time, samples[i]->time, samples[i]->magnitude};
            } else {
                interval.end_time = samples[i]->time;
                interval.max_violation = std::max(interval.max_violation, samples[i]->magnitude);
            }
        }
        intervals.push_back(interval);
    }
    std::sort(intervals.begin(), intervals.end(), [](const auto& a, const auto& b) {
        return a.start_time < b.start_time;
    });
}

}  // namespace

bool loadReplayCase(const std::string& case_dir, ReplayCase& replay_case, std::string& error) {
    namespace fs = std::filesystem;
    try {
        const fs::path directory(case_dir);
        const fs::path input_path = directory / "optimization_input.yaml";
        const fs::path trajectory_path = directory / "optimized_trajectory.csv";
        const fs::path report_path = directory / "solver_report.yaml";
        for (const fs::path& path : {input_path, trajectory_path, report_path}) {
            if (!fs::is_regular_file(path)) throw std::runtime_error("required file is missing: " + path.string());
        }

        ReplayCase loaded;
        loaded.case_id = directory.filename().string();
        const YAML::Node input = YAML::LoadFile(input_path.string());
        if (!input["participant_ids"] || !input["vehicles"]) throw std::runtime_error("optimization_input.yaml lacks participant_ids or vehicles");
        if (input["frame_id"]) loaded.frame_id = input["frame_id"].as<std::string>();
        loaded.participant_ids = input["participant_ids"].as<std::vector<int>>();
        if (loaded.participant_ids.empty()) throw std::runtime_error("participant_ids is empty");
        std::set<int> unique_ids(loaded.participant_ids.begin(), loaded.participant_ids.end());
        if (unique_ids.size() != loaded.participant_ids.size()) throw std::runtime_error("participant_ids contains duplicates");

        for (const YAML::Node& source : input["vehicles"]) {
            ReplayVehicle vehicle;
            vehicle.id = source["id"].as<int>();
            const YAML::Node parameters = source["parameters"];
            vehicle.parameters.length = requireDouble(parameters, "length");
            vehicle.parameters.width = requireDouble(parameters, "width");
            vehicle.parameters.rear_axle_to_center = requireDouble(parameters, "rear_axle_to_center");
            vehicle.parameters.disc_radius = requireDouble(parameters, "disc_radius");
            vehicle.parameters.wheel_base = requireDouble(parameters, "wheel_base");
            if (vehicle.parameters.length <= 0.0 || vehicle.parameters.width <= 0.0 ||
                vehicle.parameters.disc_radius <= 0.0 || vehicle.parameters.wheel_base <= 0.0) {
                throw std::runtime_error("vehicle " + std::to_string(vehicle.id) + " has invalid geometry");
            }
            if (!source["aabbs"] || !source["aabbs"].IsSequence() || source["aabbs"].size() == 0) {
                throw std::runtime_error("vehicle " + std::to_string(vehicle.id) + " has no AABB corridor");
            }
            for (const YAML::Node& box : source["aabbs"]) {
                ReplayAabb value{requireDouble(box, "min_x"), requireDouble(box, "max_x"),
                                 requireDouble(box, "min_y"), requireDouble(box, "max_y")};
                if (value.min_x > value.max_x || value.min_y > value.max_y) throw std::runtime_error("invalid AABB for vehicle " + std::to_string(vehicle.id));
                vehicle.corridor.push_back(value);
            }
            loaded.vehicles.push_back(vehicle);
        }
        if (loaded.vehicles.size() != loaded.participant_ids.size()) throw std::runtime_error("participant and vehicle counts differ");
        for (std::size_t i = 0; i < loaded.vehicles.size(); ++i) {
            if (loaded.vehicles[i].id != loaded.participant_ids[i]) throw std::runtime_error("vehicle order does not match participant_ids");
        }

        if (input["static_vehicles"] && input["static_vehicles"].IsSequence()) {
            for (const YAML::Node& source : input["static_vehicles"]) {
                ReplayStaticVehicle obstacle;
                obstacle.id = source["id"].as<int>();
                obstacle.rear_axle = readPose(source["rear_axle_pose"]);
                obstacle.disc_center = readPose(source["disc_center"]);
                obstacle.disc_radius = requireDouble(source, "disc_radius");
                if (obstacle.disc_radius <= 0.0) throw std::runtime_error("static vehicle has invalid radius");
                loaded.static_vehicles.push_back(obstacle);
            }
        }

        std::ifstream trajectory(trajectory_path);
        std::string line;
        if (!std::getline(trajectory, line)) throw std::runtime_error("optimized_trajectory.csv is empty");
        const std::vector<std::string> header = splitCsv(line);
        std::unordered_map<std::string, std::size_t> columns;
        for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
        const std::vector<std::string> required = {"vehicle_id", "node_index", "time", "rear_x", "rear_y", "heading", "signed_speed", "front_steer", "acceleration", "steer_rate", "aabb_index"};
        for (const std::string& name : required) if (!columns.count(name)) throw std::runtime_error("optimized_trajectory.csv lacks column '" + name + "'");
        std::map<int, std::vector<ReplayNode>> nodes;
        std::size_t row = 1;
        while (std::getline(trajectory, line)) {
            ++row;
            if (line.empty()) continue;
            const auto values = splitCsv(line);
            ReplayNode node;
            const int id = csvInt(values, columns, "vehicle_id");
            node.node_index = csvInt(values, columns, "node_index");
            node.time = csvDouble(values, columns, "time");
            node.rear_axle = {csvDouble(values, columns, "rear_x"), csvDouble(values, columns, "rear_y"), csvDouble(values, columns, "heading")};
            node.speed = csvDouble(values, columns, "signed_speed");
            node.steer = csvDouble(values, columns, "front_steer");
            node.acceleration = csvDouble(values, columns, "acceleration");
            node.steer_rate = csvDouble(values, columns, "steer_rate");
            node.aabb_index = csvInt(values, columns, "aabb_index");
            nodes[id].push_back(node);
        }
        if (!trajectory.eof()) throw std::runtime_error("failed reading optimized_trajectory.csv");

        for (ReplayVehicle& vehicle : loaded.vehicles) {
            auto found = nodes.find(vehicle.id);
            if (found == nodes.end() || found->second.size() < 2) throw std::runtime_error("trajectory missing or too short for vehicle " + std::to_string(vehicle.id));
            vehicle.nodes = std::move(found->second);
            for (std::size_t i = 0; i < vehicle.nodes.size(); ++i) {
                const ReplayNode& node = vehicle.nodes[i];
                if (node.node_index != static_cast<int>(i)) throw std::runtime_error("non-contiguous node_index for vehicle " + std::to_string(vehicle.id));
                if (node.aabb_index < 0 || static_cast<std::size_t>(node.aabb_index) >= vehicle.corridor.size()) throw std::runtime_error("invalid aabb_index for vehicle " + std::to_string(vehicle.id));
                if (i > 0 && node.time <= vehicle.nodes[i - 1].time) throw std::runtime_error("non-increasing time for vehicle " + std::to_string(vehicle.id));
            }
        }
        if (nodes.size() != loaded.vehicles.size()) throw std::runtime_error("CSV contains an unknown or duplicate vehicle set");
        loaded.node_count = loaded.vehicles.front().nodes.size();
        loaded.total_time = loaded.vehicles.front().nodes.back().time;
        if (std::abs(loaded.vehicles.front().nodes.front().time) > kTimeTolerance || loaded.total_time <= 0.0) throw std::runtime_error("invalid replay time range");
        for (const ReplayVehicle& vehicle : loaded.vehicles) {
            if (vehicle.nodes.size() != loaded.node_count) throw std::runtime_error("participant node counts differ");
            for (std::size_t i = 0; i < loaded.node_count; ++i) {
                if (std::abs(vehicle.nodes[i].time - loaded.vehicles.front().nodes[i].time) > kTimeTolerance) throw std::runtime_error("participant time axes differ");
            }
        }

        const YAML::Node report = YAML::LoadFile(report_path.string());
        if (!report["solver_success"] || !report["validation_passed"] || !report["return_status"] ||
            !report["node_count"] || !report["total_time"] || !report["validation"]) {
            throw std::runtime_error("solver_report.yaml lacks required status or validation fields");
        }
        loaded.report.solver_success = report["solver_success"].as<bool>();
        loaded.report.validation_passed = report["validation_passed"].as<bool>();
        loaded.report.return_status = report["return_status"].as<std::string>();
        loaded.report.node_count = report["node_count"].as<std::size_t>();
        loaded.report.total_time = requireDouble(report, "total_time");
        const YAML::Node validation = report["validation"];
        loaded.report.validation_reason = validation["reason"] ? validation["reason"].as<std::string>() : "";
        loaded.report.minimum_participant_clearance = requireDouble(validation, "minimum_participant_clearance");
        loaded.report.minimum_static_clearance = requireDouble(validation, "minimum_static_clearance");
        loaded.report.participant_collision_samples = validation["participant_collision_samples"].as<std::size_t>();
        loaded.report.static_collision_samples = validation["static_collision_samples"].as<std::size_t>();
        loaded.report.aabb_violation_samples = validation["aabb_violation_samples"].as<std::size_t>();
        if (loaded.report.node_count != loaded.node_count || std::abs(loaded.report.total_time - loaded.total_time) > 1e-6) {
            throw std::runtime_error("solver report time grid does not match optimized CSV");
        }
        replay_case = std::move(loaded);
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

ReplayNode integrateReplayNode(const ReplayNode& source, double dt, double wheel_base) {
    if (dt < 0.0 || wheel_base <= 0.0) throw std::invalid_argument("invalid RK4 interval or wheel base");
    NumericState state{source.rear_axle.x, source.rear_axle.y, source.rear_axle.yaw, source.speed, source.steer};
    const NumericState k1 = derivative(state, source.acceleration, source.steer_rate, wheel_base);
    const NumericState k2 = derivative(addScaled(state, k1, 0.5 * dt), source.acceleration, source.steer_rate, wheel_base);
    const NumericState k3 = derivative(addScaled(state, k2, 0.5 * dt), source.acceleration, source.steer_rate, wheel_base);
    const NumericState k4 = derivative(addScaled(state, k3, dt), source.acceleration, source.steer_rate, wheel_base);
    const double scale = dt / 6.0;
    ReplayNode result = source;
    result.time += dt;
    result.rear_axle.x += scale * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    result.rear_axle.y += scale * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y);
    result.rear_axle.yaw += scale * (k1.yaw + 2.0 * k2.yaw + 2.0 * k3.yaw + k4.yaw);
    result.speed += scale * (k1.speed + 2.0 * k2.speed + 2.0 * k3.speed + k4.speed);
    result.steer += scale * (k1.steer + 2.0 * k2.steer + 2.0 * k3.steer + k4.steer);
    return result;
}

ReplayNode sampleReplayVehicle(const ReplayVehicle& vehicle, double time, std::size_t* interval_index) {
    if (vehicle.nodes.empty()) throw std::invalid_argument("cannot sample empty trajectory");
    if (time <= vehicle.nodes.front().time + kTimeTolerance) {
        if (interval_index) *interval_index = 0;
        return vehicle.nodes.front();
    }
    if (time >= vehicle.nodes.back().time - kTimeTolerance) {
        if (interval_index) *interval_index = vehicle.nodes.size() - 1;
        return vehicle.nodes.back();
    }
    const auto upper = std::upper_bound(vehicle.nodes.begin(), vehicle.nodes.end(), time,
        [](double value, const ReplayNode& node) { return value < node.time; });
    const std::size_t interval = static_cast<std::size_t>(std::distance(vehicle.nodes.begin(), upper) - 1);
    if (interval_index) *interval_index = interval;
    return integrateReplayNode(vehicle.nodes[interval], time - vehicle.nodes[interval].time,
                               vehicle.parameters.wheel_base);
}

ReplayPose replayBodyCenter(const ReplayPose& rear_axle, double rear_axle_to_center) {
    return {rear_axle.x + rear_axle_to_center * std::cos(rear_axle.yaw),
            rear_axle.y + rear_axle_to_center * std::sin(rear_axle.yaw), rear_axle.yaw};
}

double replayAabbViolation(const ReplayPose& center, const ReplayAabb& box) {
    return std::max({box.min_x - center.x, center.x - box.max_x,
                     box.min_y - center.y, center.y - box.max_y, 0.0});
}

std::vector<ReplayViolation> checkReplayInstant(const ReplayCase& replay_case, double time,
                                                double tolerance) {
    std::vector<ReplayNode> states;
    std::vector<std::size_t> intervals;
    std::vector<ReplayPose> centers;
    for (const ReplayVehicle& vehicle : replay_case.vehicles) {
        std::size_t interval = 0;
        states.push_back(sampleReplayVehicle(vehicle, time, &interval));
        intervals.push_back(interval);
        centers.push_back(replayBodyCenter(states.back().rear_axle, vehicle.parameters.rear_axle_to_center));
    }
    std::vector<ReplayViolation> result;
    for (std::size_t i = 0; i < replay_case.vehicles.size(); ++i) {
        const auto& vehicle = replay_case.vehicles[i];
        const double outside = unionAabbViolation(vehicle, centers[i], intervals[i], time);
        if (outside > tolerance) result.push_back({ReplayViolationType::AABB, time, vehicle.id, -1,
                                                   outside, 0.0, outside, centers[i], centers[i]});
        for (std::size_t j = i + 1; j < replay_case.vehicles.size(); ++j) {
            const auto& other = replay_case.vehicles[j];
            const double required = vehicle.parameters.disc_radius + other.parameters.disc_radius;
            const double distance = std::hypot(centers[i].x - centers[j].x, centers[i].y - centers[j].y);
            if (distance - required < -tolerance) result.push_back({ReplayViolationType::PARTICIPANT_DISC, time, vehicle.id, other.id, distance, required, required - distance, centers[i], centers[j]});
            const double overlap = obbOverlap(states[i], vehicle.parameters, states[j], other.parameters);
            if (overlap > tolerance) result.push_back({ReplayViolationType::OBB, time, vehicle.id, other.id, overlap, 0.0, overlap, centers[i], centers[j]});
        }
        for (const ReplayStaticVehicle& obstacle : replay_case.static_vehicles) {
            const double required = vehicle.parameters.disc_radius + obstacle.disc_radius;
            const double distance = std::hypot(centers[i].x - obstacle.disc_center.x,
                                               centers[i].y - obstacle.disc_center.y);
            if (distance - required < -tolerance) result.push_back({ReplayViolationType::STATIC_DISC, time, vehicle.id, obstacle.id, distance, required, required - distance, centers[i], obstacle.disc_center});
        }
    }
    return result;
}

ReplayDiagnostics scanReplayCase(const ReplayCase& replay_case, std::size_t substeps,
                                 double tolerance) {
    if (substeps == 0) throw std::invalid_argument("validation substeps must be positive");
    ReplayDiagnostics diagnostics;
    diagnostics.minimum_participant_clearance = std::numeric_limits<double>::infinity();
    diagnostics.minimum_static_clearance = std::numeric_limits<double>::infinity();
    std::vector<double> sample_times;
    const auto& nodes = replay_case.vehicles.front().nodes;
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
        const double dt = nodes[i + 1].time - nodes[i].time;
        for (std::size_t substep = 0; substep < substeps; ++substep) {
            sample_times.push_back(nodes[i].time + dt * static_cast<double>(substep) / static_cast<double>(substeps));
        }
    }
    sample_times.push_back(nodes.back().time);
    double maximum_gap = 0.0;
    for (std::size_t k = 0; k < sample_times.size(); ++k) {
        if (k > 0) maximum_gap = std::max(maximum_gap, sample_times[k] - sample_times[k - 1]);
        std::vector<ReplayPose> centers;
        for (const ReplayVehicle& vehicle : replay_case.vehicles) {
            const ReplayNode state = sampleReplayVehicle(vehicle, sample_times[k]);
            centers.push_back(replayBodyCenter(state.rear_axle, vehicle.parameters.rear_axle_to_center));
        }
        for (std::size_t i = 0; i < replay_case.vehicles.size(); ++i) {
            for (std::size_t j = i + 1; j < replay_case.vehicles.size(); ++j) {
                diagnostics.minimum_participant_clearance = std::min(diagnostics.minimum_participant_clearance,
                    discClearance(centers[i], replay_case.vehicles[i].parameters.disc_radius,
                                  centers[j], replay_case.vehicles[j].parameters.disc_radius));
            }
            for (const ReplayStaticVehicle& obstacle : replay_case.static_vehicles) {
                diagnostics.minimum_static_clearance = std::min(diagnostics.minimum_static_clearance,
                    discClearance(centers[i], replay_case.vehicles[i].parameters.disc_radius,
                                  obstacle.disc_center, obstacle.disc_radius));
            }
        }
        auto violations = checkReplayInstant(replay_case, sample_times[k], tolerance);
        diagnostics.violations.insert(diagnostics.violations.end(), violations.begin(), violations.end());
    }
    if (!std::isfinite(diagnostics.minimum_participant_clearance)) diagnostics.minimum_participant_clearance = 0.0;
    if (!std::isfinite(diagnostics.minimum_static_clearance)) diagnostics.minimum_static_clearance = 0.0;
    buildIntervals(diagnostics.violations, maximum_gap * 1.01, diagnostics.intervals);
    return diagnostics;
}

double wrapReplayTime(double current, double wall_dt, double playback_rate, double total_time) {
    if (total_time <= 0.0 || wall_dt < 0.0 || playback_rate < 0.0) throw std::invalid_argument("invalid replay clock argument");
    const double advanced = current + wall_dt * playback_rate;
    if (advanced < total_time) return advanced;
    return std::fmod(advanced, total_time);
}

const char* replayViolationTypeName(ReplayViolationType type) {
    switch (type) {
        case ReplayViolationType::PARTICIPANT_DISC: return "participant_disc";
        case ReplayViolationType::STATIC_DISC: return "static_disc";
        case ReplayViolationType::AABB: return "aabb";
        case ReplayViolationType::OBB: return "obb";
    }
    return "unknown";
}

}  // namespace forklift_nlp_recovery
