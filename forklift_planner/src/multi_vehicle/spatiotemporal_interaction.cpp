#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace forklift_planner {
namespace multi_vehicle {

namespace {


std::array<InteractionPoint, 4> obbCorners(const OBB& body) {
    const double c = std::cos(body.theta);
    const double s = std::sin(body.theta);
    const double fx = c * body.half_l;
    const double fy = s * body.half_l;
    const double lx = -s * body.half_w;
    const double ly = c * body.half_w;
    return {{{body.x + fx + lx, body.y + fy + ly},
             {body.x - fx + lx, body.y - fy + ly},
             {body.x - fx - lx, body.y - fy - ly},
             {body.x + fx - lx, body.y + fy - ly}}};
}

double curvatureSpeedAt(const VehicleAgent& vehicle,
                        const MultiVehicleConfig& config,
                        double query_s) {
    if (config.lat_accel_max <= 0.0 || vehicle.track.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const double length = vehicle.track.length();
    const double s = std::max(0.0, std::min(query_s, length));
    constexpr double sample_ds = 0.05;
    const RoughWp pa = vehicle.track.poseAtS(std::max(0.0, s - sample_ds));
    const RoughWp pb = vehicle.track.poseAtS(s);
    const RoughWp pc = vehicle.track.poseAtS(std::min(length, s + sample_ds));
    const double abx = pb.x - pa.x;
    const double aby = pb.y - pa.y;
    const double acx = pc.x - pa.x;
    const double acy = pc.y - pa.y;
    const double lab = std::hypot(abx, aby);
    const double lbc = std::hypot(pc.x - pb.x, pc.y - pb.y);
    const double lac = std::hypot(acx, acy);
    if (lab < 1e-4 || lbc < 1e-4 || lac < 1e-4) {
        return std::numeric_limits<double>::infinity();
    }
    const double kappa =
        2.0 * std::abs(abx * acy - aby * acx) / (lab * lbc * lac);
    if (kappa < 1e-3) {
        return std::numeric_limits<double>::infinity();
    }
    return std::max(std::sqrt(config.lat_accel_max / kappa),
                    config.nominal_speed * config.creep_ratio);
}

double intervalDistance(double value, double begin, double end) {
    if (value < begin) return begin - value;
    if (value > end) return value - end;
    return 0.0;
}

double targetSpeed(VehicleAction action, const MultiVehicleConfig& config) {
    switch (action) {
        case VehicleAction::STOP:
            return 0.0;
        case VehicleAction::CREEP:
            return config.nominal_speed * config.creep_ratio;
        case VehicleAction::YIELD:
            return config.nominal_speed * config.yield_ratio;
        case VehicleAction::NOMINAL:
            return config.nominal_speed;
        case VehicleAction::BOOST:
            return config.enable_boost
                ? std::min(config.max_speed,
                           config.nominal_speed * config.boost_ratio)
                : config.nominal_speed;
    }
    return 0.0;
}

PredictedKinematicSample interpolatePrediction(
    const VehicleAgent& vehicle, const PredictedKinematicSample& begin,
    const PredictedKinematicSample& end, double ratio) {
    ratio = std::max(0.0, std::min(1.0, ratio));
    const double s = begin.s + ratio * (end.s - begin.s);
    const RoughWp pose = vehicle.track.poseAtS(s);
    const RoughWp begin_pose = vehicle.track.poseAtS(begin.s);
    const double begin_cos = std::cos(begin_pose.theta);
    const double begin_sin = std::sin(begin_pose.theta);
    const double offset_x = begin.body.x - begin_pose.x;
    const double offset_y = begin.body.y - begin_pose.y;
    const double longitudinal_offset =
        offset_x * begin_cos + offset_y * begin_sin;
    const double lateral_offset =
        -offset_x * begin_sin + offset_y * begin_cos;
    const double pose_cos = std::cos(pose.theta);
    const double pose_sin = std::sin(pose.theta);
    OBB body = begin.body;
    body.x = pose.x + longitudinal_offset * pose_cos -
             lateral_offset * pose_sin;
    body.y = pose.y + longitudinal_offset * pose_sin +
             lateral_offset * pose_cos;
    body.theta = pose.theta;
    return PredictedKinematicSample{
        begin.t + ratio * (end.t - begin.t),
        s,
        begin.speed + ratio * (end.speed - begin.speed),
        body};
}

}  // namespace

std::vector<InteractionPoint> intersectObbs(const OBB& a, const OBB& b) {
    const auto a_corners = obbCorners(a);
    const auto b_corners = obbCorners(b);
    std::vector<InteractionPoint> polygon(a_corners.begin(), a_corners.end());
    constexpr double kEps = 1e-9;
    for (size_t edge = 0; edge < b_corners.size() && !polygon.empty(); ++edge) {
        const InteractionPoint p0 = b_corners[edge];
        const InteractionPoint p1 = b_corners[(edge + 1) % b_corners.size()];
        const double ex = p1.x - p0.x;
        const double ey = p1.y - p0.y;
        auto signedSide = [&](const InteractionPoint& p) {
            return ex * (p.y - p0.y) - ey * (p.x - p0.x);
        };
        auto intersection = [&](const InteractionPoint& from,
                                const InteractionPoint& to) {
            const double from_side = signedSide(from);
            const double to_side = signedSide(to);
            const double denom = from_side - to_side;
            const double ratio = std::abs(denom) <= kEps
                ? 0.0 : from_side / denom;
            return InteractionPoint{
                from.x + ratio * (to.x - from.x),
                from.y + ratio * (to.y - from.y)};
        };

        std::vector<InteractionPoint> clipped;
        clipped.reserve(polygon.size() + 1);
        InteractionPoint previous = polygon.back();
        bool previous_inside = signedSide(previous) >= -kEps;
        for (const InteractionPoint& current : polygon) {
            const bool current_inside = signedSide(current) >= -kEps;
            if (current_inside != previous_inside) {
                clipped.push_back(intersection(previous, current));
            }
            if (current_inside) clipped.push_back(current);
            previous = current;
            previous_inside = current_inside;
        }
        polygon = std::move(clipped);
    }
    return polygon;
}

std::vector<PredictedKinematicSample> predictTrajectory(
    const VehicleAgent& vehicle, const MapParam& map_param,
    const MultiVehicleConfig& config, VehicleAction target_action,
    double prediction_horizon) {
    const double horizon =
        std::max(config.prediction_step, prediction_horizon);
    const double prediction_step = std::max(0.02, config.prediction_step);
    const int prediction_count = std::max(
        1, static_cast<int>(std::ceil(horizon / prediction_step)));
    std::vector<PredictedKinematicSample> output;
    if (!vehicle.active() || vehicle.track.empty()) return output;
    output.reserve(static_cast<size_t>(prediction_count + 1));
    
    double s = std::max(0.0,
                        std::min(vehicle.path_s, vehicle.track.length()));
    double speed = std::max(0.0, vehicle.current_speed);

    output.push_back(PredictedKinematicSample{
        0.0, s, speed,
        makeBody(vehicle.track.poseAtS(s), map_param, 0.0)});
    for (int k = 1; k <= prediction_count; ++k) {
        const double previous_t = (k - 1) * prediction_step;
        const double t = std::min(horizon, k * prediction_step);
        const double step = t - previous_t;
        if (step <= 1e-9) continue;
        if (s >= vehicle.track.length() - 1e-9) {
            s = vehicle.track.length();
            speed = 0.0;
        } else {
            const double desired = std::min(
                targetSpeed(target_action, config),
                curvatureSpeedAt(vehicle, config, s));
            if (desired > speed) {
                speed = std::min(desired, speed + config.max_accel * step);
            } else {
                speed = std::max(desired, speed - config.max_decel * step);
            }
            s = std::min(vehicle.track.length(), s + speed * step);
        }
        output.push_back(PredictedKinematicSample{
            t, s, speed,
            makeBody(vehicle.track.poseAtS(s), map_param, 0.0)});
    }
    return output;
}

double predictionTimeAtS(
    const std::vector<PredictedKinematicSample>& prediction,
    double target_s) {
    if (prediction.empty()) return std::numeric_limits<double>::infinity();
    if (target_s <= prediction.front().s + 1e-9) return 0.0;
    for (std::size_t i = 1; i < prediction.size(); ++i) {
        if (prediction[i].s + 1e-9 < target_s) continue;
        const double ds = prediction[i].s - prediction[i - 1].s;
        if (ds <= 1e-9) return prediction[i].t;
        const double ratio = std::max(
            0.0, std::min(1.0,
                          (target_s - prediction[i - 1].s) / ds));
        return prediction[i - 1].t +
            ratio * (prediction[i].t - prediction[i - 1].t);
    }
    return std::numeric_limits<double>::infinity();
}

PairInteractionResult detectPairInteractionFromPredictions(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const std::vector<PredictedKinematicSample>& prediction_a,
    const std::vector<PredictedKinematicSample>& prediction_b) {
    PairInteractionResult result;
    result.vehicle_a = vehicle_a.id;
    result.vehicle_b = vehicle_b.id;
    result.path_gen_a = vehicle_a.path_gen;
    result.path_gen_b = vehicle_b.path_gen;
    result.potential_zones = potential_zones;

    const size_t count = std::min(prediction_a.size(), prediction_b.size());
    if (count == 0) return result;

    auto inspect = [&](const PredictedKinematicSample& sample_a,
                       const PredictedKinematicSample& sample_b) {
        const bool hit = overlaps(sample_a.body, sample_b.body);
        if (!hit) {
            return !result.event.valid;
        }
        if (!result.event.valid) {
            result.event.valid = true;
            result.type = PairInteractionType::CROSSING;
            result.event.first_overlap_t = sample_a.t;
            const double s_a = sample_a.s;
            const double s_b = sample_b.s;
            result.event.collision_s_a = s_a;
            result.event.collision_s_b = s_b;
            result.event.danger_s_a = s_a;
            result.event.danger_s_b = s_b;
            result.event.ttc_a = predictionTimeAtS(prediction_a, s_a);
            result.event.ttc_b = predictionTimeAtS(prediction_b, s_b);
            double best_score = std::numeric_limits<double>::infinity();
            for (size_t zone_index = 0;
                 zone_index < potential_zones.size(); ++zone_index) {
                const PotentialConflictZone& zone =
                    potential_zones[zone_index];
                const double score =
                    intervalDistance(s_a, zone.s_self_enter,
                                     zone.s_self_exit) +
                    intervalDistance(s_b, zone.s_other_enter,
                                     zone.s_other_exit);
                if (score < best_score) {
                    best_score = score;
                    result.event.associated_zone_index =
                        static_cast<int>(zone_index);
                }
            }
        }
        result.event.last_t = sample_a.t;
        auto polygon = intersectObbs(sample_a.body, sample_b.body);
        if (polygon.size() >= 3) {
            result.event.timed_overlaps.push_back(
                TimedOverlapGeometry{sample_a.t, std::move(polygon)});
        }
        return true;
    };

    if (!inspect(prediction_a.front(), prediction_b.front())) return result;

    constexpr double kMaxCheckDt = 0.05;
    constexpr double kMinCheckDt = 0.005;
    constexpr double kMaxRelativeTravel = 0.01;
    for (size_t k = 1; k < count; ++k) {
        const double interval_a = prediction_a[k].t - prediction_a[k - 1].t;
        const double interval_b = prediction_b[k].t - prediction_b[k - 1].t;
        const double interval = std::min(interval_a, interval_b);
        if (interval <= 1e-9) continue;

        const double velocity_a_x =
            (prediction_a[k].body.x - prediction_a[k - 1].body.x) / interval_a;
        const double velocity_a_y =
            (prediction_a[k].body.y - prediction_a[k - 1].body.y) / interval_a;
        const double velocity_b_x =
            (prediction_b[k].body.x - prediction_b[k - 1].body.x) / interval_b;
        const double velocity_b_y =
            (prediction_b[k].body.y - prediction_b[k - 1].body.y) / interval_b;
        const double relative_speed = std::hypot(
            velocity_a_x - velocity_b_x, velocity_a_y - velocity_b_y);
        const double relative_check_dt = relative_speed > 1e-9
            ? kMaxRelativeTravel / relative_speed : kMaxCheckDt;
        const double check_dt = std::max(
            kMinCheckDt, std::min(kMaxCheckDt, relative_check_dt));
        const int subdivisions = std::max(
            1, static_cast<int>(std::ceil(interval / check_dt)));
        for (int substep = 1; substep <= subdivisions; ++substep) {
            const double ratio =
                static_cast<double>(substep) / subdivisions;
            const PredictedKinematicSample sample_a = interpolatePrediction(
                vehicle_a, prediction_a[k - 1], prediction_a[k], ratio);
            const PredictedKinematicSample sample_b = interpolatePrediction(
                vehicle_b, prediction_b[k - 1], prediction_b[k], ratio);
            if (!inspect(sample_a, sample_b)) return result;
        }
    }
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
