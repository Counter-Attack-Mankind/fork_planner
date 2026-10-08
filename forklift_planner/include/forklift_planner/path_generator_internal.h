#pragma once

#include <string>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/common/geometry2d.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"

namespace forklift_planner {
namespace path_internal {

constexpr double kPi = M_PI;
constexpr double kEps = 1e-6;

using geometry2d::Pt;

struct Sample {
    Pt p;
    double theta = 0.0;
};

struct TurnCurve {
    std::vector<Pt> pts;
    std::vector<double> headings;
    double t_in = 0.0;
    double t_out = 0.0;
};

struct PlannedTurn {
    bool active = false;
    TurnCurve curve;
    double heading = 0.0;
    Pt start;
};

enum class HDir { LEFT, RIGHT };

double corridor_center_y(const MapParam& mp, int corr_id);
double corridor_lane_y(const MapParam& mp, int corr_id, HDir dir);
double corridor_min_y(const MapParam& mp, int corr_id);
double corridor_max_y(const MapParam& mp, int corr_id);
double dual_lane_offset(double width);
bool point_in_shelf(const MapParam& mp, const Pt& p);
double normalize_angle(double a);

void push_point(std::vector<Pt>& polyline, const Pt& p);
void push_sample(std::vector<Sample>& samples, const Pt& p, double theta);
std::vector<Pt> simplify_polyline(const std::vector<Pt>& in);
void record_debug_layer(PathGenerationInfo* info,
                        DebugPathLayerType type,
                        const std::string& label,
                        const std::vector<Pt>& pts);
void record_debug_layer_from_samples(PathGenerationInfo* info,
                                     DebugPathLayerType type,
                                     const std::string& label,
                                     const std::vector<Sample>& samples,
                                     size_t begin);
void record_debug_layer_from_path(PathGenerationInfo* info,
                                  DebugPathLayerType type,
                                  const std::string& label,
                                  const RoughPath& path);

double curvature_at(double s, double total_len, double ramp_len,
                    double hold_len, double peak_k);
double quintic_blend(double t);
double quintic_blend_derivative(double t);

bool is_short_parallel_shift(const Pt& a, const Pt& b, const Pt& c,
                             const Pt& d, double max_shift);
void append_lane_shift(std::vector<Sample>& out, const Pt& b, const Pt& c,
                       const Pt& direction, double lead_in, double lead_out,
                       double ds);

TurnCurve build_clothoid_turn(double signed_angle, double max_curvature,
                              double rate_ramp_len, double ds);
TurnCurve build_g2_spiral_turn(double signed_angle, double tangent_len,
                               double ds);
TurnCurve fit_clothoid_turn(double signed_angle, double max_curvature,
                            double desired_ramp_len, double max_tangent,
                            double ds);
RoughPath build_arc_path(const std::vector<Pt>& simplified,
                         const PlannerParam& pp,
                         const Slot& src,
                         double max_curvature,
                         double sample_ds);
void warn_if_reverse_segments(const RoughPath& path);

}  // namespace path_internal
}  // namespace forklift_planner
