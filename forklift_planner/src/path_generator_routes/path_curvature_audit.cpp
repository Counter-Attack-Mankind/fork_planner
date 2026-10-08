// Read-only audit of the final sampled B <-> A1 RoughPath geometry.
#include <ros/ros.h>
#include <ros/console.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"

namespace {

using forklift_planner::multi_vehicle::MultiVehicleConfig;
using forklift_planner::multi_vehicle::footprintInsideField;

constexpr double kEpsilon = 1e-8;

double normAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  return angle;
}

const char* wpTypeName(WpType type) {
  return type == WpType::REVERSE ? "REVERSE" : "FORWARD";
}

const char* layerTypeName(DebugPathLayerType type) {
  switch (type) {
    case DebugPathLayerType::SKELETON: return "SKELETON";
    case DebugPathLayerType::CLOTHOID: return "CLOTHOID";
    case DebugPathLayerType::LANE_SHIFT: return "LANE_SHIFT";
    case DebugPathLayerType::ARC_FALLBACK: return "ARC_FALLBACK";
  }
  return "UNKNOWN";
}

std::string trim(const std::string& value) {
  std::size_t begin = 0;
  while (begin < value.size() && std::isspace(
      static_cast<unsigned char>(value[begin]))) ++begin;
  std::size_t end = value.size();
  while (end > begin && std::isspace(
      static_cast<unsigned char>(value[end - 1]))) --end;
  return value.substr(begin, end - begin);
}

bool startsWith(const std::string& value, const char* prefix) {
  const std::string expected(prefix);
  return value.size() >= expected.size() &&
         value.compare(0, expected.size(), expected) == 0;
}

struct CatalogSelection {
  bool file_exists = false;
  bool usable = false;
  std::string format;
  int declared_slot_count = -1;
  std::set<int> slots;
};

CatalogSelection readCatalogSelection(const std::string& path,
                                      int expected_slot_count) {
  CatalogSelection result;
  std::ifstream input(path);
  if (!input.is_open()) return result;
  result.file_exists = true;

  enum class Section { NONE, B_TO_A1, A1_TO_B };
  Section section = Section::NONE;
  std::set<int> b_to_a1;
  std::set<int> a1_to_b;
  std::string line;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') continue;
    if (startsWith(line, "format:")) {
      result.format = trim(line.substr(7));
    } else if (startsWith(line, "slot_count:")) {
      std::istringstream stream(trim(line.substr(11)));
      stream >> result.declared_slot_count;
    } else if (startsWith(line, "section:")) {
      const std::string name = trim(line.substr(8));
      section = name == "b_to_a1" ? Section::B_TO_A1
              : name == "a1_to_b" ? Section::A1_TO_B : Section::NONE;
    } else if (startsWith(line, "leg:")) {
      std::string leg_tag, arc_tag, points_tag;
      int slot = -1, arc = 0, points = 0;
      std::istringstream stream(line);
      stream >> leg_tag >> slot >> arc_tag >> arc >> points_tag >> points;
      if (!stream || slot < 0 || points <= 0 || arc_tag != "arc" ||
          points_tag != "points") continue;
      if (section == Section::B_TO_A1) b_to_a1.insert(slot);
      if (section == Section::A1_TO_B) a1_to_b.insert(slot);
    }
  }
  std::set_intersection(b_to_a1.begin(), b_to_a1.end(),
                        a1_to_b.begin(), a1_to_b.end(),
                        std::inserter(result.slots, result.slots.end()));
  result.usable = result.format == "forklift_a1_cycle_path_catalog_v1" &&
                  result.declared_slot_count == expected_slot_count &&
                  !result.slots.empty();
  return result;
}

Slot makeA1Slot(const MultiVehicleConfig& config) {
  Slot a1;
  a1.id = 101;
  a1.row_id = 0;
  a1.col = -1;
  a1.cx = config.a1_pickup_center_x;
  a1.cy = config.a1_pickup_center_y;
  a1.pre_dock_x = config.a1_pre_dock_x;
  a1.pre_dock_y = config.a1_pre_dock_y;
  a1.dock_theta = config.a1_pickup_theta;
  return a1;
}

RoughWp interpolatePose(const RoughWp& a, const RoughWp& b, double ratio) {
  RoughWp result;
  result.x = a.x + (b.x - a.x) * ratio;
  result.y = a.y + (b.y - a.y) * ratio;
  result.theta = normAngle(a.theta + normAngle(b.theta - a.theta) * ratio);
  result.type = ratio < 0.5 ? a.type : b.type;
  return result;
}

// This is the currently active, read-only subset of TaskAllocator::validatePath:
// empty path, optional arc rejection, kink rejection, and footprint boundary.
// Shelf rejection is intentionally refused below if enabled, because its slot
// sweep exemption is private to TaskAllocator and must not be approximated here.
std::string validateGeneratedPath(const RoughPath& path,
                                  const PathGenerationInfo& info,
                                  const MapParam& map_param,
                                  const MultiVehicleConfig& config) {
  if (path.size() < 2) return "EMPTY_PATH";
  if (config.reject_curvature_discontinuity && info.used_arc_fallback) {
    return "CURVATURE_DISCONTINUITY";
  }
  if (config.reject_path_kinks) {
    bool have_previous = false;
    double previous_dx = 0.0, previous_dy = 0.0, previous_length = 0.0;
    WpType previous_type = path.front().type;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const double dx = path[i + 1].x - path[i].x;
      const double dy = path[i + 1].y - path[i].y;
      const double length = std::hypot(dx, dy);
      if (length < 1e-4) continue;
      const WpType type = path[i + 1].type;
      if (have_previous) {
        double cosine = (previous_dx * dx + previous_dy * dy) /
                        (previous_length * length);
        cosine = std::max(-1.0, std::min(1.0, cosine));
        const double angle = std::acos(cosine);
        const bool legal_cusp = previous_type != type &&
                                angle >= config.kink_cusp_angle;
        if (angle > config.kink_min_angle && !legal_cusp) return "KINK";
      }
      previous_dx = dx;
      previous_dy = dy;
      previous_length = length;
      previous_type = type;
      have_previous = true;
    }
  }
  if (config.reject_boundary_violations) {
    const double check_ds = std::max(0.005, config.path_validation_step);
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const double length = std::hypot(path[i + 1].x - path[i].x,
                                       path[i + 1].y - path[i].y);
      const int count = std::max(1, static_cast<int>(std::ceil(length / check_ds)));
      for (int sample = 0; sample < count; ++sample) {
        const double ratio = static_cast<double>(sample) / count;
        if (!footprintInsideField(interpolatePose(path[i], path[i + 1], ratio),
                                  map_param, 0.0)) {
          return "FOOTPRINT_OUT_OF_BOUNDS";
        }
      }
    }
    if (!footprintInsideField(path.back(), map_param, 0.0)) {
      return "FOOTPRINT_OUT_OF_BOUNDS";
    }
  }
  return "NONE";
}

struct LayerRange {
  DebugPathLayerType type = DebugPathLayerType::SKELETON;
  std::string label;
  double s_min = 0.0;
  double s_max = 0.0;
};

struct CurvatureSample {
  std::size_t index = 0;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  WpType type = WpType::FORWARD;
  bool curvature_valid = false;
  double kappa = 0.0;
  double equivalent_steer = 0.0;
  bool rate_valid = false;
  double dkappa_ds = 0.0;
  bool cusp = false;
  std::string layer = "ORDINARY_OR_UNTAGGED_JOIN";
};

struct RouteAudit {
  std::string direction;
  int slot = -1;
  int row = -1;
  int col = -1;
  std::string side;
  RoughPath path;
  PathGenerationInfo generation_info;
  std::vector<CurvatureSample> samples;
  std::vector<LayerRange> layer_ranges;
  std::vector<double> cusp_s;
  double path_length = 0.0;
  double max_abs_curvature = 0.0;
  double max_abs_curvature_s = 0.0;
  double max_abs_curvature_x = 0.0;
  double max_abs_curvature_y = 0.0;
  double max_abs_equivalent_steer = 0.0;
  std::string max_curvature_layer = "ORDINARY_OR_UNTAGGED_JOIN";
  double max_abs_dkappa_ds = 0.0;
  double max_abs_dkappa_ds_s = 0.0;
  double max_abs_dkappa_ds_x = 0.0;
  double max_abs_dkappa_ds_y = 0.0;
  std::string max_rate_layer = "ORDINARY_OR_UNTAGGED_JOIN";
  double curvature_rate_ratio = 0.0;
  bool curvature_limit_exceeded = false;
  int clothoid_count = 0;
  int lane_shift_count = 0;
  int arc_fallback_count = 0;
  double anomaly_distance_to_a1 = 0.0;
  bool max_anomaly_near_a1 = false;
};

double threePointCurvature(const RoughWp& first, const RoughWp& middle,
                           const RoughWp& last, bool* valid) {
  const double ab = std::hypot(middle.x - first.x, middle.y - first.y);
  const double bc = std::hypot(last.x - middle.x, last.y - middle.y);
  const double ac = std::hypot(last.x - first.x, last.y - first.y);
  const double denominator = ab * bc * ac;
  if (ab < 1e-5 || bc < 1e-5 || ac < 1e-5 || denominator < kEpsilon) {
    *valid = false;
    return 0.0;
  }
  const double cross = (middle.x - first.x) * (last.y - first.y) -
                       (middle.y - first.y) * (last.x - first.x);
  *valid = true;
  return 2.0 * cross / denominator;
}

std::size_t nearestPathIndex(const RoughPath& path,
                             const DebugPathPoint& point) {
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = std::hypot(path[i].x - point.x,
                                       path[i].y - point.y);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

int layerPriority(DebugPathLayerType type) {
  switch (type) {
    case DebugPathLayerType::ARC_FALLBACK: return 4;
    case DebugPathLayerType::LANE_SHIFT: return 3;
    case DebugPathLayerType::CLOTHOID: return 2;
    case DebugPathLayerType::SKELETON: return 1;
  }
  return 0;
}

std::string classifyPoint(const CurvatureSample& sample,
                          const PathGenerationInfo& info,
                          double tolerance) {
  std::string best = "ORDINARY_OR_UNTAGGED_JOIN";
  double best_distance = std::numeric_limits<double>::infinity();
  int best_priority = -1;
  for (const DebugPathLayer& layer : info.debug_layers) {
    for (const DebugPathPoint& point : layer.points) {
      const double distance = std::hypot(sample.x - point.x, sample.y - point.y);
      const int priority = layerPriority(layer.type);
      if (distance <= tolerance &&
          (distance < best_distance - 1e-9 ||
           (std::fabs(distance - best_distance) <= 1e-9 &&
            priority > best_priority))) {
        best_distance = distance;
        best_priority = priority;
        best = std::string(layerTypeName(layer.type)) + ":" + layer.label;
      }
    }
  }
  return best;
}

RouteAudit auditRoute(const std::string& direction, const Slot& slot,
                      RoughPath path, PathGenerationInfo info,
                      const MapParam& map_param, const Slot& a1,
                      double kappa_limit, double reference_rate,
                      double a1_near_radius) {
  RouteAudit audit;
  audit.direction = direction;
  audit.slot = slot.id;
  audit.row = slot.row_id;
  audit.col = slot.col;
  audit.side = slot.cx < map_param.field_width * 0.5 ? "LEFT" : "RIGHT";
  audit.path = std::move(path);
  audit.generation_info = std::move(info);
  audit.samples.resize(audit.path.size());
  if (audit.path.empty()) return audit;

  std::vector<double> cumulative_s(audit.path.size(), 0.0);
  for (std::size_t i = 1; i < audit.path.size(); ++i) {
    cumulative_s[i] = cumulative_s[i - 1] +
        std::hypot(audit.path[i].x - audit.path[i - 1].x,
                   audit.path[i].y - audit.path[i - 1].y);
    if (audit.path[i].type != audit.path[i - 1].type) {
      audit.cusp_s.push_back(cumulative_s[i]);
    }
  }
  audit.path_length = cumulative_s.back();

  for (const DebugPathLayer& layer : audit.generation_info.debug_layers) {
    if (layer.type == DebugPathLayerType::CLOTHOID) ++audit.clothoid_count;
    if (layer.type == DebugPathLayerType::LANE_SHIFT) ++audit.lane_shift_count;
    if (layer.type == DebugPathLayerType::ARC_FALLBACK) ++audit.arc_fallback_count;
    if (layer.points.empty()) continue;
    LayerRange range;
    range.type = layer.type;
    range.label = layer.label;
    range.s_min = std::numeric_limits<double>::infinity();
    range.s_max = 0.0;
    for (const DebugPathPoint& point : layer.points) {
      const double s = cumulative_s[nearestPathIndex(audit.path, point)];
      range.s_min = std::min(range.s_min, s);
      range.s_max = std::max(range.s_max, s);
    }
    audit.layer_ranges.push_back(range);
  }

  const double layer_tolerance = std::max(0.03, 2.5 * map_param.path_resolution);
  for (std::size_t i = 0; i < audit.path.size(); ++i) {
    CurvatureSample& sample = audit.samples[i];
    sample.index = i;
    sample.s = cumulative_s[i];
    sample.x = audit.path[i].x;
    sample.y = audit.path[i].y;
    sample.type = audit.path[i].type;
    sample.cusp = (i > 0 && audit.path[i].type != audit.path[i - 1].type) ||
                  (i + 1 < audit.path.size() &&
                   audit.path[i].type != audit.path[i + 1].type);
    sample.layer = classifyPoint(sample, audit.generation_info, layer_tolerance);
    if (i == 0 || i + 1 >= audit.path.size() || sample.cusp ||
        audit.path[i - 1].type != audit.path[i].type ||
        audit.path[i + 1].type != audit.path[i].type) continue;
    sample.kappa = threePointCurvature(audit.path[i - 1], audit.path[i],
                                       audit.path[i + 1],
                                       &sample.curvature_valid);
    if (!sample.curvature_valid) continue;
    sample.equivalent_steer = std::atan(map_param.wheel_base * sample.kappa);
    if (std::fabs(sample.kappa) > audit.max_abs_curvature) {
      audit.max_abs_curvature = std::fabs(sample.kappa);
      audit.max_abs_curvature_s = sample.s;
      audit.max_abs_curvature_x = sample.x;
      audit.max_abs_curvature_y = sample.y;
      audit.max_abs_equivalent_steer = std::fabs(sample.equivalent_steer);
      audit.max_curvature_layer = sample.layer;
    }
  }

  std::size_t previous = audit.samples.size();
  for (std::size_t i = 0; i < audit.samples.size(); ++i) {
    CurvatureSample& sample = audit.samples[i];
    if (!sample.curvature_valid) continue;
    if (previous < audit.samples.size() &&
        audit.samples[previous].type == sample.type) {
      const double ds = sample.s - audit.samples[previous].s;
      if (ds > 1e-6) {
        sample.rate_valid = true;
        sample.dkappa_ds =
            (sample.kappa - audit.samples[previous].kappa) / ds;
        if (std::fabs(sample.dkappa_ds) > audit.max_abs_dkappa_ds) {
          audit.max_abs_dkappa_ds = std::fabs(sample.dkappa_ds);
          audit.max_abs_dkappa_ds_s = sample.s;
          audit.max_abs_dkappa_ds_x = sample.x;
          audit.max_abs_dkappa_ds_y = sample.y;
          audit.max_rate_layer = sample.layer;
        }
      }
    }
    previous = i;
  }
  audit.curvature_limit_exceeded = audit.max_abs_curvature > kappa_limit;
  audit.curvature_rate_ratio = reference_rate > 0.0
      ? audit.max_abs_dkappa_ds / reference_rate : 0.0;
  const double curvature_distance = std::hypot(audit.max_abs_curvature_x - a1.cx,
                                                audit.max_abs_curvature_y - a1.cy);
  const double rate_distance = std::hypot(audit.max_abs_dkappa_ds_x - a1.cx,
                                          audit.max_abs_dkappa_ds_y - a1.cy);
  audit.anomaly_distance_to_a1 = std::min(curvature_distance, rate_distance);
  audit.max_anomaly_near_a1 = audit.anomaly_distance_to_a1 <= a1_near_radius;
  return audit;
}

std::string yamlQuote(const std::string& value) {
  std::string output = "\"";
  for (char c : value) {
    if (c == '\\' || c == '"') output.push_back('\\');
    output.push_back(c);
  }
  output.push_back('"');
  return output;
}

void writeSampleCsv(const std::string& path, const std::string& direction,
                    const std::vector<RouteAudit>& routes) {
  std::ofstream output(path);
  if (!output.is_open()) throw std::runtime_error("cannot write " + path);
  output << "direction,slot,index,s,x,y,type,kappa,abs_kappa,"
            "equivalent_steer,dkappa_ds,cusp,debug_layer\n";
  output << std::setprecision(12);
  for (const RouteAudit& route : routes) {
    if (route.direction != direction) continue;
    for (const CurvatureSample& sample : route.samples) {
      output << route.direction << ',' << route.slot << ',' << sample.index
             << ',' << sample.s << ',' << sample.x << ',' << sample.y << ','
             << wpTypeName(sample.type) << ',';
      if (sample.curvature_valid) {
        output << sample.kappa << ',' << std::fabs(sample.kappa) << ','
               << sample.equivalent_steer;
      } else {
        output << ",,";
      }
      output << ',';
      if (sample.rate_valid) output << sample.dkappa_ds;
      output << ',' << (sample.cusp ? 1 : 0) << ','
             << yamlQuote(sample.layer) << '\n';
    }
  }
}

const RouteAudit* worstByCurvature(const std::vector<RouteAudit>& routes) {
  if (routes.empty()) return nullptr;
  return &*std::max_element(routes.begin(), routes.end(),
      [](const RouteAudit& a, const RouteAudit& b) {
        return a.max_abs_curvature < b.max_abs_curvature;
      });
}

const RouteAudit* worstByRate(const std::vector<RouteAudit>& routes) {
  if (routes.empty()) return nullptr;
  return &*std::max_element(routes.begin(), routes.end(),
      [](const RouteAudit& a, const RouteAudit& b) {
        return a.max_abs_dkappa_ds < b.max_abs_dkappa_ds;
      });
}

void writeWorst(std::ostream& output, const RouteAudit* route,
                const std::string& metric) {
  if (route == nullptr) {
    output << "{}";
    return;
  }
  output << "{direction: " << route->direction << ", slot: " << route->slot
         << ", value: "
         << (metric == "curvature" ? route->max_abs_curvature
                                    : route->max_abs_dkappa_ds)
         << ", s: "
         << (metric == "curvature" ? route->max_abs_curvature_s
                                    : route->max_abs_dkappa_ds_s)
         << ", layer: " << yamlQuote(metric == "curvature"
                                      ? route->max_curvature_layer
                                      : route->max_rate_layer) << '}';
}

void writeYaml(const std::string& path, const MapParam& map_param,
               double kappa_limit, double ramp_length, double reference_rate,
               double a1_near_radius, const CatalogSelection& catalog,
               const std::string& catalog_path,
               const std::string& selection_source,
               const std::vector<RouteAudit>& routes,
               const std::map<std::string, int>& rejected) {
  std::ofstream output(path);
  if (!output.is_open()) throw std::runtime_error("cannot write " + path);
  output << std::setprecision(12);
  int b_count = 0, a_count = 0, exceeded = 0, arc_count = 0, near_a1 = 0;
  double left_kappa = 0.0, right_kappa = 0.0;
  double left_rate = 0.0, right_rate = 0.0;
  for (const RouteAudit& route : routes) {
    if (route.direction == "B_TO_A1") ++b_count; else ++a_count;
    if (route.curvature_limit_exceeded) ++exceeded;
    if (route.generation_info.used_arc_fallback) ++arc_count;
    if (route.max_anomaly_near_a1) ++near_a1;
    if (route.side == "LEFT") {
      left_kappa = std::max(left_kappa, route.max_abs_curvature);
      left_rate = std::max(left_rate, route.max_abs_dkappa_ds);
    } else {
      right_kappa = std::max(right_kappa, route.max_abs_curvature);
      right_rate = std::max(right_rate, route.max_abs_dkappa_ds);
    }
  }
  output << "parameters:\n"
         << "  wheel_base: " << map_param.wheel_base << "\n"
         << "  max_steer_angle: " << map_param.max_steer_angle << "\n"
         << "  max_steer_rate: " << map_param.max_steer_rate << "\n"
         << "  turn_speed: " << map_param.turn_speed << "\n"
         << "  path_resolution: " << map_param.path_resolution << "\n"
         << "  kappa_limit: " << kappa_limit << "\n"
         << "  steer_limit: " << map_param.max_steer_angle << "\n"
         << "  turn_ramp_len: " << ramp_length << "\n"
         << "  reference_dkappa_ds: " << reference_rate << "\n"
         << "  a1_near_radius: " << a1_near_radius << "\n"
         << "catalog:\n"
         << "  requested_file: " << yamlQuote(catalog_path) << "\n"
         << "  file_exists: " << (catalog.file_exists ? "true" : "false") << "\n"
         << "  format: " << yamlQuote(catalog.format) << "\n"
         << "  declared_slot_count: " << catalog.declared_slot_count << "\n"
         << "  usable_for_enumeration: " << (catalog.usable ? "true" : "false") << "\n"
         << "  selection_source: " << selection_source << "\n"
         << "summary:\n"
         << "  total_b_to_a1: " << b_count << "\n"
         << "  total_a1_to_b: " << a_count << "\n"
         << "  curvature_limit_exceeded_count: " << exceeded << "\n"
         << "  arc_fallback_count: " << arc_count << "\n"
         << "  max_anomaly_near_a1_count: " << near_a1 << "\n"
         << "  left_max_abs_curvature: " << left_kappa << "\n"
         << "  right_max_abs_curvature: " << right_kappa << "\n"
         << "  right_to_left_curvature_ratio: "
         << (left_kappa > 0.0 ? right_kappa / left_kappa : 0.0) << "\n"
         << "  left_max_abs_dkappa_ds: " << left_rate << "\n"
         << "  right_max_abs_dkappa_ds: " << right_rate << "\n"
         << "  right_to_left_curvature_rate_ratio: "
         << (left_rate > 0.0 ? right_rate / left_rate : 0.0) << "\n"
         << "  worst_curvature: ";
  writeWorst(output, worstByCurvature(routes), "curvature");
  output << "\n  worst_curvature_rate: ";
  writeWorst(output, worstByRate(routes), "rate");
  output << "\n  rejected_routes:\n";
  for (const auto& item : rejected) {
    output << "    " << item.first << ": " << item.second << "\n";
  }
  output << "routes:\n";
  for (const RouteAudit& route : routes) {
    output << "  - direction: " << route.direction << "\n"
           << "    slot: " << route.slot << "\n"
           << "    row: " << route.row << "\n"
           << "    col: " << route.col << "\n"
           << "    side: " << route.side << "\n"
           << "    path_length: " << route.path_length << "\n"
           << "    max_abs_curvature: " << route.max_abs_curvature << "\n"
           << "    max_abs_curvature_s: " << route.max_abs_curvature_s << "\n"
           << "    max_abs_curvature_x: " << route.max_abs_curvature_x << "\n"
           << "    max_abs_curvature_y: " << route.max_abs_curvature_y << "\n"
           << "    max_abs_equivalent_steer: " << route.max_abs_equivalent_steer << "\n"
           << "    max_curvature_layer: " << yamlQuote(route.max_curvature_layer) << "\n"
           << "    max_abs_dkappa_ds: " << route.max_abs_dkappa_ds << "\n"
           << "    max_abs_dkappa_ds_s: " << route.max_abs_dkappa_ds_s << "\n"
           << "    max_abs_dkappa_ds_x: " << route.max_abs_dkappa_ds_x << "\n"
           << "    max_abs_dkappa_ds_y: " << route.max_abs_dkappa_ds_y << "\n"
           << "    max_rate_layer: " << yamlQuote(route.max_rate_layer) << "\n"
           << "    curvature_rate_ratio: " << route.curvature_rate_ratio << "\n"
           << "    curvature_limit_exceeded: "
           << (route.curvature_limit_exceeded ? "true" : "false") << "\n"
           << "    status: " << (route.curvature_limit_exceeded
                                  ? "CURVATURE_LIMIT_EXCEEDED" : "PASS") << "\n"
           << "    used_arc_fallback: "
           << (route.generation_info.used_arc_fallback ? "true" : "false") << "\n"
           << "    clothoid_count: " << route.clothoid_count << "\n"
           << "    lane_shift_count: " << route.lane_shift_count << "\n"
           << "    arc_fallback_count: " << route.arc_fallback_count << "\n"
           << "    anomaly_distance_to_a1: " << route.anomaly_distance_to_a1 << "\n"
           << "    max_anomaly_near_a1: "
           << (route.max_anomaly_near_a1 ? "true" : "false") << "\n"
           << "    cusp_s: [";
    for (std::size_t i = 0; i < route.cusp_s.size(); ++i) {
      if (i != 0) output << ", ";
      output << route.cusp_s[i];
    }
    output << "]\n    debug_layer_ranges:\n";
    for (const LayerRange& range : route.layer_ranges) {
      output << "      - {type: " << layerTypeName(range.type)
             << ", label: " << yamlQuote(range.label)
             << ", s_start: " << range.s_min
             << ", s_end: " << range.s_max << "}\n";
    }
  }
}

void printTop(const std::vector<RouteAudit>& all, const std::string& direction,
              bool by_rate, double kappa_limit) {
  std::vector<const RouteAudit*> selected;
  for (const RouteAudit& route : all) {
    if (route.direction == direction) selected.push_back(&route);
  }
  std::sort(selected.begin(), selected.end(),
            [by_rate](const RouteAudit* a, const RouteAudit* b) {
              return by_rate ? a->max_abs_dkappa_ds > b->max_abs_dkappa_ds
                             : a->max_abs_curvature > b->max_abs_curvature;
            });
  std::cout << "\n" << direction << " top 10 by "
            << (by_rate ? "max |dkappa/ds|" : "max |kappa|") << ":\n";
  for (std::size_t i = 0; i < std::min<std::size_t>(10, selected.size()); ++i) {
    const RouteAudit& route = *selected[i];
    std::cout << "  " << (i + 1) << ". B" << route.slot << " " << route.side
              << " |k|=" << route.max_abs_curvature
              << " |dk/ds|=" << route.max_abs_dkappa_ds
              << " rate_ratio=" << route.curvature_rate_ratio
              << " s=" << (by_rate ? route.max_abs_dkappa_ds_s
                                     : route.max_abs_curvature_s)
              << " layer=" << (by_rate ? route.max_rate_layer
                                         : route.max_curvature_layer)
              << (route.max_abs_curvature > kappa_limit ?
                  " CURVATURE_LIMIT_EXCEEDED" : "")
              << (route.generation_info.used_arc_fallback ? " ARC_FALLBACK" : "")
              << (route.max_anomaly_near_a1 ? " NEAR_A1" : "") << '\n';
  }
}

void printFlaggedSlots(const std::vector<RouteAudit>& routes,
                       const std::string& direction, bool curvature_limit) {
  std::cout << direction << (curvature_limit
      ? " CURVATURE_LIMIT_EXCEEDED slots: "
      : " local arc fallback slots: ");
  bool first = true;
  for (const RouteAudit& route : routes) {
    if (route.direction != direction) continue;
    const bool flagged = curvature_limit ? route.curvature_limit_exceeded
        : route.generation_info.used_arc_fallback;
    if (!flagged) continue;
    if (!first) std::cout << ',';
    std::cout << route.slot;
    first = false;
  }
  if (first) std::cout << "none";
  std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_curvature_audit");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  try {
    const MapParam map_param = MapParam::fromROSParam(nh);
    const PlannerParam planner_param = PlannerParam::fromROSParam(nh);
    MultiVehicleConfig config = MultiVehicleConfig::fromROSParam(nh);
    if (config.reject_shelf_collisions) {
      throw std::runtime_error(
          "reject_shelf_collisions=true cannot be audited without duplicating "
          "TaskAllocator's private slot-sweep exemption");
    }
    const ForkliftMap map(map_param);
    const Slot a1 = makeA1Slot(config);
    PathGenerator b_to_a1(map_param, planner_param,
                          PathGeneratorRouteMode::B_TO_A1);
    PathGenerator a1_to_b(map_param, planner_param,
                          PathGeneratorRouteMode::A1_TO_B);

    std::string output_dir;
    std::string catalog_path;
    private_nh.param<std::string>("output_dir", output_dir,
                                  std::string("forklift_planner/config"));
    private_nh.param<std::string>("catalog_file", catalog_path, std::string());
    const CatalogSelection catalog =
        readCatalogSelection(catalog_path, static_cast<int>(map.slots().size()));
    const std::string selection_source = catalog.usable
        ? "A1_CYCLE_CATALOG_PLUS_RUNTIME_VALIDATION"
        : "FORKLIFT_MAP_SLOTS_PLUS_RUNTIME_VALIDATION";
    if (catalog.file_exists && !catalog.usable) {
      ROS_WARN("Catalog exists but does not match current format/map; "
               "falling back to ForkliftMap slots: %s", catalog_path.c_str());
    } else if (!catalog.file_exists) {
      ROS_WARN("Configured runtime A1 catalog is absent; enumerating current "
               "ForkliftMap slots: %s", catalog_path.c_str());
    } else {
      ROS_INFO("Using catalog task IDs, with freshly generated paths and "
               "current runtime validation: %s", catalog_path.c_str());
    }

    const double kappa_limit =
        std::tan(map_param.max_steer_angle) / map_param.wheel_base;
    const double ramp_length = map_param.turn_ramp_len();
    const double reference_rate = kappa_limit / ramp_length;
    const double a1_near_radius =
        std::hypot(a1.cx - a1.pre_dock_x, a1.cy - a1.pre_dock_y) +
        2.0 * ramp_length;
    std::vector<RouteAudit> routes;
    std::map<std::string, int> rejected;
    // Production route generation currently emits extensive WARN diagnostics.
    // Keep this batch tool's terminal output focused; all generated geometry
    // and debug layers remain captured in YAML/CSV.
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Error);
    ros::console::notifyLoggerLevelsChanged();
    for (const Slot& slot : map.slots()) {
      if (catalog.usable && catalog.slots.count(slot.id) == 0) continue;
      PathGenerationInfo to_info;
      RoughPath to_path = b_to_a1.generate(slot, a1, &to_info);
      const std::string to_reject = validateGeneratedPath(
          to_path, to_info, map_param, config);
      if (to_reject == "NONE") {
        routes.push_back(auditRoute("B_TO_A1", slot, std::move(to_path),
                                   std::move(to_info), map_param, a1,
                                   kappa_limit, reference_rate, a1_near_radius));
      } else {
        ++rejected["B_TO_A1_" + to_reject];
      }

      PathGenerationInfo from_info;
      RoughPath from_path = a1_to_b.generate(a1, slot, &from_info);
      const std::string from_reject = validateGeneratedPath(
          from_path, from_info, map_param, config);
      if (from_reject == "NONE") {
        routes.push_back(auditRoute("A1_TO_B", slot, std::move(from_path),
                                   std::move(from_info), map_param, a1,
                                   kappa_limit, reference_rate, a1_near_radius));
      } else {
        ++rejected["A1_TO_B_" + from_reject];
      }
    }
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Info);
    ros::console::notifyLoggerLevelsChanged();

    if (routes.empty()) throw std::runtime_error("no legal A1 routes found");
    const std::string yaml_path = output_dir + "/a1_path_curvature_audit.yaml";
    const std::string b_csv = output_dir + "/b_to_a1_curvature_samples.csv";
    const std::string a_csv = output_dir + "/a1_to_b_curvature_samples.csv";
    writeYaml(yaml_path, map_param, kappa_limit, ramp_length, reference_rate,
              a1_near_radius, catalog, catalog_path, selection_source, routes,
              rejected);
    writeSampleCsv(b_csv, "B_TO_A1", routes);
    writeSampleCsv(a_csv, "A1_TO_B", routes);

    std::cout << std::fixed << std::setprecision(6)
              << "Path curvature audit parameters: wheel_base="
              << map_param.wheel_base << " steer_limit="
              << map_param.max_steer_angle << " kappa_limit=" << kappa_limit
              << " ramp_len=" << ramp_length
              << " reference_dkappa_ds=" << reference_rate << '\n';
    printTop(routes, "B_TO_A1", false, kappa_limit);
    printTop(routes, "B_TO_A1", true, kappa_limit);
    printTop(routes, "A1_TO_B", false, kappa_limit);
    printTop(routes, "A1_TO_B", true, kappa_limit);
    std::cout << '\n';
    printFlaggedSlots(routes, "B_TO_A1", true);
    printFlaggedSlots(routes, "A1_TO_B", true);
    printFlaggedSlots(routes, "B_TO_A1", false);
    printFlaggedSlots(routes, "A1_TO_B", false);
    std::cout << "\nOutputs:\n  " << yaml_path << "\n  " << b_csv
              << "\n  " << a_csv << '\n';
    return 0;
  } catch (const std::exception& error) {
    ROS_ERROR("Path curvature audit failed: %s", error.what());
    return 1;
  }
}
