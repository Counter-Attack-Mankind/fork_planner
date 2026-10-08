#include "forklift_planner/path_generator_internal.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <ros/console.h>
#include <utility>

#include "forklift_map/common/clothoid.h"

namespace forklift_planner {
namespace path_internal {

using geometry2d::cross;
using geometry2d::dist;
using geometry2d::dot;
using geometry2d::left_normal;
using geometry2d::normalize;
using geometry2d::right_normal;

// 把公共库的曲率连续拐弯曲线转换成 planner 内部使用的 TurnCurve 类型。
static TurnCurve toTurnCurve(const forklift_geom::ClothoidTurn& c) {
    TurnCurve out;
    out.pts.reserve(c.pts.size());
    for (const auto& p : c.pts) out.pts.push_back(Pt{p.x, p.y});
    out.headings = c.headings;
    out.t_in = c.t_in;
    out.t_out = c.t_out;
    return out;
}

double corridor_center_y(const MapParam& mp, int corr_id) {
    switch (corr_id) {
        case 1: return mp.corridor1_cy();
        case 2: return mp.corridor2_cy();
        case 3: return mp.corridor3_cy();
        default: return mp.corridor4_cy();
    }
}

double corridor_lane_y(const MapParam& mp, int corr_id, HDir dir) {
    const double h_lane_off = std::min(0.14, mp.corridor_height * 0.22);
    const double cy = corridor_center_y(mp, corr_id);
    return (dir == HDir::LEFT) ? (cy + h_lane_off) : (cy - h_lane_off);
}

double corridor_min_y(const MapParam& mp, int corr_id) {
    switch (corr_id) {
        case 1: return mp.y7();
        case 2: return mp.y5();
        case 3: return mp.y3();
        default: return mp.y1();
    }
}

double corridor_max_y(const MapParam& mp, int corr_id) {
    switch (corr_id) {
        case 1: return mp.y8();
        case 2: return mp.y6();
        case 3: return mp.y4();
        default: return mp.y2();
    }
}

double dual_lane_offset(double aisle_width) {
    return aisle_width * 0.25;
}

bool point_inside_rect(const Pt& p, double x0, double y0, double w, double h) {
    constexpr double eps = 1e-5;
    return p.x > x0 + eps && p.x < x0 + w - eps &&
           p.y > y0 + eps && p.y < y0 + h - eps;
}

bool point_in_shelf(const MapParam& mp, const Pt& p) {
    const double tb_gap = mp.field_width - 2.0 * mp.tb_shelf_width;
    const double row2_right_x = mp.row2_left_width + mp.row2_gap;
    const double row3_right_x = mp.row3_left_shelf + mp.row3_center_aisle;
    return point_inside_rect(p, 0.0, mp.y8(), mp.tb_shelf_width,
                             mp.bottom_shelf_depth) ||
           point_inside_rect(p, mp.tb_shelf_width + tb_gap, mp.y8(),
                             mp.tb_shelf_width, mp.bottom_shelf_depth) ||
           point_inside_rect(p, mp.row1_left_aisle, mp.y6(),
                             mp.row1_shelf_width, mp.shelf_row_depth) ||
           point_inside_rect(p, mp.field_width - mp.row1_mini_shelf, mp.y6(),
                             mp.row1_mini_shelf, mp.shelf_row_depth) ||
           point_inside_rect(p, 0.0, mp.y4(), mp.row2_left_width,
                             mp.shelf_row_depth) ||
           point_inside_rect(p, row2_right_x, mp.y4(),
                             mp.field_width - row2_right_x,
                             mp.shelf_row_depth) ||
           point_inside_rect(p, 0.0, mp.y2(), mp.row3_left_shelf,
                             mp.shelf_row_depth) ||
           point_inside_rect(p, row3_right_x, mp.y2(),
                             mp.field_width - row3_right_x,
                             mp.shelf_row_depth) ||
           point_inside_rect(p, 0.0, 0.0, mp.tb_shelf_width,
                             mp.bottom_shelf_depth) ||
           point_inside_rect(p, mp.tb_shelf_width + tb_gap, 0.0,
                             mp.tb_shelf_width, mp.bottom_shelf_depth);
}

double normalize_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

void push_point(std::vector<Pt>& polyline, const Pt& p) {
    if (polyline.empty() || dist(polyline.back(), p) > 1e-4) {
        polyline.push_back(p);
    }
}

void push_sample(std::vector<Sample>& samples, const Pt& p, double theta) {
    if (!samples.empty() && dist(samples.back().p, p) <= 1e-4) {
        samples.back().theta = theta;
        return;
    }
    samples.push_back({p, theta});
}

std::vector<Pt> simplify_polyline(const std::vector<Pt>& in) {
    std::vector<Pt> out;
    for (const Pt& p : in) {
        push_point(out, p);
    }

    std::vector<Pt> simplified;
    for (const Pt& p : out) {
        simplified.push_back(p);
        while (simplified.size() >= 3) {
            const Pt& a = simplified[simplified.size() - 3];
            const Pt& b = simplified[simplified.size() - 2];
            const Pt& c = simplified[simplified.size() - 1];
            const Pt ab = b - a;
            const Pt bc = c - b;
            if (dist(a, b) < kEps || dist(b, c) < kEps) {
                simplified.erase(simplified.end() - 2);
                continue;
            }
            if (std::abs(cross(ab, bc)) < 1e-5 && dot(ab, bc) >= 0.0) {
                simplified.erase(simplified.end() - 2);
                continue;
            }
            break;
        }
    }
    return simplified;
}

void record_debug_layer(PathGenerationInfo* info,
                        DebugPathLayerType type,
                        const std::string& label,
                        const std::vector<Pt>& pts) {
    if (info == nullptr || pts.size() < 2) return;
    DebugPathLayer layer;
    layer.type = type;
    layer.label = label;
    layer.points.reserve(pts.size());
    for (const Pt& p : pts) {
        if (!layer.points.empty()) {
            const DebugPathPoint& prev = layer.points.back();
            if (std::hypot(prev.x - p.x, prev.y - p.y) <= 1e-5) continue;
        }
        layer.points.push_back({p.x, p.y});
    }
    if (layer.points.size() >= 2) {
        info->debug_layers.push_back(std::move(layer));
    }
}

void record_debug_layer_from_samples(PathGenerationInfo* info,
                                     DebugPathLayerType type,
                                     const std::string& label,
                                     const std::vector<Sample>& samples,
                                     size_t begin) {
    if (info == nullptr || begin >= samples.size()) return;
    std::vector<Pt> pts;
    pts.reserve(samples.size() - begin);
    for (size_t i = begin; i < samples.size(); ++i) {
        pts.push_back(samples[i].p);
    }
    record_debug_layer(info, type, label, pts);
}

void record_debug_layer_from_path(PathGenerationInfo* info,
                                  DebugPathLayerType type,
                                  const std::string& label,
                                  const RoughPath& path) {
    if (info == nullptr || path.size() < 2) return;
    DebugPathLayer layer;
    layer.type = type;
    layer.label = label;
    layer.points.reserve(path.size());
    for (const RoughWp& wp : path) {
        if (!layer.points.empty()) {
            const DebugPathPoint& prev = layer.points.back();
            if (std::hypot(prev.x - wp.x, prev.y - wp.y) <= 1e-5) continue;
        }
        layer.points.push_back({wp.x, wp.y});
    }
    if (layer.points.size() >= 2) {
        info->debug_layers.push_back(std::move(layer));
    }
}

double curvature_at(double s, double total_len, double ramp_len,
                    double hold_len, double peak_k) {
    return forklift_geom::curvature_at(s, total_len, ramp_len, hold_len, peak_k);
}

double quintic_blend(double t) {
    return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t);
}

double quintic_blend_derivative(double t) {
    return 30.0 * t * t * (1.0 - t) * (1.0 - t);
}

bool is_short_parallel_shift(const Pt& a, const Pt& b, const Pt& c,
                             const Pt& d, double max_shift) {
    const Pt u0 = normalize(b - a);
    const Pt um = normalize(c - b);
    const Pt u1 = normalize(d - c);
    const double shift_len = dist(b, c);
    return shift_len > kEps &&
           shift_len <= max_shift &&
           dot(u0, u1) > 0.98 &&
           std::abs(dot(u0, um)) < 0.2;
}

void append_lane_shift(std::vector<Sample>& out, const Pt& b, const Pt& c,
                       const Pt& direction, double lead_in, double lead_out,
                       double ds) {
    const Pt u = normalize(direction);
    const Pt n = left_normal(u);
    const Pt shift = c - b;
    const double lateral = dot(shift, n);
    const double length = std::max(lead_in + lead_out, ds);
    const Pt start = b - u * lead_in;
    const double base_heading = std::atan2(u.y, u.x);
    const int steps = std::max(4, static_cast<int>(std::ceil(length / ds)));

    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const double x = length * t;
        const double y = lateral * quintic_blend(t);
        const double dydx =
            lateral * quintic_blend_derivative(t) / std::max(length, kEps);
        const Pt p = start + u * x + n * y;
        push_sample(out, p, normalize_angle(base_heading + std::atan(dydx)));
    }
}

TurnCurve build_clothoid_turn(double signed_angle, double max_curvature,
                              double rate_ramp_len, double ds) {
    return toTurnCurve(forklift_geom::build_clothoid_turn(
        signed_angle, max_curvature, rate_ramp_len, ds));
}

Pt bezier5(const std::array<Pt, 6>& c, double t) {
    const double u = 1.0 - t;
    const double b0 = u * u * u * u * u;
    const double b1 = 5.0 * t * u * u * u * u;
    const double b2 = 10.0 * t * t * u * u * u;
    const double b3 = 10.0 * t * t * t * u * u;
    const double b4 = 5.0 * t * t * t * t * u;
    const double b5 = t * t * t * t * t;
    return {b0 * c[0].x + b1 * c[1].x + b2 * c[2].x +
            b3 * c[3].x + b4 * c[4].x + b5 * c[5].x,
            b0 * c[0].y + b1 * c[1].y + b2 * c[2].y +
            b3 * c[3].y + b4 * c[4].y + b5 * c[5].y};
}

Pt bezier5_derivative(const std::array<Pt, 6>& c, double t) {
    const double u = 1.0 - t;
    const double b0 = u * u * u * u;
    const double b1 = 4.0 * t * u * u * u;
    const double b2 = 6.0 * t * t * u * u;
    const double b3 = 4.0 * t * t * t * u;
    const double b4 = t * t * t * t;
    const Pt d0 = c[1] - c[0];
    const Pt d1 = c[2] - c[1];
    const Pt d2 = c[3] - c[2];
    const Pt d3 = c[4] - c[3];
    const Pt d4 = c[5] - c[4];
    return {(b0 * d0.x + b1 * d1.x + b2 * d2.x + b3 * d3.x + b4 * d4.x) * 5.0,
            (b0 * d0.y + b1 * d1.y + b2 * d2.y + b3 * d3.y + b4 * d4.y) * 5.0};
}

TurnCurve build_g2_spiral_turn(double signed_angle, double tangent_len,
                               double ds) {
    TurnCurve curve;
    const double turn = std::abs(signed_angle);
    if (turn < 1e-4 || tangent_len < 1e-4) return curve;

    const Pt dir0{1.0, 0.0};
    const Pt dir1{std::cos(signed_angle), std::sin(signed_angle)};
    const Pt p0{0.0, 0.0};
    const Pt corner{tangent_len, 0.0};
    const Pt p5 = corner + dir1 * tangent_len;
    const double chord = dist(p0, p5);
    const double ctrl = std::max(ds * 0.5, std::min(tangent_len * 0.55, chord / 3.0));

    std::array<Pt, 6> c{
        p0,
        p0 + dir0 * ctrl,
        p0 + dir0 * (2.0 * ctrl),
        p5 - dir1 * (2.0 * ctrl),
        p5 - dir1 * ctrl,
        p5
    };

    const int steps = std::max(4, static_cast<int>(std::ceil(chord / ds)));
    curve.pts.reserve(steps + 1);
    curve.headings.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        curve.pts.push_back(bezier5(c, t));
        const Pt d = bezier5_derivative(c, t);
        curve.headings.push_back(std::atan2(d.y, d.x));
    }
    curve.headings.front() = 0.0;
    curve.headings.back() = signed_angle;
    curve.t_in = tangent_len;
    curve.t_out = tangent_len;
    return curve;
}

TurnCurve fit_clothoid_turn(double signed_angle, double max_curvature,
                            double desired_ramp_len, double max_tangent,
                            double ds) {
    return toTurnCurve(forklift_geom::fit_clothoid_turn(
        signed_angle, max_curvature, desired_ramp_len, max_tangent, ds));
}

RoughPath build_arc_path(const std::vector<Pt>& simplified,
                         const PlannerParam& pp,
                         const Slot& src,
                         double max_curvature,
                         double sample_ds) {
    const double min_radius = 1.0 / std::max(max_curvature, kEps);
    std::vector<Pt> dense;
    dense.push_back(simplified.front());

    for (size_t i = 1; i + 1 < simplified.size(); ++i) {
        const Pt& a = simplified[i - 1];
        const Pt& b = simplified[i];
        const Pt& c = simplified[i + 1];

        const double len1 = dist(a, b);
        const double len2 = dist(b, c);
        if (len1 < kEps || len2 < kEps) continue;

        const Pt u1 = normalize(b - a);
        const Pt u2 = normalize(c - b);
        const double bend = cross(u1, u2);
        const double cos_angle = dot(u1, u2);

        if (std::abs(bend) < 1e-4 || std::abs(cos_angle) > 0.99) {
            push_point(dense, b);
            continue;
        }

        const double local_radius =
            std::min(min_radius, 0.45 * std::min(len1, len2));
        if (local_radius < 1e-3) {
            push_point(dense, b);
            continue;
        }

        const Pt t1 = b - u1 * local_radius;
        const Pt t2 = b + u2 * local_radius;
        push_point(dense, t1);

        const Pt normal = (bend > 0.0) ? left_normal(u1) : right_normal(u1);
        const Pt center = t1 + normal * local_radius;

        double a0 = std::atan2(t1.y - center.y, t1.x - center.x);
        double a1 = std::atan2(t2.y - center.y, t2.x - center.x);
        if (bend > 0.0) {
            while (a1 <= a0) a1 += 2.0 * kPi;
        } else {
            while (a1 >= a0) a1 -= 2.0 * kPi;
        }

        const double arc_angle = std::abs(a1 - a0);
        const double arc_len = arc_angle * local_radius;
        const int steps = std::max({2,
            static_cast<int>(std::ceil(arc_len / sample_ds)),
            static_cast<int>(std::ceil(arc_angle / 0.04))});
        for (int s = 1; s < steps; ++s) {
            const double ratio = static_cast<double>(s) / static_cast<double>(steps);
            const double ang = a0 + (a1 - a0) * ratio;
            push_point(dense, {center.x + local_radius * std::cos(ang),
                               center.y + local_radius * std::sin(ang)});
        }
        push_point(dense, t2);
    }

    push_point(dense, simplified.back());

    RoughPath path;
    path.reserve(dense.size());
    for (size_t i = 0; i < dense.size(); ++i) {
        double theta = 0.0;
        if (i + 1 < dense.size()) {
            theta = std::atan2(dense[i + 1].y - dense[i].y,
                               dense[i + 1].x - dense[i].x);
        } else if (!path.empty()) {
            theta = path.back().theta;
        } else {
            theta = normalize_angle(src.dock_theta + kPi);
        }
        path.push_back({dense[i].x, dense[i].y, theta, WpType::FORWARD});
    }
    (void)pp;
    return path;
}

void warn_if_reverse_segments(const RoughPath& path) {
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        const double len = std::hypot(dx, dy);
        if (len < 1e-5) continue;
        const double motion = std::atan2(dy, dx);
        const double heading_err =
            std::abs(normalize_angle(path[i].theta - motion));
        if (path[i + 1].type == WpType::REVERSE) {
            if (std::abs(heading_err - kPi) > 0.45) {
                ROS_WARN("[planner] reverse segment heading mismatch at i=%zu "
                         "pose=(%.3f, %.3f) theta=%.3f motion=%.3f err=%.3f",
                         i, path[i].x, path[i].y, path[i].theta,
                         motion, heading_err);
            }
        }
    }
}

}  // namespace path_internal
}  // namespace forklift_planner
