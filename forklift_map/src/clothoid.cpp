#include "forklift_map/common/clothoid.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace forklift_geom {

namespace {
constexpr double kEps = 1e-6;
}  // namespace

double curvature_at(double s, double /*total_len*/, double ramp_len,
                    double hold_len, double peak_k) {
    if (s <= ramp_len) {
        return peak_k * s / std::max(ramp_len, kEps);
    }
    if (s <= ramp_len + hold_len) {
        return peak_k;
    }
    const double down_s = s - ramp_len - hold_len;
    return peak_k * std::max(0.0, 1.0 - down_s / std::max(ramp_len, kEps));
}

ClothoidTurn build_clothoid_turn(double signed_angle, double max_curvature,
                                 double rate_ramp_len, double ds) {
    ClothoidTurn curve;
    const double turn = std::abs(signed_angle);
    const double sign = (signed_angle >= 0.0) ? 1.0 : -1.0;
    if (turn < 1e-4 || max_curvature < kEps) return curve;

    const double sigma = max_curvature / std::max(rate_ramp_len, ds);
    double peak_k = max_curvature;
    double ramp_len = rate_ramp_len;
    double hold_len = 0.0;

    if (turn < max_curvature * rate_ramp_len) {
        peak_k = std::sqrt(std::max(turn * sigma, 0.0));
        ramp_len = peak_k / sigma;
    } else {
        hold_len = (turn - max_curvature * rate_ramp_len) / max_curvature;
    }

    ramp_len = std::max(ramp_len, ds);
    const double total_len = 2.0 * ramp_len + hold_len;
    const int steps = std::max(2, static_cast<int>(std::ceil(total_len / ds)));
    const double step = total_len / static_cast<double>(steps);

    CurvePt p{0.0, 0.0};
    double heading = 0.0;
    curve.pts.push_back(p);
    curve.headings.push_back(heading);
    for (int i = 0; i < steps; ++i) {
        const double s_mid = (i + 0.5) * step;
        const double k = sign * curvature_at(s_mid, total_len, ramp_len, hold_len, peak_k);
        heading += k * step;
        p.x += std::cos(heading) * step;
        p.y += std::sin(heading) * step;
        curve.pts.push_back(p);
        curve.headings.push_back(heading);
    }
    curve.headings.back() = signed_angle;

    if (std::abs(std::sin(signed_angle)) < 1e-5) return curve;

    const CurvePt& end = curve.pts.back();
    const double lambda = -end.y / std::sin(signed_angle);
    curve.t_in = end.x + lambda * std::cos(signed_angle);
    curve.t_out = -lambda;
    return curve;
}

ClothoidTurn fit_clothoid_turn(double signed_angle, double max_curvature,
                               double desired_ramp_len, double max_tangent,
                               double ds) {
    ClothoidTurn best = build_clothoid_turn(signed_angle, max_curvature,
                                            desired_ramp_len, ds);
    if (!best.pts.empty() &&
        std::max(best.t_in, best.t_out) <= max_tangent) {
        return best;
    }

    double lo = ds;
    double hi = std::max(desired_ramp_len, ds);
    for (int i = 0; i < 24; ++i) {
        const double mid = 0.5 * (lo + hi);
        ClothoidTurn candidate = build_clothoid_turn(signed_angle, max_curvature, mid, ds);
        if (!candidate.pts.empty() &&
            std::max(candidate.t_in, candidate.t_out) <= max_tangent) {
            lo = mid;
            best = std::move(candidate);
        } else {
            hi = mid;
        }
    }

    if (!best.pts.empty() &&
        std::max(best.t_in, best.t_out) <= max_tangent) {
        return best;
    }

    return {};
}

}  // namespace forklift_geom
