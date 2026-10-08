#pragma once

#include <cmath>

namespace forklift_planner {
namespace geometry2d {

constexpr double kGeomEps = 1e-6;

struct Pt {
    double x = 0.0;
    double y = 0.0;
};

inline Pt operator+(const Pt& a, const Pt& b) {
    return {a.x + b.x, a.y + b.y};
}

inline Pt operator-(const Pt& a, const Pt& b) {
    return {a.x - b.x, a.y - b.y};
}

inline Pt operator*(const Pt& a, double s) {
    return {a.x * s, a.y * s};
}

inline double dot(const Pt& a, const Pt& b) {
    return a.x * b.x + a.y * b.y;
}

inline double cross(const Pt& a, const Pt& b) {
    return a.x * b.y - a.y * b.x;
}

inline double dist(const Pt& a, const Pt& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

inline Pt normalize(const Pt& v) {
    const double n = std::hypot(v.x, v.y);
    if (n < kGeomEps) return {0.0, 0.0};
    return {v.x / n, v.y / n};
}

inline Pt left_normal(const Pt& v) {
    return {-v.y, v.x};
}

inline Pt right_normal(const Pt& v) {
    return {v.y, -v.x};
}

inline Pt rotate_to_heading(const Pt& p, double heading) {
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    return {c * p.x - s * p.y, s * p.x + c * p.y};
}

}  // namespace geometry2d
}  // namespace forklift_planner
