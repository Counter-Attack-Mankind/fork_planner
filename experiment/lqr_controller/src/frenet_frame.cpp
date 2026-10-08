//
// Created by yenkn on 20-7-27.
//

#include <planning_core/frenet_frame.h>
#include <planning_core/math_utils.h>

namespace planning_core {
void FrenetFrame::calc_station() {
  std::vector<double> xs_real, ys_real, s_real;
  s_real.reserve(s_.size());
  xs_real.reserve(xs_.size());
  ys_real.reserve(ys_.size());

  double cum = 0.0;
  for(int i = 1; i < xs_.size(); i++) {
    double distance = hypot(xs_[i] - xs_[i-1], ys_[i] - ys_[i-1]);
    if(distance == 0) continue;
    cum += distance;
    s_real.push_back(cum);
    xs_real.push_back(xs_[i]);
    ys_real.push_back(ys_[i]);
  }

  s_ = s_real;
  xs_ = xs_real;
  ys_ = ys_real;
}

double FrenetFrame::calc_yaw(double s) const {
  return normalize_angle(atan2(sy_.deriv(1, s), sx_.deriv(1, s)));
}

double FrenetFrame::calc_curvature(double s) const {
  double dx = sx_.deriv(1, s), dy = sy_.deriv(1, s);

  return (dx * sy_.deriv(2, s) - sx_.deriv(2, s) * dy) / pow(dx * dx + dy * dy, 1.5);
}

double FrenetFrame::iterative_find_s(double x, double y, double resolution) const {
  double count = s_.back() / resolution;
  double min_distance = std::numeric_limits<double>::max();
  int margin = int(0 / resolution);
  int min_index = -1;
  for(int i = -margin; i < count + margin; i++) {
    double distance = hypot(sx_(i * resolution) - x, sy_(i * resolution) - y);
    if(distance < min_distance) {
      min_distance = distance;
      min_index = i;
    }
  }
  return min_index * resolution;
}

double FrenetFrame::find_s(double x, double y, double s0) const {
  double opt = s0;
  double opt_last = opt;
  for(int i = 0; i < 20; i++) {
    double ox = sx_(opt), oy = sy_(opt);
    double dx = sx_.deriv(1, opt), dy = sy_.deriv(1, opt);
    double ddx = sx_.deriv(2, opt), ddy = sy_.deriv(2, opt);
    double error_x = ox - x, error_y = oy - y;
    double jac = 2.0 * error_x * dx + 2.0 * error_y * dy;
    double hessian = 2.0 * dx * dx + 2.0 * error_x * ddx + 2.0 * dy * dy + 2.0 * error_y * ddy;

    opt -= jac / hessian;

    if (fabs(opt - opt_last) < 1e-5)
      return opt;
    opt_last = opt;
  }

  return s0;
}

std::pair<double, double> FrenetFrame::to_frenet(double x, double y, double s0) const {
  if(s0 == 0.0) {
    s0 = iterative_find_s(x, y, 1.0);
  }
  auto s = find_s(x, y, s0);
  double theta = calc_yaw(s);
  double dx = x - sx_(s), dy = y - sy_(s);
  double norm = cos(theta) * dy - sin(theta) * dx;
  double lat = copysign(hypot(dx, dy), -norm);

  return std::make_pair(s, lat);
}

std::pair<double, double> FrenetFrame::to_cartesian(double s, double lat) const {
  auto yaw = calc_yaw(s);
  double x = sx_(s) - lat * cos(M_PI_2 + yaw);
  double y = sy_(s) - lat * sin(M_PI_2 + yaw);
  return { x, y };
}
}