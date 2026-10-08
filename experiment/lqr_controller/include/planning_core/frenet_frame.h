//
// Created by yenkn on 20-7-27.
//

#pragma once

#include <vector>
#include "spline.h"
#include "numpy.h"

namespace planning_core {
class FrenetFrame {
public:
  FrenetFrame() = default;

  explicit FrenetFrame(const std::vector<double> &xs, const std::vector <double> &ys) {
    set_points(xs, ys);
  }

  void set_points(const std::vector<double> &xs, const std::vector <double> &ys) {
    xs_ = xs;
    ys_ = ys;
    calc_station();
    sx_.set_points(s_, xs_, true, false);
    sy_.set_points(s_, ys_, true, false);
  }

  inline double get_length() const { return s_.back(); }

  double calc_yaw(double s) const;

  double calc_curvature(double s) const;

  /**
   * use newton method to find optimized s
   */
  double find_s(double x, double y, double s0 = 0.0) const;

  double iterative_find_s(double x, double y, double resolution = 0.01) const;

  std::pair<double, double> to_frenet(double x, double y, double s0 = 0.0) const;

  std::pair<double, double> to_cartesian(double s, double lat) const;

private:
  std::vector<double> s_, xs_, ys_;
  spline sx_;
  spline sy_;

  void calc_station();
};
}