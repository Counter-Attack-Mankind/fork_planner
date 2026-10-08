//
// Created by yenkn on 2020/10/11.
//

#include "lqr_controller.h"
#include <iostream>
#include <planning_core/math_utils.h>

void LQRController::update_trajectory(const Trajectory &traj) {
  trajectory_ = traj;

  std::vector<double> xs(traj.size()), ys(traj.size());
  for(int i = 0; i < traj.size(); i++) {
    xs[i] = traj[i].x;
    ys[i] = traj[i].y;
  }

  frame_.set_points(xs, ys);
}

LQRController::MatrixQ LQRController::solve_dare(const MatrixA &A, const MatrixB &B, const MatrixQ &Q, const MatrixR &R) {
  MatrixQ x = Q;
  MatrixQ x_next = Q;
  int max_iter = 150;
  double eps = 0.01;

  for(int i = 0; i < max_iter; i++) {
    x_next = A.transpose() * x * A - A.transpose() * x * B *
                                     (R + B.transpose() * x * B).inverse() * B.transpose() * x * A + Q;
    if ((x_next - x).cwiseAbs().maxCoeff() < eps) {
      break;
    }
    x = x_next;
  }

  return x_next;
}

LQRController::MatrixK LQRController::discrete_lqr(const MatrixA &A, const MatrixB &B, const MatrixQ &Q, const MatrixR &R) {
  // first, try to solve the ricatti equation
  MatrixQ X = solve_dare(A, B, Q, R);

  // compute the LQR gain
  MatrixK K = (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
  // auto eig_result = (A - B * K).eigenvalues();
  return K;
}

ChassisCommand LQRController::control() {
  if(hypot(state_.x - goal_.x, state_.y - goal_.y) < 0.03) {
    return { 0.0, 0.0 };
  }
  double station = frame_.iterative_find_s(state_.x, state_.y);
  auto waypoint = frame_.to_cartesian(station + 0.1, 0.0);
  double yaw = frame_.calc_yaw(station);
  double dxl = waypoint.first - state_.x;
  double dyl = waypoint.second - state_.y;
  double angle = normalize_angle(yaw - atan2(dyl, dxl));

  MatrixA A = MatrixA::Zero();
  A(0, 0) = 1.0;
  A(0, 1) = dt_;
  A(1, 2) = state_.velocity;
  A(2, 2) = 1.0;
  A(2, 3) = dt_;
  // A(4, 4) = 1.0;

  MatrixB B = MatrixB::Zero();
  B(3, 0) = state_.velocity / wheel_base_;

  MatrixK K = discrete_lqr(A, B, q_matrix_, r_matrix_);

  // state vector
  // x = [e, dot_e, th_e, dot_th_e]
  // e: lateral distance to the path
  // dot_e: derivative of e
  // dot_th_e: derivative of th_e
  MatrixX x = MatrixX::Zero();
  double lateral_error = hypot(waypoint.first - state_.x, waypoint.second - state_.y);
  if(angle < 0) {
    lateral_error *= -1;
  }
  static double previous_lateral_error = lateral_error;
  double orientation_error = normalize_angle(state_.yaw - yaw);
  static double previous_orientation_error = orientation_error;

  x(0, 0) = lateral_error;
  x(1, 0) = (lateral_error - previous_lateral_error) / dt_;
  x(2, 0) = orientation_error;
  x(3, 0) = (orientation_error - previous_orientation_error) / dt_;
  // x(4, 0) = state_.velocity - target_speed;

  previous_lateral_error = lateral_error;
  previous_orientation_error = orientation_error;

  // input vector
  // u = [delta, accel]
  // delta: steering angle
  auto ustar = -K * x;

  // calc steering input
  double ref_s = frame_.iterative_find_s(state_.x, state_.y);
  double feedforward_delta = atan2(wheel_base_ * frame_.calc_curvature(ref_s), 1); // feedforward steering angle
  double feedback_delta = normalize_angle(ustar(0, 0));  // feedback steering angle
  double delta = 0.0 * feedforward_delta + 1.0 * feedback_delta;

  geometry_msgs::Vector3 msg;
  msg.x = state_.yaw;
  msg.y = yaw;
  msg.z = feedforward_delta;
  debug_publisher_.publish(msg);
  ROS_INFO("ref %f, vel %f, LE: %f, OE: %f, FF: %f, FB: %f, DE: %f", ref_s, state_.velocity, lateral_error, orientation_error, feedforward_delta, feedback_delta, delta);

  static double last_delta = 0.0;
//  if(fabs(delta - last_delta) > M_PI_4) {
//    ROS_WARN("delta angle changes too much, rejected.");
//    delta = last_delta;
//  }

  ChassisCommand command = {
      .throttle = longitude_control(),
      .steering = delta,
  };

  last_delta = delta;

  return command;
}

void LQRController::set_start(const TrajectoryPoint &goal) {
  goal_ = goal;
  started_ = true;
}

double LQRController::longitude_control() {
  static double output = 0.0, last_error = 0.0;

  double relative_x = cos(ground_truth_.theta) * (ground_truth_.x - state_.x) + sin(ground_truth_.theta) * (ground_truth_.y - state_.y);

  output += longitude_kp_ * (relative_x - last_error) + longitude_ki_ * relative_x;
  last_error = relative_x;
  output = std::min(std::max(output, -0.6), 0.6);

  return output;
}