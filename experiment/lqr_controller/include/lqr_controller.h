//
// Created by yenkn on 2020/10/11.
//

#ifndef SRC_LQR_CONTROLLER_H
#define SRC_LQR_CONTROLLER_H

#include <chrono>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <planning_core/datatypes.h>
#include <planning_core/spline.h>
#include <planning_core/frenet_frame.h>
#include <nav_msgs/Path.h>


using namespace planning_core;
using namespace std::chrono;

class LQRController {
public:
  typedef Eigen::Matrix<double, 4, 4> MatrixQ, MatrixA;
  typedef Eigen::Matrix<double, 1, 1> MatrixR;
  typedef Eigen::Matrix<double, 4, 1> MatrixB;
  typedef Eigen::Matrix<double, 1, 4> MatrixK;
  typedef Eigen::Matrix<double, 4, 1> MatrixX;

  struct State {
    double x;
    double y;
    double yaw;
    double velocity;
  };

  LQRController(double dt, double wheel_base): dt_(dt), wheel_base_(wheel_base), node_("~") {
    debug_publisher_ = node_.advertise<geometry_msgs::Vector3>("/lqr_debug", 1, false);
  }

  inline void update_state(const State &state) { state_ = state; }

  void set_start(const TrajectoryPoint &goal);

  void update_trajectory(const Trajectory &traj);

  inline void set_ground_truth(const TrajectoryPoint &pt) { ground_truth_ = pt; }

  inline void set_q(const MatrixQ &q_matrix) { q_matrix_ = q_matrix; }

  inline void set_r(const MatrixR &r_matrix) { r_matrix_ = r_matrix; }

  inline void set_longitudial_pi(double kp, double ki) { longitude_kp_= kp; longitude_ki_ = ki; }

  ChassisCommand control();

private:
  MatrixQ q_matrix_ = MatrixQ::Identity();
  MatrixR r_matrix_ = MatrixR::Identity();
  double dt_, wheel_base_;
  double longitude_kp_ = 1.0, longitude_ki_ = 0.2;
  State state_;
  Trajectory trajectory_;
  TrajectoryPoint ground_truth_, goal_;
  FrenetFrame frame_, ref_frame_;
  ros::NodeHandle node_;

  ros::Publisher debug_publisher_;

  bool started_ = false, ref_received_ = false;

  static MatrixQ solve_dare(const MatrixA &A, const MatrixB &B, const MatrixQ &Q, const MatrixR &R);
  static MatrixK discrete_lqr(const MatrixA &A, const MatrixB &B, const MatrixQ &Q, const MatrixR &R);
  double longitude_control();
};

#endif //SRC_LQR_CONTROLLER_H