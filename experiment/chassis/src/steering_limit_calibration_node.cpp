#include <ros/ros.h>
#include <sandbox_msgs/AprilObject.h>
#include <sandbox_msgs/ChassisCommand.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::atomic<bool> g_stop_requested(false);

void sigintHandler(int) {
  g_stop_requested.store(true);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

bool finitePose(double x, double y, double yaw) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
}

struct PoseSample {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  PoseSample() = default;
  PoseSample(double x_in, double y_in, double yaw_in)
      : x(x_in), y(y_in), yaw(yaw_in) {}
};

struct CircleFit {
  bool valid = false;
  double cx = 0.0;
  double cy = 0.0;
  double radius = 0.0;
  double rms = 0.0;
};

bool solve3x3(double a[3][4], double solution[3]) {
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
    }
    if (std::fabs(a[pivot][col]) < 1e-10) return false;
    if (pivot != col) {
      for (int k = col; k < 4; ++k) std::swap(a[col][k], a[pivot][k]);
    }
    const double divisor = a[col][col];
    for (int k = col; k < 4; ++k) a[col][k] /= divisor;
    for (int row = 0; row < 3; ++row) {
      if (row == col) continue;
      const double factor = a[row][col];
      for (int k = col; k < 4; ++k) a[row][k] -= factor * a[col][k];
    }
  }
  for (int i = 0; i < 3; ++i) solution[i] = a[i][3];
  return true;
}

CircleFit fitCircle(const std::vector<PoseSample>& samples,
                    std::size_t begin, std::size_t end) {
  CircleFit result;
  if (end <= begin || end - begin < 3) return result;

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    mean_x += samples[i].x;
    mean_y += samples[i].y;
  }
  const double count = static_cast<double>(end - begin);
  mean_x /= count;
  mean_y /= count;

  double suu = 0.0, suv = 0.0, svv = 0.0;
  double su = 0.0, sv = 0.0, suz = 0.0, svz = 0.0, sz = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    const double u = samples[i].x - mean_x;
    const double v = samples[i].y - mean_y;
    const double z = u * u + v * v;
    suu += u * u;
    suv += u * v;
    svv += v * v;
    su += u;
    sv += v;
    suz += u * z;
    svz += v * z;
    sz += z;
  }
  double system[3][4] = {
      {suu, suv, su, suz},
      {suv, svv, sv, svz},
      {su, sv, count, sz}};
  double solution[3] = {0.0, 0.0, 0.0};
  if (!solve3x3(system, solution)) return result;

  const double radius_sq = solution[2] +
      0.25 * (solution[0] * solution[0] + solution[1] * solution[1]);
  if (!(radius_sq > 0.0) || !std::isfinite(radius_sq)) return result;
  result.cx = mean_x + 0.5 * solution[0];
  result.cy = mean_y + 0.5 * solution[1];
  result.radius = std::sqrt(radius_sq);
  double squared_error = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    const double radial_error =
        std::hypot(samples[i].x - result.cx, samples[i].y - result.cy) -
        result.radius;
    squared_error += radial_error * radial_error;
  }
  result.rms = std::sqrt(squared_error / count);
  result.valid = std::isfinite(result.cx) && std::isfinite(result.cy) &&
                 std::isfinite(result.radius) && std::isfinite(result.rms);
  return result;
}

std::string vehicleLabel(int target) {
  return std::string("V") + std::to_string(target);
}

}  // namespace

class SteeringLimitCalibration {
 public:
  SteeringLimitCalibration()
      : nh_(), pnh_("~"), started_at_(ros::WallTime::now()) {
    loadParameters();
    validateParameters();

    const std::string stem = log_dir_ + "/steering_calibration_" +
        vehicleLabel(target_) + "_" + direction_;
    tracking_.open((stem + "_tracking.csv").c_str(), std::ios::out | std::ios::trunc);
    steps_.open((stem + "_steps.csv").c_str(), std::ios::out | std::ios::trunc);
    report_path_ = stem + "_report.txt";
    if (!tracking_.is_open() || !steps_.is_open()) {
      ROS_FATAL("Cannot open calibration logs under '%s'", log_dir_.c_str());
      throw std::runtime_error("failed to open calibration logs");
    }
    tracking_ << "wall_time,target,direction,stage,command_throttle,"
                 "command_steering,actual_x,actual_y,actual_yaw,sample_valid,"
                 "boundary_distance\n";
    steps_ << "target,direction,step_index,command_steering,fitted_radius,"
              "kappa_real,equivalent_steer,steering_error,rms_fit_error,"
              "arc_angle,travel_distance,sample_count,result,reason\n";

    command_pub_ = nh_.advertise<sandbox_msgs::ChassisCommand>("/chassis", 10, false);
    object_sub_ = nh_.subscribe("/object", 20,
                                &SteeringLimitCalibration::objectCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1),
                             &SteeringLimitCalibration::timerCallback, this);
    ROS_WARN("[%s %s] Calibration armed; waiting for valid /object pose. "
             "No command will be sent before pose validation.",
             vehicleLabel(target_).c_str(), direction_.c_str());
  }

  ~SteeringLimitCalibration() {
    if (!finalized_) finish("USER_ABORT", false, false);
  }

 private:
  enum class Stage { WAITING_FOR_POSE, SETTLING, MEASURING, STOPPING, DONE };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber object_sub_;
  ros::Publisher command_pub_;
  ros::Timer timer_;
  Stage stage_ = Stage::WAITING_FOR_POSE;

  int target_ = -1;
  double throttle_ = 0.05;
  double steering_start_ = 0.30;
  double steering_step_ = 0.03;
  double steering_max_test_ = 0.65;
  std::string direction_;
  double direction_sign_ = 1.0;
  double settle_time_ = 2.0;
  double measure_time_ = 5.0;
  double wheel_base_ = 0.143;
  double safe_min_x_ = 0.0, safe_max_x_ = 0.0;
  double safe_min_y_ = 0.0, safe_max_y_ = 0.0;
  double boundary_margin_ = 0.18;
  std::string log_dir_;

  int min_samples_ = 30;
  double min_travel_distance_ = 0.15;
  double min_arc_angle_ = 0.15;
  double max_fit_rms_ = 0.02;
  double max_radius_variation_ratio_ = 0.20;
  double min_effective_increment_ = 0.01;
  double object_timeout_ = 0.30;
  double max_pose_jump_ = 0.25;
  double max_total_time_ = 180.0;
  int stop_frame_count_ = 10;

  bool have_pose_ = false;
  bool finalized_ = false;
  bool boundary_stop_ = false;
  bool saturation_stop_ = false;
  bool step_recorded_ = true;
  std::uint64_t pose_sequence_ = 0;
  std::uint64_t sampled_pose_sequence_ = 0;
  PoseSample pose_;
  PoseSample previous_pose_;
  ros::WallTime last_pose_time_;
  ros::WallTime started_at_;
  ros::WallTime stage_started_at_;
  std::vector<PoseSample> samples_;
  int step_index_ = -1;
  double command_steering_ = 0.0;
  double maximum_tested_command_ = 0.0;
  bool have_pass_ = false;
  double last_pass_command_ = 0.0;
  double last_pass_equivalent_ = 0.0;
  double last_pass_curvature_ = 0.0;
  double last_pass_radius_ = 0.0;
  std::string stop_reason_ = "UNKNOWN";
  std::ofstream tracking_;
  std::ofstream steps_;
  std::string report_path_;

  void loadParameters() {
    if (!pnh_.getParam("target", target_)) {
      ROS_FATAL("Required parameter '~target' is missing");
      throw std::runtime_error("missing target");
    }
    if (!pnh_.getParam("direction", direction_)) {
      ROS_FATAL("Required parameter '~direction' is missing");
      throw std::runtime_error("missing direction");
    }
    pnh_.param("throttle", throttle_, throttle_);
    pnh_.param("steering_start", steering_start_, steering_start_);
    pnh_.param("steering_step", steering_step_, steering_step_);
    pnh_.param("steering_max_test", steering_max_test_, steering_max_test_);
    pnh_.param("settle_time", settle_time_, settle_time_);
    pnh_.param("measure_time", measure_time_, measure_time_);
    if (!pnh_.getParam("wheel_base", wheel_base_)) {
      nh_.param("/forklift_planner/wheel_base", wheel_base_, wheel_base_);
    }
    if (!pnh_.getParam("safe_min_x", safe_min_x_) ||
        !pnh_.getParam("safe_max_x", safe_max_x_) ||
        !pnh_.getParam("safe_min_y", safe_min_y_) ||
        !pnh_.getParam("safe_max_y", safe_max_y_)) {
      ROS_FATAL("safe_min_x/safe_max_x/safe_min_y/safe_max_y are required");
      throw std::runtime_error("missing safety boundary");
    }
    pnh_.param("boundary_margin", boundary_margin_, boundary_margin_);
    pnh_.param<std::string>("log_dir", log_dir_, std::string("src/log"));
    pnh_.param("min_samples", min_samples_, min_samples_);
    pnh_.param("min_travel_distance", min_travel_distance_, min_travel_distance_);
    pnh_.param("min_arc_angle", min_arc_angle_, min_arc_angle_);
    pnh_.param("max_fit_rms", max_fit_rms_, max_fit_rms_);
    pnh_.param("max_radius_variation_ratio", max_radius_variation_ratio_,
               max_radius_variation_ratio_);
    pnh_.param("min_effective_increment", min_effective_increment_,
               min_effective_increment_);
    pnh_.param("object_timeout", object_timeout_, object_timeout_);
    pnh_.param("max_pose_jump", max_pose_jump_, max_pose_jump_);
    pnh_.param("max_total_time", max_total_time_, max_total_time_);
    pnh_.param("stop_frame_count", stop_frame_count_, stop_frame_count_);
  }

  void validateParameters() {
    if (target_ < 0 || target_ > 7) throw std::runtime_error("target must be 0..7");
    if (direction_ == "left") direction_sign_ = 1.0;
    else if (direction_ == "right") direction_sign_ = -1.0;
    else throw std::runtime_error("direction must be left or right");
    if (!(throttle_ > 0.0) || !(steering_start_ > 0.0) ||
        !(steering_step_ > 0.0) || steering_max_test_ < steering_start_ ||
        !(settle_time_ >= 0.0) || !(measure_time_ > 0.0) ||
        !(wheel_base_ > 0.0) || !(safe_min_x_ < safe_max_x_) ||
        !(safe_min_y_ < safe_max_y_) || !(boundary_margin_ >= 0.0) ||
        safe_min_x_ + 2.0 * boundary_margin_ >= safe_max_x_ ||
        safe_min_y_ + 2.0 * boundary_margin_ >= safe_max_y_) {
      throw std::runtime_error("invalid calibration or safety parameters");
    }
  }

  std::string stageName() const {
    switch (stage_) {
      case Stage::WAITING_FOR_POSE: return "WAITING_FOR_POSE";
      case Stage::SETTLING: return "SETTLING";
      case Stage::MEASURING: return "MEASURING";
      case Stage::STOPPING: return "STOPPING";
      case Stage::DONE: return "DONE";
    }
    return "UNKNOWN";
  }

  double boundaryDistance(const PoseSample& pose) const {
    return std::min(std::min(pose.x - safe_min_x_, safe_max_x_ - pose.x),
                    std::min(pose.y - safe_min_y_, safe_max_y_ - pose.y));
  }

  bool insideSafeInterior(const PoseSample& pose) const {
    return pose.x > safe_min_x_ + boundary_margin_ &&
           pose.x < safe_max_x_ - boundary_margin_ &&
           pose.y > safe_min_y_ + boundary_margin_ &&
           pose.y < safe_max_y_ - boundary_margin_;
  }

  void objectCallback(const sandbox_msgs::AprilObjectConstPtr& msg) {
    if (msg->type != sandbox_msgs::AprilObject::VEHICLE || msg->id != target_) return;
    const ros::WallTime now = ros::WallTime::now();
    if (!finitePose(msg->x, msg->y, msg->yaw)) {
      if (stage_ == Stage::WAITING_FOR_POSE) have_pose_ = false;
      if (stage_ != Stage::WAITING_FOR_POSE && stage_ != Stage::DONE)
        finish("OBJECT_INVALID", false, false);
      return;
    }
    PoseSample next{msg->x, msg->y, msg->yaw};
    if (have_pose_ && stage_ != Stage::WAITING_FOR_POSE &&
        std::hypot(next.x - pose_.x, next.y - pose_.y) > max_pose_jump_) {
      finish("POSE_JUMP", false, false);
      return;
    }
    previous_pose_ = pose_;
    pose_ = next;
    have_pose_ = true;
    ++pose_sequence_;
    last_pose_time_ = now;

    if (!insideSafeInterior(pose_) && stage_ != Stage::WAITING_FOR_POSE &&
        stage_ != Stage::DONE) {
      boundary_stop_ = true;
      finish("BOUNDARY_LIMIT", false, true);
    }
  }

  void publishCommand(double throttle, double steering) {
    sandbox_msgs::ChassisCommand command;
    command.target = target_;
    command.throttle = throttle;
    command.steering = steering;
    command_pub_.publish(command);
  }

  void publishStopFrames() {
    for (int i = 0; i < std::max(1, stop_frame_count_); ++i) {
      publishCommand(0.0, 0.0);
      ros::WallDuration(0.02).sleep();
    }
  }

  bool predictedArcIsSafe(double steering_abs, double radius_hint) const {
    double radius = radius_hint;
    if (!(radius > 0.0) || !std::isfinite(radius)) {
      radius = wheel_base_ / std::tan(std::min(steering_abs, 1.45));
    }
    if (!(radius > 0.0) || !std::isfinite(radius)) return false;
    const double signed_turn = direction_sign_;
    const double cx = pose_.x - signed_turn * radius * std::sin(pose_.yaw);
    const double cy = pose_.y + signed_turn * radius * std::cos(pose_.yaw);
    const double start_angle = std::atan2(pose_.y - cy, pose_.x - cx);
    const double arc = std::min(2.0 * M_PI,
        throttle_ * (settle_time_ + measure_time_) / radius);
    for (int i = 0; i <= 40; ++i) {
      const double angle = start_angle + signed_turn * arc * i / 40.0;
      PoseSample predicted;
      predicted.x = cx + radius * std::cos(angle);
      predicted.y = cy + radius * std::sin(angle);
      if (!insideSafeInterior(predicted)) return false;
    }
    return true;
  }

  void beginStep(double steering_abs) {
    const double radius_hint = have_pass_ ? last_pass_radius_ : 0.0;
    if (!predictedArcIsSafe(steering_abs, radius_hint)) {
      boundary_stop_ = true;
      finish("BOUNDARY_LIMIT", false, true);
      return;
    }
    ++step_index_;
    step_recorded_ = false;
    command_steering_ = direction_sign_ * steering_abs;
    maximum_tested_command_ = command_steering_;
    samples_.clear();
    sampled_pose_sequence_ = pose_sequence_;
    stage_ = Stage::SETTLING;
    stage_started_at_ = ros::WallTime::now();
    ROS_INFO("[%s %s] step %d: command steering %.3f rad",
             vehicleLabel(target_).c_str(), direction_.c_str(), step_index_,
             command_steering_);
    publishCommand(throttle_, command_steering_);
  }

  void logTracking(bool sample_valid) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    tracking_ << std::fixed << std::setprecision(6)
              << ros::WallTime::now().toSec() << ',' << target_ << ','
              << direction_ << ',' << stageName() << ','
              << ((stage_ == Stage::SETTLING || stage_ == Stage::MEASURING)
                      ? throttle_ : 0.0) << ','
              << ((stage_ == Stage::SETTLING || stage_ == Stage::MEASURING)
                      ? command_steering_ : 0.0) << ','
              << (have_pose_ ? pose_.x : nan) << ','
              << (have_pose_ ? pose_.y : nan) << ','
              << (have_pose_ ? pose_.yaw : nan) << ','
              << (sample_valid ? 1 : 0) << ','
              << (have_pose_ ? boundaryDistance(pose_) : nan) << '\n';
    tracking_.flush();
  }

  void finishStep() {
    const CircleFit fit = fitCircle(samples_, 0, samples_.size());
    double travel = 0.0;
    double yaw_change = 0.0;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
      travel += std::hypot(samples_[i].x - samples_[i - 1].x,
                           samples_[i].y - samples_[i - 1].y);
      yaw_change += normalizeAngle(samples_[i].yaw - samples_[i - 1].yaw);
    }
    double arc_angle = 0.0;
    if (fit.valid) {
      for (std::size_t i = 1; i < samples_.size(); ++i) {
        const double a0 = std::atan2(samples_[i - 1].y - fit.cy,
                                     samples_[i - 1].x - fit.cx);
        const double a1 = std::atan2(samples_[i].y - fit.cy,
                                     samples_[i].x - fit.cx);
        arc_angle += std::fabs(normalizeAngle(a1 - a0));
      }
    }
    const double curvature = fit.valid ? 1.0 / fit.radius : 0.0;
    const double equivalent = fit.valid
        ? direction_sign_ * std::atan(wheel_base_ / fit.radius) : 0.0;
    const double steering_error = command_steering_ - equivalent;

    std::string result = "PASS";
    std::string reason = "OK";
    if (static_cast<int>(samples_.size()) < min_samples_) {
      result = "FAIL"; reason = "INSUFFICIENT_SAMPLES";
    } else if (!fit.valid) {
      result = "FAIL"; reason = "CIRCLE_FIT_FAILED";
    } else if (travel < min_travel_distance_) {
      result = "FAIL"; reason = "INSUFFICIENT_TRAVEL";
    } else if (arc_angle < min_arc_angle_) {
      result = "FAIL"; reason = "INSUFFICIENT_ARC";
    } else if (fit.rms > max_fit_rms_) {
      result = "FAIL"; reason = "FIT_RMS_HIGH";
    } else if (direction_sign_ * yaw_change <= 0.0) {
      result = "FAIL"; reason = "YAW_DIRECTION_MISMATCH";
    } else {
      const std::size_t middle = samples_.size() / 2;
      const CircleFit first = fitCircle(samples_, 0, middle);
      const CircleFit second = fitCircle(samples_, middle, samples_.size());
      const double variation = (first.valid && second.valid)
          ? std::fabs(first.radius - second.radius) / fit.radius
          : std::numeric_limits<double>::infinity();
      if (variation > max_radius_variation_ratio_) {
        result = "FAIL"; reason = "RADIUS_UNSTABLE";
      } else if (have_pass_ &&
                 std::fabs(equivalent) - std::fabs(last_pass_equivalent_) <
                     min_effective_increment_) {
        result = "FAIL"; reason = "STEERING_SATURATION";
        saturation_stop_ = true;
      }
    }

    steps_ << std::fixed << std::setprecision(6)
           << target_ << ',' << direction_ << ',' << step_index_ << ','
           << command_steering_ << ','
           << (fit.valid ? fit.radius : 0.0) << ',' << curvature << ','
           << equivalent << ',' << steering_error << ','
           << (fit.valid ? fit.rms : 0.0) << ',' << arc_angle << ','
           << travel << ',' << samples_.size() << ',' << result << ','
           << reason << '\n';
    steps_.flush();
    step_recorded_ = true;

    if (result != "PASS") {
      finish(reason, saturation_stop_, false);
      return;
    }
    have_pass_ = true;
    last_pass_command_ = command_steering_;
    last_pass_equivalent_ = equivalent;
    last_pass_curvature_ = direction_sign_ * curvature;
    last_pass_radius_ = fit.radius;

    const double next_abs = std::fabs(command_steering_) + steering_step_;
    if (next_abs > steering_max_test_ + 1e-9) {
      finish("MAX_TEST_LIMIT_REACHED", false, false);
      return;
    }
    beginStep(next_abs);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (finalized_) return;
    const ros::WallTime now = ros::WallTime::now();
    if (g_stop_requested.load()) {
      finish("USER_ABORT", false, false);
      return;
    }
    if ((now - started_at_).toSec() > max_total_time_) {
      finish("TOTAL_TIMEOUT", false, false);
      return;
    }
    if (stage_ == Stage::WAITING_FOR_POSE) {
      logTracking(false);
      if (!have_pose_) return;
      if (!insideSafeInterior(pose_)) {
        boundary_stop_ = true;
        finish("BOUNDARY_LIMIT", false, true);
        return;
      }
      beginStep(steering_start_);
      return;
    }
    if ((now - last_pose_time_).toSec() > object_timeout_) {
      finish("OBJECT_TIMEOUT", false, false);
      return;
    }
    if (!insideSafeInterior(pose_)) {
      boundary_stop_ = true;
      finish("BOUNDARY_LIMIT", false, true);
      return;
    }

    if (stage_ == Stage::SETTLING) {
      publishCommand(throttle_, command_steering_);
      logTracking(false);
      if ((now - stage_started_at_).toSec() >= settle_time_) {
        stage_ = Stage::MEASURING;
        stage_started_at_ = now;
        samples_.clear();
      }
    } else if (stage_ == Stage::MEASURING) {
      publishCommand(throttle_, command_steering_);
      const bool fresh_sample = pose_sequence_ != sampled_pose_sequence_;
      if (fresh_sample) {
        samples_.push_back(pose_);
        sampled_pose_sequence_ = pose_sequence_;
      }
      logTracking(fresh_sample);
      if ((now - stage_started_at_).toSec() >= measure_time_) finishStep();
    }
  }

  void writeReport() {
    std::ofstream report(report_path_.c_str(), std::ios::out | std::ios::trunc);
    if (!report.is_open()) {
      ROS_ERROR("Cannot write report '%s'", report_path_.c_str());
      return;
    }
    const bool complete = have_pass_ && !boundary_stop_ &&
        stop_reason_ != "USER_ABORT" && stop_reason_ != "OBJECT_TIMEOUT" &&
        stop_reason_ != "OBJECT_INVALID" && stop_reason_ != "POSE_JUMP" &&
        stop_reason_ != "TOTAL_TIMEOUT";
    report << std::fixed << std::setprecision(6)
           << "Vehicle ID: " << vehicleLabel(target_) << '\n'
           << "Direction: " << direction_ << '\n'
           << "Maximum tested command: " << maximum_tested_command_ << " rad\n"
           << "Maximum PASS command: "
           << (have_pass_ ? last_pass_command_ : 0.0) << " rad\n"
           << "Next step command: " << maximum_tested_command_ << " rad\n"
           << "Maximum measured stable curvature: "
           << (have_pass_ ? last_pass_curvature_ : 0.0) << " 1/m\n"
           << "Maximum measured stable equivalent steering: "
           << (have_pass_ ? last_pass_equivalent_ : 0.0) << " rad\n"
           << "Corresponding turning radius: "
           << (have_pass_ ? last_pass_radius_ : 0.0) << " m\n"
           << "Next step failure/stop reason: " << stop_reason_ << '\n'
           << "Stopped by mechanical saturation: "
           << (saturation_stop_ ? "YES" : "NO") << '\n'
           << "Stopped by field boundary: "
           << (boundary_stop_ ? "YES" : "NO") << '\n'
           << "Complete test: "
           << (complete ? "YES" : "NO") << '\n';
    if (have_pass_) {
      report << vehicleLabel(target_) << ' ';
      for (char c : direction_) report << static_cast<char>(std::toupper(c));
      report << " maximum trustworthy steering capability: "
             << last_pass_equivalent_ << " rad, curvature "
             << last_pass_curvature_ << " 1/m, radius "
             << last_pass_radius_ << " m.\n";
    } else {
      report << vehicleLabel(target_) << " " << direction_
             << ": no trustworthy steering capability was measured.\n";
    }
  }

  void writeInterruptedStep(const std::string& reason) {
    if (step_index_ < 0 || step_recorded_) return;
    const CircleFit fit = fitCircle(samples_, 0, samples_.size());
    double travel = 0.0;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
      travel += std::hypot(samples_[i].x - samples_[i - 1].x,
                           samples_[i].y - samples_[i - 1].y);
    }
    double arc_angle = 0.0;
    if (fit.valid) {
      for (std::size_t i = 1; i < samples_.size(); ++i) {
        const double a0 = std::atan2(samples_[i - 1].y - fit.cy,
                                     samples_[i - 1].x - fit.cx);
        const double a1 = std::atan2(samples_[i].y - fit.cy,
                                     samples_[i].x - fit.cx);
        arc_angle += std::fabs(normalizeAngle(a1 - a0));
      }
    }
    const double curvature = fit.valid ? 1.0 / fit.radius : 0.0;
    const double equivalent = fit.valid
        ? direction_sign_ * std::atan(wheel_base_ / fit.radius) : 0.0;
    steps_ << std::fixed << std::setprecision(6)
           << target_ << ',' << direction_ << ',' << step_index_ << ','
           << command_steering_ << ',' << (fit.valid ? fit.radius : 0.0)
           << ',' << curvature << ',' << equivalent << ','
           << (command_steering_ - equivalent) << ','
           << (fit.valid ? fit.rms : 0.0) << ',' << arc_angle << ','
           << travel << ',' << samples_.size() << ",FAIL," << reason << '\n';
    steps_.flush();
    step_recorded_ = true;
  }

  void finish(const std::string& reason, bool saturation, bool boundary) {
    if (finalized_) return;
    finalized_ = true;
    stage_ = Stage::STOPPING;
    stop_reason_ = reason;
    saturation_stop_ = saturation_stop_ || saturation;
    boundary_stop_ = boundary_stop_ || boundary;
    writeInterruptedStep(reason);
    publishStopFrames();
    writeReport();
    stage_ = Stage::DONE;
    if (have_pass_) {
      ROS_WARN("[%s %s] maximum trustworthy steering capability: %.3f rad, "
               "curvature %.3f 1/m, radius %.3f m. stop_reason=%s",
               vehicleLabel(target_).c_str(), direction_.c_str(),
               last_pass_equivalent_, last_pass_curvature_, last_pass_radius_,
               stop_reason_.c_str());
    } else {
      ROS_ERROR("[%s %s] no trustworthy steering capability measured; reason=%s",
                vehicleLabel(target_).c_str(), direction_.c_str(),
                stop_reason_.c_str());
    }
    ros::shutdown();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "steering_limit_calibration",
            ros::init_options::NoSigintHandler);
  std::signal(SIGINT, sigintHandler);
  try {
    SteeringLimitCalibration calibration;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Steering calibration startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
