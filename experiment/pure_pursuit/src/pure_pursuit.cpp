#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sandbox_msgs/Trajectory.h>
#include <sandbox_msgs/AprilObject.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

// Pure Pursuit Parameters
const double lfw_ = 0.0; // distance of the forward anchor point from the rear axle
const double lrv_ = 0.0;
const double wheel_base_ = 0.143;  // = 推演器 map_param.wheel_base(原0.1445,改为与规划一致)
const double vehicle_length_ = 0.211;
const double vehicle_width_ = 0.191;
const double rear_axle_to_center_ = 0.0735;

// Goal Approaching
const double approaching_tolerance_ = 0.04; // 5cm

using namespace sandbox_msgs;

class PurePursuitVisualization {
public:
  explicit PurePursuitVisualization(const ros::NodeHandle &nh): node_handle_(nh) {
    node_handle_.param<std::string>("visualization_frame", visualization_frame_, visualization_frame_);
    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/pure_pursuit", 20, true);
  }

  void set_target(int target) {
    target_ = target;
    init_marker();
  }

  void visualize_goal(double goal_x, double goal_y, double goal_yaw) {
    goal_.header.stamp = ros::Time::now();
    goal_.pose.position.x = goal_x;
    goal_.pose.position.y = goal_y;
    goal_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);

    marker_publisher_.publish(goal_);
  }

  void visualize(double pos_x, double pos_y, double car_x, double car_y,
                 double waypoint_x, double waypoint_y, bool is_virtual_waypoint) {
    points_.header.stamp = ros::Time::now();
    line_strip_.header.stamp = ros::Time::now();
    lookahead_waypoint_.header.stamp = ros::Time::now();
    virtual_waypoint_.header.stamp = ros::Time::now();
    points_.points.clear();
    line_strip_.points.clear();

    points_.points.push_back(make_point(car_x, car_y));
    points_.points.push_back(make_point(pos_x, pos_y));
    points_.points.push_back(make_point(waypoint_x, waypoint_y));
    line_strip_.points.push_back(make_point(pos_x, pos_y));
    line_strip_.points.push_back(make_point(waypoint_x, waypoint_y));

    marker_publisher_.publish(points_);
    marker_publisher_.publish(line_strip_);

    // 虚拟末端预瞄点单独使用较大的球体显示，便于在 RViz 中观察末端对齐过程。
    if(is_virtual_waypoint) {
      lookahead_waypoint_.action = visualization_msgs::Marker::DELETE;
      virtual_waypoint_.action = visualization_msgs::Marker::ADD;
      virtual_waypoint_.pose.position.x = waypoint_x;
      virtual_waypoint_.pose.position.y = waypoint_y;
      virtual_waypoint_.pose.position.z = 0.035;
    } else {
      lookahead_waypoint_.action = visualization_msgs::Marker::ADD;
      lookahead_waypoint_.pose.position.x = waypoint_x;
      lookahead_waypoint_.pose.position.y = waypoint_y;
      lookahead_waypoint_.pose.position.z = 0.03;
      // 离开末端虚拟预瞄状态后删除旧 Marker，避免 RViz 中残留。
      virtual_waypoint_.action = visualization_msgs::Marker::DELETE;
    }
    marker_publisher_.publish(lookahead_waypoint_);
    marker_publisher_.publish(virtual_waypoint_);
  }

  void visualize_reference_pose(double actual_x, double actual_y,
                                double ref_x, double ref_y, double ref_yaw,
                                double longitudinal_error, double lateral_error) {
    const ros::Time now = ros::Time::now();
    std::stringstream ns;
    ns << "Reference_" << target_;

    visualization_msgs::Marker arrow;
    arrow.header.frame_id = visualization_frame_;
    arrow.header.stamp = now;
    arrow.ns = ns.str();
    arrow.id = 10;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.pose.orientation = tf::createQuaternionMsgFromYaw(ref_yaw);
    arrow.pose.position.x = ref_x;
    arrow.pose.position.y = ref_y;
    arrow.pose.position.z = 0.08;
    arrow.scale.x = 0.12;
    arrow.scale.y = 0.025;
    arrow.scale.z = 0.04;
    arrow.color.r = 0.0;
    arrow.color.g = 0.45;
    arrow.color.b = 1.0;
    arrow.color.a = 1.0;
    marker_publisher_.publish(arrow);

    visualization_msgs::Marker body;
    body.header = arrow.header;
    body.ns = ns.str();
    body.id = 11;
    body.action = visualization_msgs::Marker::ADD;
    body.type = visualization_msgs::Marker::CUBE;
    body.color.r = 0.0;
    body.color.g = 0.85;
    body.color.b = 1.0;
    body.color.a = 0.35;
    const double cx = ref_x + rear_axle_to_center_ * std::cos(ref_yaw);
    const double cy = ref_y + rear_axle_to_center_ * std::sin(ref_yaw);
    body.pose.position.x = cx;
    body.pose.position.y = cy;
    body.pose.position.z = 0.045;
    body.pose.orientation = tf::createQuaternionMsgFromYaw(ref_yaw);
    body.scale.x = vehicle_length_;
    body.scale.y = vehicle_width_;
    body.scale.z = 0.025;
    marker_publisher_.publish(body);

    visualization_msgs::Marker text;
    text.header = arrow.header;
    text.ns = ns.str();
    text.id = 12;
    text.action = visualization_msgs::Marker::ADD;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.pose.orientation.w = 1.0;
    text.pose.position.x = 0.5 * (actual_x + ref_x);
    text.pose.position.y = 0.5 * (actual_y + ref_y);
    text.pose.position.z = 0.18;
    text.scale.z = 0.055;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    std::stringstream ss;
    ss << "V" << target_
       << " lon=" << std::fixed << std::setprecision(3) << longitudinal_error
       << " lat=" << lateral_error;
    text.text = ss.str();
    marker_publisher_.publish(text);
  }

private:
  static inline geometry_msgs::Point make_point(double x, double y, double z = 0.0) {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
  }

  void init_marker() {
    points_.header.frame_id = line_strip_.header.frame_id = goal_.header.frame_id =
        lookahead_waypoint_.header.frame_id = virtual_waypoint_.header.frame_id =
            visualization_frame_;
    std::stringstream ss;
    ss << "Markers_" << target_;
    points_.ns = line_strip_.ns = goal_.ns = lookahead_waypoint_.ns =
        virtual_waypoint_.ns = ss.str();
    points_.action = line_strip_.action = goal_.action = visualization_msgs::Marker::ADD;
    points_.pose.orientation.w = line_strip_.pose.orientation.w = goal_.pose.orientation.w =
        lookahead_waypoint_.pose.orientation.w = virtual_waypoint_.pose.orientation.w = 1.0;
    points_.id = 0;
    line_strip_.id = 1;
    goal_.id = 2;
    virtual_waypoint_.id = 3;
    lookahead_waypoint_.id = 4;

    points_.type = visualization_msgs::Marker::POINTS;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    goal_.type = visualization_msgs::Marker::CYLINDER;
    lookahead_waypoint_.type = visualization_msgs::Marker::SPHERE;
    virtual_waypoint_.type = visualization_msgs::Marker::SPHERE;
    points_.scale.x = 0.02;
    points_.scale.y = 0.02;

    line_strip_.scale.x = 0.01;

    goal_.scale.x = approaching_tolerance_;
    goal_.scale.y = approaching_tolerance_;
    goal_.scale.z = 0.1;

    lookahead_waypoint_.scale.x = 0.055;
    lookahead_waypoint_.scale.y = 0.055;
    lookahead_waypoint_.scale.z = 0.055;
    lookahead_waypoint_.color.r = 0.0;
    lookahead_waypoint_.color.g = 1.0;
    lookahead_waypoint_.color.b = 0.35;
    lookahead_waypoint_.color.a = 1.0;
    lookahead_waypoint_.lifetime = ros::Duration(0.25);

    virtual_waypoint_.scale.x = 0.07;
    virtual_waypoint_.scale.y = 0.07;
    virtual_waypoint_.scale.z = 0.07;
    virtual_waypoint_.color.r = 1.0;
    virtual_waypoint_.color.g = 0.15;
    virtual_waypoint_.color.b = 0.0;
    virtual_waypoint_.color.a = 1.0;
    virtual_waypoint_.lifetime = ros::Duration(0.25);

    points_.color.g = 1.0f;
    points_.color.a = 1.0;

    line_strip_.color.b = 1.0;
    line_strip_.color.a = 1.0;

    goal_.color.r = 1.0;
    goal_.color.g = 1.0;
    goal_.color.b = 0.0;
    goal_.color.a = 0.5;
  }

  int target_ = 0;
  ros::NodeHandle node_handle_;
  ros::Publisher marker_publisher_;
  std::string visualization_frame_ = "map";
  visualization_msgs::Marker points_, line_strip_, goal_, lookahead_waypoint_,
      virtual_waypoint_;
};

TrajectoryPoint interpolate(const TrajectoryPoint p0, TrajectoryPoint p1, double weight) {
  TrajectoryPoint pt;
  pt.time = (1 - weight) * p0.time + weight * p1.time;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.yaw = (1 - weight) * p0.yaw + weight * p1.yaw;
  pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
  pt.acceleration = (1 - weight) * p0.acceleration + weight * p1.acceleration;
  return pt;
}

TrajectoryPoint interpolateWithTime(const TrajectoryPoint p0, TrajectoryPoint p1, double time) {
  double weight = (time - p0.time) / (p1.time - p0.time);
  return interpolate(p0, p1, weight);
}

class PurePursuit {
public:
  PurePursuit(const ros::NodeHandle &nh): node_handle_(nh), visualization_(nh) {
    
    tracking_object_ = node_handle_.param<int>("target", 0);

    // Per-vehicle PP gains are loaded under /controller_<target> from
    // vehicle_controller_gains.yaml. The private NodeHandle reads that namespace.
    longitude_kp_ = node_handle_.param("longitude_kp", longitude_kp_);
    longitude_ki_ = node_handle_.param("longitude_ki", longitude_ki_);
    lookahead_scale_ = node_handle_.param("lookahead_scale", lookahead_scale_);
    lookahead_distance_min_ = node_handle_.param(
        "lookahead_distance_min", lookahead_distance_min_);
    lookahead_distance_max_ = node_handle_.param(
        "lookahead_distance_max", lookahead_distance_max_);
    lookahead_distance_ratio_ = node_handle_.param(
        "lookahead_distance_ratio", lookahead_distance_ratio_);
    pp_max_steer_angle_ = node_handle_.param(
        "pp_max_steer_angle", pp_max_steer_angle_);
    final_lon_tolerance_ = node_handle_.param("final_lon_tolerance", final_lon_tolerance_);
    final_lat_tolerance_ = node_handle_.param("final_lat_tolerance", final_lat_tolerance_);
    final_yaw_tolerance_ = node_handle_.param("final_yaw_tolerance", final_yaw_tolerance_);
    final_stable_cycles_required_ = node_handle_.param("final_stable_cycles", final_stable_cycles_required_);
    terminal_lookahead_extension_ = node_handle_.param("terminal_lookahead_extension", terminal_lookahead_extension_);
    terminal_slowdown_distance_ = node_handle_.param("terminal_slowdown_distance", terminal_slowdown_distance_);
    terminal_max_speed_ = node_handle_.param("terminal_max_speed", terminal_max_speed_);
    ROS_INFO("[PP] target=%d longitude_kp=%.3f longitude_ki=%.3f "
             "lookahead=[%.3f,%.3f] ratio=%.3f scale=%.3f max_steer=%.3frad",
             tracking_object_, longitude_kp_, longitude_ki_,
             lookahead_distance_min_, lookahead_distance_max_,
             lookahead_distance_ratio_, lookahead_scale_, pp_max_steer_angle_);
    ROS_INFO("[PP] final tolerance: lon=%.3f lat=%.3f yaw=%.1fdeg stable=%d, extension=%.3f slow_dist=%.3f max_speed=%.3f",
             final_lon_tolerance_, final_lat_tolerance_, final_yaw_tolerance_ * 180.0 / M_PI,
             final_stable_cycles_required_, terminal_lookahead_extension_,
             terminal_slowdown_distance_, terminal_max_speed_);

    const std::string planner_package = ros::package::getPath("forklift_planner");
    const std::string planner_parent = planner_package.empty()
        ? "." : planner_package.substr(0, planner_package.find_last_of('/'));
    debug_log_dir_ = planner_parent + "/log";
    debug_log_enable_ = node_handle_.param("debug_log_enable", debug_log_enable_);
    node_handle_.param<std::string>("debug_log_dir", debug_log_dir_, debug_log_dir_);
    if(debug_log_enable_) {
      ensure_log_dir();
      initialize_debug_logs();
      ROS_INFO("[PP] target=%d debug logs enabled: %s", tracking_object_, debug_log_dir_.c_str());
    }

    visualization_.set_target(tracking_object_);

    std::string traj_topic = "/traj_" + std::to_string(tracking_object_);
    traj_subscriber_ = node_handle_.subscribe<Trajectory>(traj_topic, 1, &PurePursuit::traj_callback, this);
    object_subscriber_ = node_handle_.subscribe<AprilObject>("/object", 5, &PurePursuit::object_callback, this);
    command_publisher_ = node_handle_.advertise<ChassisCommand>("/chassis", 1, true);
    reached_publisher_ = node_handle_.advertise<std_msgs::Int32>("/reached", 1, false);
    estop_subscriber_ = node_handle_.subscribe<std_msgs::Bool>(
        "/estop", 1, &PurePursuit::estop_callback, this);

    timer_ = node_handle_.createTimer(ros::Duration(0.1), &PurePursuit::control_callback, this);
  }

private:
  ros::NodeHandle node_handle_;
  ros::Publisher command_publisher_, reached_publisher_;
  ros::Subscriber traj_subscriber_, object_subscriber_, truth_subscriber_, execute_subscriber_;
  ros::Subscriber estop_subscriber_;
  ros::Timer timer_;
  double start_time_ = 0.0;
  PurePursuitVisualization visualization_;
  Trajectory trajectory_;
  AprilObject object_;
  std::vector<std::pair<int, int>> ranges_;

  int current_range_ = 0;
  int tracking_object_ = 0;

  double lookahead_distance_ = 0.10;
  double lookahead_distance_min_ = 0.10;
  double lookahead_distance_max_ = 0.15;
  double lookahead_distance_ratio_ = 0.8;
  double lookahead_scale_ = 1.0;
  double pp_max_steer_angle_ = 0.45;
  double longitude_kp_ = 1.8, longitude_ki_ = 0.0, longitude_output_ = 0.0, longitude_perror_ = 0.0;
  // 末端位姿判定参数：分别约束目标坐标系下的纵向、横向和航向误差。
  double final_lon_tolerance_ = 0.02;
  double final_lat_tolerance_ = 0.015;
  double final_yaw_tolerance_ = 5.0 * M_PI / 180.0;
  int final_stable_cycles_required_ = 5;

  // 末端 PP 参数。虚拟点只参与横向转向计算，纵向 PI 始终跟踪真实轨迹。
  double terminal_lookahead_extension_ = 0.12;
  double terminal_slowdown_distance_ = 0.20;
  double terminal_max_speed_ = 0.08;
  double lateral_target_x_ = 0.0, lateral_target_y_ = 0.0;
  bool using_virtual_lookahead_ = false;
  int final_stable_cycles_ = 0;
  int car_index_ = 0, lookahead_index_ = 0;
  bool debug_log_enable_ = true;
  std::string debug_log_dir_;
  unsigned long long trajectory_seq_ = 0;
  double trajectory_receive_time_ = 0.0;
  int current_path_gen_ = -1;
  bool object_seen_ = false;
  std::string trajectory_refresh_type_ = "NEW_PATH";
  std::ofstream open_loop_log_, closed_loop_log_;

  bool approached_ = false;
  bool temporary_hold_ = false;    //短暂静止
  bool estop_ = false;
  bool waiting_for_post_estop_trajectory_ = false;

  void reset_control_history() {
    longitude_output_ = 0.0;
    longitude_perror_ = 0.0;
    final_stable_cycles_ = 0;
    car_index_ = 0;
    lookahead_index_ = 0;
    current_range_ = 0;
    using_virtual_lookahead_ = false;
    approached_ = false;
  }

  void publish_zero_command() {
    ChassisCommand command;
    command.target = tracking_object_;
    command.throttle = 0.0;
    command.steering = 0.0;
    command_publisher_.publish(command);
  }

  void estop_callback(const std_msgs::BoolConstPtr &msg) {
    const bool was_estopped = estop_;
    estop_ = msg->data;

    if(estop_) {
      waiting_for_post_estop_trajectory_ = true;
      reset_control_history();
      publish_zero_command();
      ROS_ERROR("[PP] target=%d global estop asserted: chassis throttle=0",
                tracking_object_);
      return;
    }

    if(was_estopped) {
      // Drop the pre-estop reference and PI state. Control stays at zero
      // until the planner's measured-state refresh reaches traj_callback().
      trajectory_.points.clear();
      ranges_.clear();
      reset_control_history();
      waiting_for_post_estop_trajectory_ = true;
      publish_zero_command();
      ROS_WARN("[PP] target=%d global estop released: waiting for fresh trajectory",
               tracking_object_);
    }
  }

  void object_callback(const AprilObjectConstPtr &msg) {
    if(msg->id == tracking_object_ && msg->type == AprilObject::VEHICLE) {
      double new_x = msg->x / 1000.0;
      double new_y = msg->y / 1000.0;
      if(object_.x != 0 && object_.y != 0) {
        double distance = hypot(new_x - object_.x, new_y - object_.y);
        if(distance > 0.5) return;
      }
      object_ = *msg;
      object_.x = new_x;
      object_.y = new_y;
      object_seen_ = true;
    }
  }

  int nearest_index_in(const Trajectory &traj) const {
    if(!object_seen_ || traj.points.empty()) return 0;
    int nearest = 0;
    double best_score = DBL_MAX;
    for(int i = 0; i < static_cast<int>(traj.points.size()); ++i) {
      const TrajectoryPoint &point = traj.points[i];
      const double distance = hypot(point.y - object_.y, point.x - object_.x);
      const double yaw_error = fabs(normalize_angle(point.yaw - object_.yaw));
      const double score = distance + 0.03 * yaw_error / M_PI;
      if(score < best_score) {
        best_score = score;
        nearest = i;
      }
    }
    return nearest;
  }

  void traj_callback(const TrajectoryConstPtr &msg) {
    if(msg->target != tracking_object_) return;
    if(msg->points.empty()) return;
    if(estop_) return;

    waiting_for_post_estop_trajectory_ = false;

    // 单点轨迹是 planner 的临时 HOLD，不代表当前 path_gen 已真正到达终点。后续收到正常 rolling trajectory 时自动解除 HOLD。
    temporary_hold_ = (msg->points.size() == 1);
    if(!temporary_hold_ && approached_) 
    {
      approached_ = false;
      final_stable_cycles_ = 0;
    }
    std::cout << tracking_object_ << " - Trajectory received" << std::endl;

    const int incoming_path_gen = static_cast<int>(msg->header.seq);
    const bool new_path = current_path_gen_ < 0 ||
                          incoming_path_gen != current_path_gen_;
    trajectory_refresh_type_ = new_path ? "NEW_PATH" : "ROLLING_REFRESH";
    if(new_path) {
      current_path_gen_ = incoming_path_gen;
      car_index_ = lookahead_index_ = 0;
      current_range_ = 0;
      final_stable_cycles_ = 0;
      using_virtual_lookahead_ = false;
      approached_ = false;
    }

    ranges_.clear();
    int start = 0;

    for(int i = 1; i < (int)msg->points.size(); i++) {
      if(fabs(msg->points[i].velocity) > 0.01 &&
        fabs(msg->points[i-1].velocity) > 0.01 &&
        copysign(1.0, msg->points[i].velocity) !=
        copysign(1.0, msg->points[i-1].velocity)) {

        ranges_.emplace_back(start, i);
        start = i;
      }
    }

    ranges_.emplace_back(start, (int)msg->points.size());
    // 先安装本次新的 rolling trajectory
    trajectory_ = *msg;

    if(!new_path)
    {
    // 新 rolling trajectory 的 range 0 就是当前运动段，禁止在整条未来轨迹上重新做全局 nearest。
      current_range_ = 0;
      const int range_start = ranges_[current_range_].first;
      const int range_end   = ranges_[current_range_].second;
      car_index_ = nearest_index_in_range(range_start, range_end);
      lookahead_index_ = car_index_;
      using_virtual_lookahead_ = false;
      ROS_INFO(
      "[PP][ROLLING_RANGE] target=%d path_gen=%d "
      "force_current_range=0 range=[%d,%d) car_index=%d ranges=%zu",
      tracking_object_,
      current_path_gen_,
      range_start,
      range_end,
      car_index_,
      ranges_.size());
    }

    auto next_goal = msg->points[ranges_[current_range_].second - 1];
    visualization_.visualize_goal(next_goal.x, next_goal.y, next_goal.yaw);

    start_time_ = ros::Time::now().toSec();
    trajectory_receive_time_ = start_time_;
    ++trajectory_seq_;
    log_open_loop_trajectory(*msg);
    if(!new_path && object_seen_) update_lookahead();
    ROS_INFO("[PP] target=%d trajectory_seq=%llu path_gen=%d refresh=%s "
             "car_index=%d lookahead_index=%d longitude_output=%.6f "
             "longitude_perror=%.6f",
             tracking_object_, trajectory_seq_, current_path_gen_,
             trajectory_refresh_type_.c_str(), car_index_, lookahead_index_,
             longitude_output_, longitude_perror_);
  }

  inline double distance_to_traj(int i, double x, double y) {
    return hypot(trajectory_.points[i].y - y, trajectory_.points[i].x - x);
  }

  int nearest_index_in_range(int start, int end) const {
    start = std::max(0, start);
    end = std::min(end, static_cast<int>(trajectory_.points.size()));
    if(start >= end) return start;

    int nearest = start;
    double min_distance = DBL_MAX;
    for(int i = start; i < end; ++i) {
      const double distance = hypot(trajectory_.points[i].x - object_.x,
                                    trajectory_.points[i].y - object_.y);
      if(distance < min_distance) {
        min_distance = distance;
        nearest = i;
      }
    }
    return nearest;
  }

  double remaining_range_length(int from, int end) const {
    from = std::max(0, from);
    end = std::min(end, static_cast<int>(trajectory_.points.size()));
    double length = 0.0;
    for(int i = from + 1; i < end; ++i) {
      length += hypot(trajectory_.points[i].x - trajectory_.points[i - 1].x,
                      trajectory_.points[i].y - trajectory_.points[i - 1].y);
    }
    return length;
  }

  static double normalize_angle(double angle) {
    const double kPi = 3.14159265358979323846;
    while(angle > kPi) angle -= 2.0 * kPi;
    while(angle < -kPi) angle += 2.0 * kPi;
    return angle;
  }

  void ensure_log_dir() {
    mkdir(debug_log_dir_.c_str(), 0755);
  }

  std::string log_path(const std::string &kind) const {
    std::stringstream ss;
    ss << debug_log_dir_ << "/controller_V" << tracking_object_ << "_"
       << kind << ".csv";
    return ss.str();
  }

  TrajectoryPoint reference_at_time(double elapsed) const {
    TrajectoryPoint ref = trajectory_.points.back();
    auto upper_point = std::lower_bound(trajectory_.points.begin(), trajectory_.points.end(), elapsed,
      [](const TrajectoryPoint &tp, double t) {
        return tp.time < t;
      });

    if(upper_point != trajectory_.points.end()) {
      ref = *upper_point;
      if(upper_point > trajectory_.points.begin()) {
        const TrajectoryPoint &lo = *std::prev(upper_point);
        const TrajectoryPoint &hi = *upper_point;
        if(fabs(hi.time - lo.time) > 1e-6) {
          ref = interpolateWithTime(lo, hi, elapsed);
        }
      }
    }
    return ref;
  }

  void initialize_debug_logs() {
    if(!debug_log_enable_) return;
    ensure_log_dir();
    const std::string open_path = log_path("open_loop");
    const std::string closed_path = log_path("tracking");
    open_loop_log_.open(open_path.c_str(), std::ios::out | std::ios::trunc);
    closed_loop_log_.open(closed_path.c_str(), std::ios::out | std::ios::trunc);

    if(open_loop_log_) {
      open_loop_log_ << "trajectory_seq,receive_wall_time,index,traj_time,"
                     << "x,y,yaw,velocity,acceleration\n";
      open_loop_log_.flush();
    }

    if(closed_loop_log_) {
      closed_loop_log_ << "wall_time,trajectory_seq,trajectory_receive_time,"
                       << "traj_time,target,path_gen,trajectory_refresh_type,"
                       << "range,car_index,lookahead_index,"
                       << "ref_x,ref_y,ref_yaw,ref_velocity,"
                       << "actual_x,actual_y,actual_yaw,"
                       << "longitudinal_error,lateral_error,yaw_error,nearest_distance,"
                       << "lookahead_distance,lookahead_scale,longitude_kp,longitude_ki,"
                       << "longitude_output,longitude_perror,"
                       << "throttle,steering,lookahead_x,lookahead_y,"
                       << "using_virtual_lookahead,eta\n";
      closed_loop_log_.flush();
    }

    ROS_INFO("[PP] target=%d logging open_loop=%s closed_loop=%s",
             tracking_object_, open_path.c_str(), closed_path.c_str());
  }

  void log_open_loop_trajectory(const Trajectory &traj) {
    if(!debug_log_enable_ || !open_loop_log_) return;
    for(size_t i = 0; i < traj.points.size(); ++i) {
      const auto &p = traj.points[i];
      open_loop_log_ << trajectory_seq_ << "," << trajectory_receive_time_
                     << "," << i << "," << p.time << "," << p.x << ","
                     << p.y << "," << p.yaw << "," << p.velocity << ","
                     << p.acceleration << "\n";
    }
    open_loop_log_.flush();
  }

  void log_tracking_sample(double throttle, double steering, double eta) {
    if(!debug_log_enable_ || !closed_loop_log_ || trajectory_.points.empty()) return;
    const double now = ros::Time::now().toSec();
    const double elapsed = now - start_time_;
    const TrajectoryPoint ref = reference_at_time(elapsed);

    const double sin_ref = sin(ref.yaw), cos_ref = cos(ref.yaw);
    const double dx = ref.x - object_.x;
    const double dy = ref.y - object_.y;
    const double longitudinal_error = cos_ref * dx + sin_ref * dy;
    const double lateral_error = -sin_ref * dx + cos_ref * dy;
    const double yaw_error = normalize_angle(ref.yaw - object_.yaw);
    const double nearest_distance = (car_index_ >= 0 && car_index_ < (int)trajectory_.points.size())
        ? distance_to_traj(car_index_, object_.x, object_.y) : 0.0;

    closed_loop_log_ << now << "," << trajectory_seq_ << ","
                     << trajectory_receive_time_ << "," << elapsed << ","
                     << tracking_object_ << "," << current_path_gen_ << ","
                     << trajectory_refresh_type_ << ","
                     << current_range_ << "," << car_index_ << "," << lookahead_index_ << ","
                     << ref.x << "," << ref.y << "," << ref.yaw << "," << ref.velocity << ","
                     << object_.x << "," << object_.y << "," << object_.yaw << ","
                     << longitudinal_error << "," << lateral_error << "," << yaw_error << ","
                     << nearest_distance << "," << lookahead_distance_ << "," << lookahead_scale_ << ","
                     << longitude_kp_ << "," << longitude_ki_ << ","
                     << longitude_output_ << "," << longitude_perror_ << ","
                     << throttle << "," << steering << ","
                     << lateral_target_x_ << "," << lateral_target_y_ << ","
                     << (using_virtual_lookahead_ ? 1 : 0) << "," << eta << "\n";
    closed_loop_log_.flush();
  }

  void update_lookahead() {
    if(trajectory_.points.empty()) return;

    bool found_lookahead = false;
    int start = ranges_[current_range_].first, end = ranges_[current_range_].second;

    car_index_ = nearest_index_in_range(start, end);

    const double v_cmd = fabs(trajectory_.points[car_index_].velocity);
    const double base_lookahead = std::max(
        lookahead_distance_min_,
        std::min(lookahead_distance_max_,
                 v_cmd * lookahead_distance_ratio_));
    lookahead_distance_ = base_lookahead * lookahead_scale_;

    double travel_sign = 1.0;
    for(int i = end - 1; i >= start; --i) {
      if(fabs(trajectory_.points[i].velocity) > 0.01) {
        travel_sign = trajectory_.points[i].velocity > 0.0 ? 1.0 : -1.0;
        break;
      }
    }
    const double motion_yaw = object_.yaw + (travel_sign < 0.0 ? M_PI : 0.0);
    const double motion_x = cos(motion_yaw);
    const double motion_y = sin(motion_yaw);
    double accumulated_distance = 0.0;
    int last_forward_index = -1;
    for(int i = car_index_ + 1; i < end; ++i) {
      accumulated_distance += hypot(
          trajectory_.points[i].x - trajectory_.points[i - 1].x,
          trajectory_.points[i].y - trajectory_.points[i - 1].y);
      const double relative_x = trajectory_.points[i].x - object_.x;
      const double relative_y = trajectory_.points[i].y - object_.y;
      const double forward_dot = relative_x * motion_x + relative_y * motion_y;
      if(forward_dot <= 0.0) continue;
      last_forward_index = i;
      if(accumulated_distance >= lookahead_distance_) {
        lookahead_index_ = i;
        found_lookahead = true;
        break;
      }
    }

    if(!found_lookahead) {
      lookahead_index_ = last_forward_index >= 0 ? last_forward_index : car_index_;
    }

    lateral_target_x_ = trajectory_.points[lookahead_index_].x;
    lateral_target_y_ = trajectory_.points[lookahead_index_].y;
    using_virtual_lookahead_ = false;

    if(!found_lookahead && last_forward_index < 0) {
      lateral_target_x_ = object_.x + lookahead_distance_ * motion_x;
      lateral_target_y_ = object_.y + lookahead_distance_ * motion_y;
      using_virtual_lookahead_ = true;
    }

    if(current_range_ < static_cast<int>(ranges_.size()) - 1) {
      const double remaining_length = remaining_range_length(car_index_, end);
      const double extension = std::min(
          0.12, std::max(0.0, lookahead_distance_ - remaining_length));
      if(extension > 1e-6) {
        const TrajectoryPoint &cusp = trajectory_.points[end - 1];
        double travel_sign = 1.0;
        for(int i = end - 1; i >= start; --i) {
          if(fabs(trajectory_.points[i].velocity) > 0.01) {
            travel_sign = trajectory_.points[i].velocity > 0.0 ? 1.0 : -1.0;
            break;
          }
        }
        const double motion_yaw = cusp.yaw + (travel_sign < 0.0 ? M_PI : 0.0);
        lateral_target_x_ = cusp.x + extension * cos(motion_yaw);
        lateral_target_y_ = cusp.y + extension * sin(motion_yaw);
        using_virtual_lookahead_ = true;
      }
    }

    // 最后一段没有足够远的真实预瞄点时，沿“实际行驶方向”延长一个虚拟点。
    // 前进时延伸到目标朝向前方；倒车时延伸到目标朝向后方。
    // 该虚拟点只用于 PP 横向转向，纵向 PI 仍使用真实轨迹终点，
    // 因此不会把虚拟点当成停车目标而纵向贯穿货位。
    if(!found_lookahead && current_range_ == static_cast<int>(ranges_.size()) - 1) {
      const TrajectoryPoint &goal = trajectory_.points[end - 1];
      double travel_sign = 1.0;
      for(int i = end - 1; i >= start; --i) {
        if(fabs(trajectory_.points[i].velocity) > 0.01) {
          travel_sign = trajectory_.points[i].velocity > 0.0 ? 1.0 : -1.0;
          break;
        }
      }
      lateral_target_x_ = goal.x + travel_sign * terminal_lookahead_extension_ * cos(goal.yaw);
      lateral_target_y_ = goal.y + travel_sign * terminal_lookahead_extension_ * sin(goal.yaw);
      using_virtual_lookahead_ = true;
    }

    visualization_.visualize(object_.x, object_.y,
                             trajectory_.points[car_index_].x, trajectory_.points[car_index_].y,
                             lateral_target_x_, lateral_target_y_, using_virtual_lookahead_);
  }

  void switch_range() {
    // already final path
    int end = ranges_[current_range_].second;
    double approaching_distance = distance_to_traj(end-1, object_.x, object_.y);
    const bool final_range = current_range_ == static_cast<int>(ranges_.size()) - 1;

    if(final_range) {
      const TrajectoryPoint &goal = trajectory_.points[end - 1];
      const double dx = goal.x - object_.x;
      const double dy = goal.y - object_.y;
      const double e_lon = cos(goal.yaw) * dx + sin(goal.yaw) * dy;
      const double e_lat = -sin(goal.yaw) * dx + cos(goal.yaw) * dy;
      const double e_yaw = normalize_angle(goal.yaw - object_.yaw);

      // 位置和航向必须连续多个控制周期同时满足，避免定位抖动造成误到达。
      if(fabs(e_lon) <= final_lon_tolerance_ &&
         fabs(e_lat) <= final_lat_tolerance_ &&
         fabs(e_yaw) <= final_yaw_tolerance_) {
        ++final_stable_cycles_;
      } else {
        final_stable_cycles_ = 0;
      }

      if(final_stable_cycles_ >= std::max(1, final_stable_cycles_required_)) {
        approached_ = true;
        std_msgs::Int32 msg;
        msg.data = tracking_object_;
        reached_publisher_.publish(msg);
        longitude_output_ = longitude_perror_ = 0.0;
        std::cout << tracking_object_ << " - Goal pose reached" << std::endl;
      }
      return;
    }

    const int start = ranges_[current_range_].first;
    const int projected_index = nearest_index_in_range(start, end);
    const bool projection_at_cusp =
        projected_index >= std::max(start, end - 2);
    if(projection_at_cusp && approaching_distance <= approaching_tolerance_) {
        current_range_++;

        const int next_start = ranges_[current_range_].first;
        end = ranges_[current_range_].second;
        car_index_ = nearest_index_in_range(next_start, end);
        lookahead_index_ = car_index_;
        using_virtual_lookahead_ = false;
        auto goal = trajectory_.points[end - 1];

        visualization_.visualize_goal(goal.x, goal.y, goal.yaw);
        std::cout << tracking_object_ << " - Approached, changing goal to: (" << goal.x << ", " << goal.y << ')' << std::endl;
    }
  }

  double longitude_controller() {
    double time = ros::Time::now().toSec() - start_time_;
    TrajectoryPoint truth = reference_at_time(time);

    double sinx = sin(truth.yaw), cosx = cos(truth.yaw);
    double relative_x = cosx * (truth.x - object_.x) + sinx * (truth.y - object_.y);
    double relative_y = -sinx * (truth.x - object_.x) + cosx * (truth.y - object_.y);
    visualization_.visualize_reference_pose(object_.x, object_.y, truth.x, truth.y,
                                            truth.yaw, relative_x, relative_y);

//     std::cout << tracking_object_ << " - error: " << relative_x << std::endl;

    longitude_output_ += longitude_kp_ * (relative_x - longitude_perror_) + longitude_ki_ * relative_x;
    longitude_perror_ = relative_x;

    longitude_output_ = std::min(std::max(longitude_output_, -0.26), 0.26);  // = 推演器 max_speed 0.26(原±0.2)
    // 靠近真实终点后限制纵向速度，给横向和航向误差留出稳定收敛时间。
    // 这里的距离、速度限制均相对真实终点计算，与虚拟横向预瞄点无关。
    if(current_range_ == static_cast<int>(ranges_.size()) - 1) {
      const int end = ranges_[current_range_].second;
      const double goal_distance = distance_to_traj(end - 1, object_.x, object_.y);
      if(goal_distance <= terminal_slowdown_distance_) {
        longitude_output_ = std::min(std::max(longitude_output_, -terminal_max_speed_),
                                     terminal_max_speed_);
      }
    }
    return longitude_output_;
  }

  void controller() {
    const TrajectoryPoint &lookahead = trajectory_.points[lookahead_index_];
    // lateral_target_* 可能是末端虚拟点，但它绝不传给 longitude_controller()。
    const double lookahead_distance = hypot(lateral_target_x_ - object_.x,
                                            lateral_target_y_ - object_.y);

    double sinx = sin(object_.yaw), cosx = cos(object_.yaw);
    double relative_x = cosx * (lateral_target_x_ - object_.x) + sinx * (lateral_target_y_ - object_.y);
    double relative_y = -sinx * (lateral_target_x_ - object_.x) + cosx * (lateral_target_y_ - object_.y);
    double eta = atan2(relative_y, relative_x);

    // steering angle
    double delta = atan2(wheel_base_ * sin(eta), lookahead_distance / 2 + (lookahead.velocity > 0 ? lfw_ : lrv_) * cos(eta));
    delta = std::max(-pp_max_steer_angle_,
                     std::min(pp_max_steer_angle_, delta));
    double velocity = longitude_controller();

//    ROS_INFO("[%d] velocity: %f", tracking_object_, velocity);
//    ROS_INFO("PP: %f, %f, %f", eta, lookahead_distance, delta);

    ChassisCommand command;
    command.target = tracking_object_;
    command.throttle = velocity;
    command.steering = delta;
//    for(auto &cmd : last_commands_) {
//      command.throttle += cmd.throttle;
//      command.steering += cmd.steering;
//    }
//    command.throttle /= last_commands_.size() + 1;
//    command.steering /= last_commands_.size() + 1;
//
//    if(last_commands_.size() >= 3) {
//      last_commands_.pop_front();
//    }
//    last_commands_.push_back(command);

    command_publisher_.publish(command);
    log_tracking_sample(velocity, delta, eta);
  }

  void control_callback(const ros::TimerEvent &evt) {
    if(estop_ || waiting_for_post_estop_trajectory_ || temporary_hold_) {
      publish_zero_command();
      return;
    }

    if(!trajectory_.points.empty() && !approached_) {
      switch_range();
      // 本周期刚满足末端位姿判定时立即停车，避免再发送一帧非零控制命令。
      if(approached_) {
        ChassisCommand command;
        command.target = tracking_object_;
        command.throttle = 0.0;
        command.steering = 0.0;
        command_publisher_.publish(command);
        return;
      }
      update_lookahead();
      controller();
    } else {
      ChassisCommand command;
      command.target = tracking_object_;
      command.throttle = 0.0;
      command.steering = 0.0;

      command_publisher_.publish(command);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit");
  ros::NodeHandle nh("~");

  PurePursuit pp(nh);
  ros::spin();

  return 0;
}
