#include <ros/ros.h>
#include <lqr_controller.h>
#include <sandbox_msgs/Trajectory.h>
#include <sandbox_msgs/AprilObject.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <string>
#include <vector>

// 多车 + 实车适配版(与 pure_pursuit 对齐,可由 launch 的 controller:=lqr 选用):
//   · target 参数化:订 /traj_<target> + /coord_speed_<target>,/object 按 id==target 过滤,发 /chassis(target)。
//   · /object 是 mm → /1000 转米(原版漏了)。
//   · 第一性:纵向统一用协调的 /coord_speed(带符号),LQR 只负责【横向 steering】——
//     这样 PP/LQR 公平对比(都听协调的速度),且忠实复现仿真的停走/错峰。
//   · 自动启动(实车无 /execute):收到首条轨迹即开始。
//   · 多段(入库倒车 cusp):按速度符号切分,车到段末切下一段(原版只跟第一段)。
class LQRControllerNode {
public:
  explicit LQRControllerNode(const ros::NodeHandle &nh) : nh_(nh), controller_(0.1, 0.1445) {
    target_ = nh_.param<int>("target", 0);
    odom_subscriber_ = nh_.subscribe("/object", 5, &LQRControllerNode::object_callback, this);
    traj_subscriber_ = nh_.subscribe("/traj_" + std::to_string(target_), 1,
                                     &LQRControllerNode::traj_callback, this);
    coord_speed_subscriber_ = nh_.subscribe("/coord_speed_" + std::to_string(target_), 1,
                                            &LQRControllerNode::coord_speed_callback, this);
    twist_publisher_ = nh_.advertise<sandbox_msgs::ChassisCommand>("/chassis", 1);
    timer_ = nh_.createTimer(ros::Duration(0.1), &LQRControllerNode::control_callback, this);

    // Q/R 设一次(原在 /execute 回调里;实车自动启,移到构造)。
    // q_lat = 横向误差权重 Q(0,0):原版 1.0(整定~2.3s,偏肉);可选 ~8(整定~1.4s,正常误差不饱和)。
    // 经桌面复验:Q(0,0)≥10 在 v=0.1、10cm偏差时 δ 会超 ±28.65° 饱和,故备选值取 8 而非更大。二选一实测。
    double q_lat = nh_.param("q_lat", 1.0);
    LQRController::MatrixQ Q = LQRController::MatrixQ::Identity();
    Q(0, 0) = q_lat; Q(1, 1) = 0.2; Q(2, 2) = 0.2; Q(3, 3) = 0.2;
    controller_.set_q(Q);
    LQRController::MatrixR R = LQRController::MatrixR::Identity();
    R(0, 0) = 1.0;
    controller_.set_r(R);

    double kp = nh_.param("longitude_kp", 2.0);
    double ki = nh_.param("longitude_ki", 0.01);
    controller_.set_longitudial_pi(kp, ki);
    ROS_INFO("[LQR] target=%d q_lat=%.1f 初始化(横向LQR + 纵向用/coord_speed)", target_, q_lat);
  }

  void stop() {
    sandbox_msgs::ChassisCommand m;
    m.target = target_; m.throttle = 0.0; m.steering = 0.0;
    twist_publisher_.publish(m);
  }

private:
  ros::NodeHandle nh_;
  LQRController controller_;
  int target_ = 0;
  bool trajectory_received_ = false;
  bool coord_speed_received_ = false;
  double coord_speed_ = 0.0;

  Trajectory trajectory_;                       // planning_core::Trajectory(全程)
  std::vector<std::pair<int, int>> ranges_;     // 速度符号分段(cusp)
  int current_range_ = 0;

  ros::Timer timer_;
  ros::Subscriber odom_subscriber_, traj_subscriber_, coord_speed_subscriber_;
  ros::Publisher twist_publisher_;
  LQRController::State state_{};
  double last_state_time_ = 0.0;

  void coord_speed_callback(const std_msgs::Float64ConstPtr &msg) {
    coord_speed_ = msg->data;
    coord_speed_received_ = true;
  }

  // /object(mm,后轮中心)→ 本车状态(/1000 转米),并差分估速度供 LQR 的 A 矩阵用。
  void object_callback(const sandbox_msgs::AprilObjectConstPtr &msg) {
    if (msg->id != target_ || msg->type != sandbox_msgs::AprilObject::VEHICLE) return;
    double nx = msg->x / 1000.0, ny = msg->y / 1000.0;
    if (state_.x != 0 && state_.y != 0) {
      double d = std::hypot(nx - state_.x, ny - state_.y);
      if (d > 0.5) return;  // 动捕跳变保护
      double dt = msg->header.stamp.toSec() - last_state_time_;
      if (dt > 1e-3) state_.velocity = d / dt;
    }
    last_state_time_ = msg->header.stamp.toSec();
    state_.x = nx; state_.y = ny; state_.yaw = msg->yaw;
    controller_.update_state(state_);
  }

  void traj_callback(const sandbox_msgs::TrajectoryConstPtr &msg) {
    if (msg->target != target_ || msg->points.empty()) return;
    trajectory_.resize(msg->points.size());
    for (size_t i = 0; i < trajectory_.size(); ++i) {
      trajectory_[i].x = msg->points[i].x;
      trajectory_[i].y = msg->points[i].y;
      trajectory_[i].theta = msg->points[i].yaw;
      trajectory_[i].velocity = msg->points[i].velocity;  // ±1 方向标记
    }
    // 按速度符号切分多段(forward/reverse cusp)
    ranges_.clear();
    int start = 0;
    for (size_t i = 1; i < trajectory_.size(); ++i) {
      if (std::copysign(1.0, trajectory_[i].velocity) !=
          std::copysign(1.0, trajectory_[i - 1].velocity)) {
        ranges_.emplace_back(start, (int)i);
        start = (int)i;
      }
    }
    ranges_.emplace_back(start, (int)trajectory_.size());
    current_range_ = 0;
    loadRange(0);
    // 自动启动:目标=轨迹末点
    TrajectoryPoint goal{};
    goal.x = trajectory_.back().x; goal.y = trajectory_.back().y; goal.theta = trajectory_.back().theta;
    controller_.set_start(goal);
    trajectory_received_ = true;
  }

  void loadRange(int r) {
    std::vector<TrajectoryPoint> seg;
    for (int i = ranges_[r].first; i < ranges_[r].second; ++i) seg.push_back(trajectory_[i]);
    controller_.update_trajectory(seg);
  }

  // 车接近当前段末端 → 切下一段(多段 cusp:到 cusp 点换向继续)。
  void maybeSwitchRange() {
    if (current_range_ >= (int)ranges_.size() - 1) return;
    const auto &last = trajectory_[ranges_[current_range_].second - 1];
    if (std::hypot(last.x - state_.x, last.y - state_.y) <= 0.05) {
      ++current_range_;
      loadRange(current_range_);
    }
  }

  void control_callback(const ros::TimerEvent &) {
    if (!trajectory_received_) return;
    maybeSwitchRange();
    auto cmd = controller_.control();  // LQR 算横向 steering(其纵向 throttle 我们不用)
    sandbox_msgs::ChassisCommand msg;
    msg.target = target_;
    msg.steering = cmd.steering;  // 横向:LQR 状态反馈
    // 纵向:协调实时速度(带符号,STOP→0);未收到则退回 LQR 自身纵向(兼容旧单机)。
    msg.throttle = coord_speed_received_ ? coord_speed_ : cmd.throttle;
    twist_publisher_.publish(msg);
  }
};

int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "lqr_controller_node");
  ros::NodeHandle nh("~");
  LQRControllerNode node(nh);
  ros::spin();
  node.stop();
  return 0;
}
