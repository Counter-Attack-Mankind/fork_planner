#include <ros/ros.h>
#include <ros/package.h>

#include <algorithm>
#include <array>
#include <clocale>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <deque>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/marker_publisher.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/real_state_estimation.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/task_allocator.h"
#include "forklift_planner/multi_vehicle/traffic_resource_map.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"
#include "geometry_msgs/Point.h"
#include "sandbox_msgs/AprilObject.h"
#include "sandbox_msgs/Trajectory.h"
#include "sandbox_msgs/TrajectoryPoint.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

namespace {

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

}  // namespace

class MultiVehiclePatrolNode {
public:
    MultiVehiclePatrolNode() : nh_("~") {
        ros::NodeHandle param_nh;
        mp_ = MapParam::fromROSParam(param_nh);
        pp_ = PlannerParam::fromROSParam(param_nh);
        cfg_ = forklift_planner::multi_vehicle::MultiVehicleConfig::fromROSParam(
            param_nh);
        nh_.param("one_shot", one_shot_, one_shot_);
        const std::string planner_package =
            ros::package::getPath("forklift_planner");
        const std::string default_log_dir = planner_package.empty()
            ? "log"
            : (std::filesystem::path(planner_package).parent_path() / "log")
                  .string();
        nh_.param<std::string>("debug_log_dir", debug_log_dir_,
                               default_log_dir);
        nh_.param<std::string>("coord_log_file", coord_log_file_,
                               debug_log_dir_ +
                                   "/multi_vehicle_coordination.log");
        nh_.param("coord_log_enabled", coord_log_enabled_, true);
        nh_.param("stress_watchdog_enabled", stress_watchdog_enabled_, false);
        nh_.param("stress_quiet", stress_quiet_, false);
        nh_.param("stress_progress_timeout", stress_progress_timeout_, 120.0);
        nh_.param<std::string>("stress_result_file", stress_result_file_, "");
        nh_.param<std::string>("stress_failure_file", stress_failure_file_, "");
        nh_.param("snapshot_debug_enabled", snapshot_debug_enabled_, false);
        nh_.param("debug_timeline_start", debug_timeline_start_, -1.0);
        nh_.param("debug_timeline_end", debug_timeline_end_, -1.0);
        nh_.param<std::string>("snapshot_trigger_topic",
                               snapshot_trigger_topic_,
                               "/forklift_planner/debug_snapshot_trigger");
        if (snapshot_debug_enabled_) {
            snapshot_trigger_pub_ = nh_.advertise<std_msgs::String>(
                snapshot_trigger_topic_, 2, false);
        }
        onset_log_file_ = debug_log_dir_ + "/forklift_onset.log";
        realbridge_positions_file_ =
            debug_log_dir_ + "/realbridge_positions.txt";
        rb_horizon_ = cfg_.rolling_horizon;
        rb_horizon_refresh_period_ = cfg_.rolling_refresh_period;
        rb_horizon_refresh_ = std::max(
            1, static_cast<int>(std::lround(rb_horizon_refresh_period_ * pp_.update_rate)));
        rb_one_shot_traj_ = cfg_.one_shot_traj;
        if (cfg_.use_a1_cycle && rb_one_shot_traj_) {
            ROS_WARN("[multi_patrol][A1] one_shot_traj is incompatible with "
                     "task assignment at A1; forcing rolling-horizon mode");
            cfg_.one_shot_traj = false;
            rb_one_shot_traj_ = false;
        }
        initCoordLog();

        map_ = std::make_unique<ForkliftMap>(mp_);
        resource_map_ = std::make_unique<
            forklift_planner::multi_vehicle::TrafficResourceMap>(
            mp_, map_->slots(), map_->road_segments());
        generator_ = std::make_unique<PathGenerator>(mp_, pp_);
        allocator_ = std::make_unique<forklift_planner::multi_vehicle::TaskAllocator>(
            mp_, pp_, cfg_, *map_, *generator_);
        rule_engine_ = std::make_unique<forklift_planner::multi_vehicle::RuleEngine>(
            mp_, cfg_);
        const auto coord_log_sink = [this](const std::string& line) {
            coordLog(line);
        };
        allocator_->setCoordLogSink(coord_log_sink);
        rule_engine_->setCoordLogSink(coord_log_sink);
        setCoordLogContext("REAL", 0, -1, -1);
        rule_engine_->setResourceMap(resource_map_.get());
        // A方案:仿真也画每车完整轨迹。real 模式 setupRealIO 会再 advertise(同topic,无害)。
        horizon_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);

        if (stress_quiet_) {
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                           ros::console::levels::Error);
            ros::console::notifyLoggerLevelsChanged();
        }
        if (cfg_.precompute_task_filter) {
            allocator_->buildCache();
        }
        const Slot a1_pickup = allocator_->a1PickupSlot();
        marker_pub_ = std::make_unique<forklift_planner::multi_vehicle::MarkerPublisher>(
            nh_, mp_, pp_, map_->slots(), cfg_, a1_pickup);
        initAgents();
        if (cfg_.real_mode && !rb_one_shot_traj_) {
            initRealProjectionLogs();
        }
        visited_slots_.assign(map_->slots().size(), false);
        one_shot_done_.assign(agents_.size(), false);
        if (stress_quiet_) {
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                           ros::console::levels::Error);
            ros::console::notifyLoggerLevelsChanged();
        }
        dumpResourceSpans();  // Phase 1.3 验证:打印各车路径经过的资源占用区间

        if (cfg_.real_mode) {
            setupRealIO();  // 实车模式:建 /object 订阅 + /traj_i//coord_speed_i 发布,打印摆位
            timer_ = nh_.createTimer(ros::Duration(1.0 / pp_.update_rate),
                                     &MultiVehiclePatrolNode::tick, this);
            ROS_WARN("[multi_patrol] *** 实车模式 real_mode=true ***:位置取 /object,"
                     "发 /traj_i + /coord_speed_i;协调与 sim 一致。");
            return;  // 实车模式不走 batch
        }

        // 1. 无头批处理快速回归:~batch_ticks>0 时不建实时 timer,由 main 调 runBatch 狂跑。
        // 也支持 ~batch_minutes(按仿真分钟换算成拍数,更直观)。
        ros::NodeHandle pnh("~");
        int batch_ticks = 0;
        double batch_minutes = 0.0;
        pnh.param("batch_ticks", batch_ticks, 0);
        pnh.param("batch_minutes", batch_minutes, 0.0);
        if (batch_ticks <= 0 && batch_minutes > 0.0)
            batch_ticks = static_cast<int>(batch_minutes * 60.0 * pp_.update_rate);
        cfg_batch_ticks_ = batch_ticks > 0 ? static_cast<unsigned long long>(batch_ticks) : 0;
        if (cfg_batch_ticks_ > 0) {
            ROS_WARN("[batch] 无头快速回归模式:将狂跑 %llu 拍(≈%.0f 仿真分钟),"
                     "不发 marker、不按实时。", cfg_batch_ticks_,
                     cfg_batch_ticks_ / (pp_.update_rate * 60.0));
            return;  // 不建 timer
        }

        // 2. ROS实时模式，创建timer，按照步长调用tick()函数

        timer_ = nh_.createTimer(ros::Duration(1.0 / pp_.update_rate),
                                 &MultiVehiclePatrolNode::tick, this);

        ROS_INFO("[multi_patrol] started RViz timestamp simulation: vehicles=%d "
                 "seed=%d speed=%.2f max=%.2f dwell=%.2f horizon=%.2f step=%.2f",
                 cfg_.vehicle_count, cfg_.random_seed, cfg_.nominal_speed,
                 cfg_.max_speed, cfg_.dwell_time, cfg_.prediction_horizon,
                 cfg_.prediction_step);
    }

private:
    using VehicleAgent = forklift_planner::multi_vehicle::VehicleAgent;
    using VehicleAction = forklift_planner::multi_vehicle::VehicleAction;
    using VehicleMode = forklift_planner::multi_vehicle::VehicleMode;
    using MissionPhase = forklift_planner::multi_vehicle::MissionPhase;
    using LegTargetKind = forklift_planner::multi_vehicle::LegTargetKind;

    // Transitional simulation plan (stage 2 of the horizon refactor).
    // The planner still uses the existing sandbox world model for now, but
    // real simulation ticks execute these frozen decisions for one 2 s
    // commitment window instead of invoking ordinary arbitration again.
    struct SimPlannedAgentDecision {
        int path_gen = -1;
        VehicleMode mode = VehicleMode::NEED_TASK;
        VehicleAction action = VehicleAction::STOP;
        VehicleAction requested_action = VehicleAction::STOP;
        int blocker_id = -1;
        double wait_time = 0.0;
        double action_hold_remaining = 0.0;
        double ttc_stop_hold_remaining = 0.0;
        forklift_planner::multi_vehicle::RecoveryMotion planned_motion =
            forklift_planner::multi_vehicle::RecoveryMotion::NORMAL;
        double planned_motion_target_s = 0.0;
        std::string reason;
    };

    struct SimPlanFrame {
        std::vector<SimPlannedAgentDecision> agents;
        forklift_planner::multi_vehicle::RuleEngine::SimSnapshot rule_state;
        forklift_planner::multi_vehicle::RuleEngine::RollingDynamicDecision
            rolling_dynamic_decision;
    };

    // A real rolling period keeps only structural identity and the ordinary
    // decision selected from the latest measured state. Predicted frames are
    // never installed into the live real-vehicle executor.
    using RealPlanAgentIdentity =
        std::tuple<int, int, int, int, bool, int>;
    using DepartureTransactionIdentity =
        std::tuple<int, int, int, int, int, int, int, bool>;
    using RecoveryIdentity =
        std::tuple<int, int, int, int, int, double, double>;

    struct ExecutedRollingDecisionMetrics {
        unsigned long long far_periods = 0;
        unsigned long long mid_periods = 0;
        unsigned long long near_periods = 0;
        unsigned long long legacy_periods = 0;
        unsigned long long far_to_nominal = 0;
        unsigned long long mid_to_yield = 0;
        unsigned long long near_to_creep = 0;
        unsigned long long nominal_to_creep = 0;
        std::array<unsigned long long, 5> emergency_stop_from{};
        unsigned long long selected_rollout_clear = 0;
        unsigned long long yield_delayed = 0;
        unsigned long long creep_delayed = 0;
        unsigned long long eventually_resolved = 0;
        std::array<unsigned long long, 5> target_actions{};
        unsigned long long reservation_creates = 0;
        unsigned long long reservation_updates = 0;
        unsigned long long reservation_deletes = 0;
        unsigned long long existing_reservation_holds = 0;
        unsigned long long reservation_create_ordinary_dynamic = 0;
        unsigned long long reservation_create_a1 = 0;
        unsigned long long reservation_create_terminal = 0;
        unsigned long long reservation_create_already_inside = 0;
        unsigned long long reservation_create_braking_safety = 0;
        unsigned long long reservation_create_multi_vehicle = 0;
        unsigned long long reservation_create_other = 0;
    };

    struct A1LaunchMetrics {
        unsigned long long allows = 0;
        unsigned long long holds = 0;
        unsigned long long a1_prefix_holds = 0;
        unsigned long long ordinary_road_holds = 0;
        unsigned long long retries = 0;
        unsigned long long released_after_hold = 0;
        double max_hold_duration = 0.0;
    };

    struct A1LaunchHoldState {
        double since = 0.0;
        int blocker_id = -1;
        int candidate_path_gen = -1;
        std::string reason;
    };

    void initCoordLog() {
        if (!coord_log_enabled_) return;
        std::error_code error;
        std::filesystem::create_directories(debug_log_dir_, error);
        if (error) {
            ROS_WARN("[multi_patrol] failed to create debug log directory %s: %s",
                     debug_log_dir_.c_str(), error.message().c_str());
            return;
        }
        const std::filesystem::path log_path(coord_log_file_);
        if (log_path.has_parent_path()) {
            std::filesystem::create_directories(log_path.parent_path(), error);
        }
        if (error) {
            ROS_WARN("[multi_patrol] failed to create log directory %s: %s",
                     log_path.parent_path().string().c_str(),
                     error.message().c_str());
            return;
        }
        coord_log_.open(coord_log_file_, std::ios::out | std::ios::trunc);
        if (!coord_log_) {
            ROS_WARN("[multi_patrol] failed to open coordination log: %s",
                     coord_log_file_.c_str());
            return;
        }
        coord_log_ << "[multi_patrol] coordination log started\n";
        coord_log_ << "vehicle_count=" << cfg_.vehicle_count
                   << " one_shot=" << (one_shot_ ? 1 : 0)
                   << " use_a1_cycle=" << (cfg_.use_a1_cycle ? 1 : 0)
                   << "\n";
        coord_log_.flush();
        ROS_WARN("[multi_patrol] coordination log: %s",
                 coord_log_file_.c_str());
    }

    void initRealProjectionLogs() {
        std::error_code error;
        std::filesystem::create_directories(debug_log_dir_, error);
        if (error) {
            ROS_WARN("[real_projection] failed to create log directory %s: %s",
                     debug_log_dir_.c_str(), error.message().c_str());
            return;
        }
        real_projection_logs_.resize(agents_.size());
        for (size_t i = 0; i < agents_.size(); ++i) {
            const std::string path =
                debug_log_dir_ + "/real_projection_V" +
                std::to_string(agents_[i].id) + ".csv";
            real_projection_logs_[i].open(
                path, std::ios::out | std::ios::trunc);
            if (!real_projection_logs_[i]) {
                ROS_WARN("[real_projection] failed to open %s", path.c_str());
                continue;
            }
            real_projection_logs_[i]
                << "wall_time,sim_time,vehicle_id,real_x,real_y,real_yaw,"
                << "path_gen,mode,mission_phase,leg_target,previous_path_s,"
                << "projected_path_s,delta_s,projected_x,projected_y,"
                << "projected_yaw,projection_distance,"
                << "yaw_error_to_projected_path,wp_type,current_speed,action,"
                << "real_plan_id,search_s_min,search_s_max,"
                << "raw_single_step_speed,window_speed,speed_window_duration,"
                << "speed_window_samples,best_xy_distance,"
                << "selected_heading_error,selected_continuity_error,"
                << "projection_candidate_count\n";
            real_projection_logs_[i].flush();
            ROS_WARN("[real_projection] V%d log: %s", agents_[i].id,
                     path.c_str());
        }
    }

    void logRealProjectionSample(size_t i, double previous_path_s,
                                 double projected_path_s,
                                 double search_s_min,
                                 double search_s_max,
                                 const forklift_planner::multi_vehicle::
                                     ArcLengthSpeedResult& speed,
                                 const forklift_planner::multi_vehicle::
                                     RealProjectionResult& projection) {
        if (i >= agents_.size() || i >= real_projection_logs_.size() ||
            !real_projection_logs_[i] || !real_pose_ok_[i]) {
            return;
        }
        const VehicleAgent& v = agents_[i];
        const double nan = std::numeric_limits<double>::quiet_NaN();
        RoughWp projected{nan, nan, nan, WpType::FORWARD};
        std::string wp_type = "NONE";
        if (!v.track.empty()) {
            const double s = std::max(
                0.0, std::min(projected_path_s, v.track.length()));
            projected = v.track.poseAtS(s);
            wp_type = v.track.typeAtS(s) == WpType::REVERSE
                ? "REVERSE" : "FORWARD";
        }
        const double dx = projected.x - real_x_[i];
        const double dy = projected.y - real_y_[i];
        const double projection_distance = std::hypot(dx, dy);
        const double yaw_error = std::atan2(
            std::sin(real_yaw_[i] - projected.theta),
            std::cos(real_yaw_[i] - projected.theta));
        std::ofstream& log = real_projection_logs_[i];
        log << std::setprecision(15) << ros::Time::now().toSec() << ","
            << sim_time_ << "," << v.id << "," << real_x_[i] << ","
            << real_y_[i] << "," << real_yaw_[i] << "," << v.path_gen
            << "," << static_cast<int>(v.mode) << ","
            << static_cast<int>(v.mission_phase) << ","
            << static_cast<int>(v.leg_target) << "," << previous_path_s
            << "," << projected_path_s << ","
            << (projected_path_s - previous_path_s) << "," << projected.x
            << "," << projected.y << "," << projected.theta << ","
            << projection_distance << "," << yaw_error << "," << wp_type
            << "," << v.current_speed << "," << actionName(v.action) << ","
            << static_cast<unsigned long long>(sim_plan_id_) << ","
            << search_s_min << "," << search_s_max << ","
            << speed.raw_single_step_speed << "," << speed.window_speed << ","
            << speed.window_duration << "," << speed.window_samples << ","
            << projection.best_xy_distance << ","
            << projection.selected_heading_error << ","
            << projection.selected_continuity_error << ","
            << projection.candidate_count << "\n";
        log.flush();
    }

    void publishTrajectoryWithPathGen(
        size_t i, sandbox_msgs::Trajectory trajectory) {
        if (i >= traj_pubs_.size()) return;
        if (i < agents_.size()) {
            trajectory.header.seq = static_cast<uint32_t>(agents_[i].path_gen);
        }
        traj_pubs_[i].publish(trajectory);
    }

    void coordLog(const std::string& line) {
        coordLogWithContext(line, coord_log_source_, coord_log_plan_id_,
                            coord_log_frame_id_, coord_log_rollout_step_);
    }

    std::string contextualLog(const std::string& line,
                              const std::string& source,
                              uint64_t plan_id, int frame_id,
                              int rollout_step) const {
        char prefix[160];
        std::snprintf(prefix, sizeof(prefix),
                      "[SOURCE=%s] [plan=%llu] [frame=%d] "
                      "[rollout_step=%d] ",
                      source.c_str(),
                      static_cast<unsigned long long>(plan_id), frame_id,
                      rollout_step);
        return std::string(prefix) + line;
    }

    void coordLogWithContext(const std::string& line,
                             const std::string& source,
                             uint64_t plan_id, int frame_id,
                             int rollout_step) {
        if (!coord_log_ || coord_log_suppressed_) return;
        coord_log_ << contextualLog(line, source, plan_id, frame_id,
                                    rollout_step)
                   << "\n";
        coord_log_.flush();
    }

    void setCoordLogContext(const std::string& source, uint64_t plan_id,
                            int frame_id, int rollout_step) {
        coord_log_source_ = source;
        coord_log_plan_id_ = plan_id;
        coord_log_frame_id_ = frame_id;
        coord_log_rollout_step_ = rollout_step;
        if (rule_engine_) {
            rule_engine_->setDebugLogContext(source, plan_id, frame_id,
                                             rollout_step);
        }
    }

    std::string readableSimTime(double seconds) const {
        const double nonnegative = std::max(0.0, seconds);
        const long long tenths =
            static_cast<long long>(std::llround(nonnegative * 10.0));
        const long long minutes = tenths / 600;
        const double remainder = static_cast<double>(tenths % 600) / 10.0;
        char text[80];
        std::snprintf(text, sizeof(text), "%lldmin%.1fs", minutes, remainder);
        return text;
    }

    const char* modeName(VehicleMode mode) const {
        switch (mode) {
            case VehicleMode::NEED_TASK: return "NEED_TASK";
            case VehicleMode::ACTIVE: return "ACTIVE";
            case VehicleMode::DWELL: return "DWELL";
        }
        return "UNKNOWN";
    }

    const char* missionPhaseName(MissionPhase phase) const {
        switch (phase) {
            case MissionPhase::DIRECT_TO_B: return "DIRECT_TO_B";
            case MissionPhase::TO_A1: return "TO_A1";
            case MissionPhase::PICKUP_DWELL: return "PICKUP_DWELL";
            case MissionPhase::WAIT_DROPOFF_TASK: return "WAIT_DROPOFF_TASK";
            case MissionPhase::TO_B: return "TO_B";
            case MissionPhase::UNLOAD_DWELL: return "UNLOAD_DWELL";
        }
        return "UNKNOWN";
    }

    const char* legTargetName(LegTargetKind target) const {
        switch (target) {
            case LegTargetKind::B_SLOT: return "B_SLOT";
            case LegTargetKind::A1: return "A1";
        }
        return "UNKNOWN";
    }

    void initAgents() {
        agents_.clear();
        agents_.reserve(static_cast<size_t>(cfg_.vehicle_count));

        const std::array<std_msgs::ColorRGBA, 8> colors = {
            rgba(0.00f, 0.28f, 0.82f),
            rgba(0.78f, 0.04f, 0.04f),
            rgba(0.00f, 0.52f, 0.16f),
            rgba(0.78f, 0.48f, 0.00f),
            rgba(0.46f, 0.16f, 0.78f),
            rgba(0.00f, 0.52f, 0.58f),
            rgba(0.82f, 0.24f, 0.00f),
            rgba(0.76f, 0.00f, 0.46f),
        };

        std::mt19937 rng(static_cast<unsigned int>(cfg_.random_seed));
        const int slot_count = static_cast<int>(map_->slots().size());

        // A1-cycle 的起点必须存在有效 B->A1 航段；普通模式要求能出库，
        // simple_forward_demo 还要求至少存在一个全程前进目标。
        auto startOK = [&](int s) {
            if (cfg_.use_a1_cycle) return allocator_->hasValidPickupLeg(s);
            if (!allocator_->hasValidOutbound(s)) return false;
            if (cfg_.simple_forward_demo && !allocator_->hasForwardTarget(s)) return false;
            return true;
        };

        // 简单测试版一次性诊断:贪心算 want 对互不冲突(起点≠终点、起点两两不同、终点两两不同)的
        // 无尖点前进对,分别按「最短(走得近)」和「最长(走得远)」各算一套,打印供离线预设到 start/target_slots。
        if (cfg_.simple_forward_demo) {
            const int want = std::max(1, cfg_.vehicle_count);
            auto greedyMatch = [&](bool prefer_long) {
                std::vector<int> cs, ct; std::vector<double> cl;
                std::vector<bool> us(slot_count, false), ut(slot_count, false);
                for (int iter = 0; iter < want; ++iter) {
                    int bs = -1, bt = -1; double blen = prefer_long ? -1.0 : 1e18;
                    for (int s = 0; s < slot_count; ++s) {
                        if (us[s] || ut[s] || !startOK(s)) continue;
                        std::vector<int> ft; std::vector<double> fl;
                        allocator_->forwardTargets(s, ft, &fl);  // 实际路径弧长
                        for (size_t k = 0; k < ft.size(); ++k) {
                            const int t = ft[k];
                            if (us[t] || ut[t] || t == s) continue;
                            const bool better = prefer_long ? (fl[k] > blen) : (fl[k] < blen);
                            if (better) { blen = fl[k]; bs = s; bt = t; }
                        }
                    }
                    if (bs < 0) break;
                    us[bs] = ut[bt] = true; cs.push_back(bs); ct.push_back(bt); cl.push_back(blen);
                }
                std::string ss, ts; double tot = 0.0;
                for (size_t k = 0; k < cs.size(); ++k) {
                    ss += std::to_string(cs[k]) + (k+1<cs.size()?", ":"");
                    ts += std::to_string(ct[k]) + (k+1<ct.size()?", ":"");
                    tot += cl[k];
                }
                ROS_WARN("[simple-scan] %s %zu 对(总长%.2fm):", prefer_long?"【走得远】":"【走得近】", cs.size(), tot);
                for (size_t k = 0; k < cs.size(); ++k)
                    ROS_WARN("[simple-scan]   对%zu: %d -> %d  len=%.3f", k+1, cs[k], ct[k], cl[k]);
                ROS_WARN("[simple-scan]   start_slots:  [%s]", ss.c_str());
                ROS_WARN("[simple-scan]   target_slots: [%s]", ts.c_str());
            };
            greedyMatch(false);  // 短
            greedyMatch(true);   // 长
        }

        std::vector<int> random_starts;
        if (cfg_.randomize_start) {
            for (int s = 0; s < slot_count; ++s) {
                if (startOK(s)) random_starts.push_back(s);
            }
            std::shuffle(random_starts.begin(), random_starts.end(), rng);
        }

        std::vector<bool> used(static_cast<size_t>(slot_count), false);
        for (int i = 0; i < cfg_.vehicle_count; ++i) {
            VehicleAgent v;
            const int vehicle_id = cfg_.real_mode
                ? cfg_.vehicle_ids.at(static_cast<size_t>(i)) : i;
            v.id = vehicle_id;
            int start_slot;
            if (cfg_.randomize_start && !random_starts.empty()) {
                start_slot = random_starts[static_cast<size_t>(i) %
                                           random_starts.size()];
            } else {
                const size_t slot_config_index = cfg_.real_mode
                    ? static_cast<size_t>(vehicle_id)
                    : static_cast<size_t>(i);
                start_slot = cfg_.start_slots.empty()
                    ? vehicle_id
                    : cfg_.start_slots[slot_config_index %
                                       cfg_.start_slots.size()];
            }
            start_slot = ((start_slot % slot_count) + slot_count) % slot_count;

            // 简单测试版下 startOK 还要求该起点有「全程前进(无尖点)」目标,保证每车都能一把开进。
            if (used[start_slot] || !startOK(start_slot)) {
                int repl = -1;
                for (int s = 0; s < slot_count; ++s) {
                    if (!used[s] && startOK(s)) {
                        repl = s;
                        break;
                    }
                }
                if (repl >= 0) {
                    if (!startOK(start_slot)) {
                        ROS_WARN("[multi_patrol] start slot %d 不可用"
                                 "(陷阱/无前进目标); V%d 改从 slot %d 起步",
                                 start_slot, vehicle_id, repl);
                    }
                    start_slot = repl;
                }
            }
            used[start_slot] = true;

            v.current_slot = start_slot;
            v.target_slot = v.current_slot;
            if (cfg_.use_a1_cycle) {
                v.loaded = false;
                v.mission_phase = MissionPhase::TO_A1;
                v.leg_target = LegTargetKind::A1;
            } else {
                std::bernoulli_distribution load_dist(0.5);
                v.loaded = load_dist(rng);
                v.mission_phase = MissionPhase::DIRECT_TO_B;
                v.leg_target = LegTargetKind::B_SLOT;
            }
            v.color = colors[static_cast<size_t>(vehicle_id) % colors.size()];
            v.mode = VehicleMode::NEED_TASK;
            agents_.push_back(v);
        }

        const int enabled_count = static_cast<int>(agents_.size());
        int assigned_count = 0;
        for (VehicleAgent& v : agents_) {
            const bool assigned = cfg_.use_a1_cycle
                ? launchPickupLegWithA1Admission(v)
                : allocator_->assignNextTask(v, agents_);
            if (assigned) {
                ++assigned_count;
            }
        }
        if (cfg_.use_a1_cycle) {
            ROS_INFO("[multi_patrol][A1] initial pickup legs assigned: %d/%d",
                     assigned_count, enabled_count);
            if (assigned_count != enabled_count) {
                ROS_ERROR("[multi_patrol][A1] %d enabled vehicle(s) have no "
                          "initial B->A1 task; inspect the preceding path errors",
                          enabled_count - assigned_count);
            }
        }
        force_horizon_refresh_ = true;
        resetStatusLogState();
    }

    // Phase 1.3 验证:对每辆 active 车,打印其固定路径经过的资源及弧长区间,
    // 用于核对资源地图(SLOT_BODY/DOCK 等)与实际路径一致。一次性。
    void dumpResourceSpans() {
        ROS_INFO("[res_map] resources built: %zu",
                 resource_map_->resources().size());
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) continue;
            const auto spans = resource_map_->spansForPath(v.track);
            for (const auto& sp : spans) {
                const auto* r = resource_map_->byId(sp.resource_id);
                ROS_INFO("[res_map] V%d uses %s s=[%.3f,%.3f] (len=%.3f)",
                         v.id, r ? r->name.c_str() : "?",
                         sp.s_enter, sp.s_exit, v.track.length());
            }
        }
    }

    void resetStatusLogState() {
        const size_t n = agents_.size();
        last_logged_mode_.assign(n, VehicleMode::NEED_TASK);
        last_logged_action_.assign(n, VehicleAction::STOP);
        last_logged_reason_.assign(n, "");
        last_logged_blocker_.assign(n, -999);
        last_logged_task_count_.assign(n, -1);
        last_logged_mission_phase_.assign(n, MissionPhase::DIRECT_TO_B);
        last_status_log_time_.assign(n, ros::Time(0));
        last_diag_time_.assign(n, ros::Time(0));
    }

    double limitedSpeed(double current, double desired, double dt) const {
        if (desired > current) {
            return std::min(desired, current + cfg_.max_accel * dt);
        }
        return std::max(desired, current - cfg_.max_decel * dt);
    }

    RoughWp poseForCollision(const VehicleAgent& v, double path_s) const {
        if (!v.track.empty()) {
            if (v.mode == VehicleMode::DWELL) {
                return v.track.poseAtS(v.track.length());
            }
            return v.track.poseAtS(std::min(path_s, v.track.length()));
        }
        // 无轨迹的 idle 车(NEED_TASK)仍物理停在 current_slot 上，别的车不能从它身上
        // 碾过去。车身中心在 dock，朝向朝库外(pre_dock 方向)，参考点沿鼻向后移 d。
        const Slot& s = map_->slots().at(static_cast<size_t>(v.current_slot));
        const double th = std::atan2(s.pre_dock_y - s.dock_y(),
                                     s.pre_dock_x - s.dock_x());
        RoughWp p;
        p.x = s.dock_x() - mp_.rear_axle_to_center * std::cos(th);
        p.y = s.dock_y() - mp_.rear_axle_to_center * std::sin(th);
        p.theta = th;
        p.type = WpType::FORWARD;
        return p;
    }

    void rollWorldModel(double horizon, std::vector<sandbox_msgs::Trajectory>& out,
                        std::vector<bool>& hold,
                        std::vector<SimPlanFrame>* plan_frames = nullptr,
                        bool first_step_task_state_is_current = false,
                        std::vector<size_t>* path_gen_cut_indices = nullptr) {

        const double dt = 1.0 / pp_.update_rate;            //系统每触发一次，就向前推进dt时间
        const int H = std::max(1, (int)std::lround(horizon / dt));      //四舍五入决定仿真步数，但至少模拟1步

        //=====（初始化轨迹，全部置空并且默认车辆均为静止状态）===========
        const size_t n = agents_.size();
        out.assign(n, sandbox_msgs::Trajectory{});
        hold.assign(n, true);
        const std::string previous_log_source = coord_log_source_;
        const uint64_t previous_log_plan = coord_log_plan_id_;
        const int previous_log_frame = coord_log_frame_id_;
        const int previous_rollout_step = coord_log_rollout_step_;
        const uint64_t rollout_plan_id =
            plan_frames != nullptr ? sim_plan_id_ + 1 : ++rollout_log_id_;
        setCoordLogContext("ROLLOUT", rollout_plan_id, 0, 0);
        //初始化轨迹，所有离散点中的目标点按序排列，并且坐标系设为世界坐标系
        for (size_t i = 0; i < n; ++i) {
            out[i].target = agents_[i].id;
            out[i].header.frame_id = "world";
        }

        //===========（状态快照与回滚）============
        const std::vector<VehicleAgent> sa = agents_;    //（将Agents通过拷贝构造函数给sa，sa设为const，后续仅改变备份的agents，对显示不产生影响）  
        std::vector<int> live_path_gen(n, -1);
        for (size_t i = 0; i < n; ++i) live_path_gen[i] = sa[i].path_gen;
        if (path_gen_cut_indices != nullptr) {
            path_gen_cut_indices->assign(
                n, std::numeric_limits<size_t>::max());
        }
        const std::vector<bool> sv = visited_slots_;
        const auto sr = rule_engine_->snapshot();       //保存规则引擎状态
        const auto live_a1_intrusion_corrections =
            rule_engine_->captureLiveA1IntrusionCorrections();
        const auto sl = allocator_->snapshot();         //保存任务分配器状态
        const bool prev = sim_mode_;                    //保存现在模式（仿真或是实际）
        sim_mode_ = true;       //切换到仿真模式，因为现在属于提前规划，必须视为仿真

        //=========（创建匿名函数recored，用来记录当前车辆状态，并每拍给每辆车生成一个TrajPoint）========
        auto record = [&](int s) {
            for (size_t i = 0; i < n; ++i) {
                const VehicleAgent& v = agents_[i];         //对每辆车都记录一次当前状态，v是agents_[i] 的只读引用
                const RoughWp p = poseForCollision(v, v.path_s);    //根据车辆v当前走到路径上的距离 path_s，求它当前在地图里的姿态
                // 几何判前进/倒车(用户判据):存的航向(=车头)与路径切向(=前进方向)反向→倒车。
                // 不靠 typeAtS 标签(可能没标对)。前进段速度取正、倒车段取负——就这么简单。
                
                sandbox_msgs::TrajectoryPoint tp;
                tp.x = p.x; tp.y = p.y; tp.yaw = p.theta;                          // 车头朝向
                const bool retreat =
                    rule_engine_->motionOverrideFor(v.id).motion ==
                    forklift_planner::multi_vehicle::RecoveryMotion::RETREAT;
                const double motion_sign =
                    forklift_planner::multi_vehicle::signedPathMotionDirection(
                        v.track, v.path_s, retreat ? -1 : 1);
                tp.velocity = motion_sign * std::max(0.0, v.current_speed);
                tp.time = s * dt;
                if (path_gen_cut_indices != nullptr &&
                    (*path_gen_cut_indices)[i] ==
                        std::numeric_limits<size_t>::max() &&
                    v.path_gen != live_path_gen[i]) {
                    (*path_gen_cut_indices)[i] = out[i].points.size();
                }
                out[i].points.push_back(tp);
                if (v.current_speed > 1e-3) hold[i] = false;   // 整段都不动才算 hold
            }
        };

        //=====（将刚才判断记录的，作为预测的第0个点）====
        record(0);
        if (plan_frames != nullptr) {
            plan_frames->clear();
            plan_frames->reserve(static_cast<size_t>(H));
        }
        forklift_planner::multi_vehicle::RuleEngine::RollingDynamicDecision
            period_ordinary_decision;

        //开始向未来预测H步长，H = 预测时间/dt
        for (int s = 1; s <= H; ++s) {
            setCoordLogContext("ROLLOUT", rollout_plan_id, s - 1, s);
            // The simulation executor calls updateDwellAndTasks() before it
            // asks for a new plan. Its first planned control therefore starts
            // from that already-updated task state; later frames advance the
            // task state normally. Existing real rollout keeps the old order.
            if (!(first_step_task_state_is_current && s == 1)) {
                updateDwellAndTasks(dt);
            }
            const bool reuse_ordinary_coordination =
                plan_frames != nullptr && s > 1;
            if (plan_frames != nullptr) {
                // All decisions used to build this active plan share one
                // prediction end time. At future offset tau, only the
                // remaining prediction horizon stays visible; never open a
                // fresh full window at every future step.
                const double tau = static_cast<double>(s - 1) * dt;
                const double remaining_horizon =
                    std::max(dt, cfg_.prediction_horizon - tau);
                rule_engine_->decide(agents_, dt, remaining_horizon,
                                     reuse_ordinary_coordination,
                                     reuse_ordinary_coordination
                                         ? &period_ordinary_decision : nullptr);
            } else {
                rule_engine_->decide(agents_, dt);
            }
            if (plan_frames != nullptr && s == 1) {
                period_ordinary_decision =
                    rule_engine_->lastRollingDynamicDecision();
            }
            if (plan_frames != nullptr) {
                SimPlanFrame frame;
                frame.agents.reserve(agents_.size());
                for (const VehicleAgent& v : agents_) {
                    SimPlannedAgentDecision d;
                    d.path_gen = v.path_gen;
                    d.mode = v.mode;
                    d.action = v.action;
                    d.requested_action = v.requested_action;
                    d.blocker_id = v.blocker_id;
                    d.wait_time = v.wait_time;
                    d.action_hold_remaining = v.action_hold_remaining;
                    d.ttc_stop_hold_remaining =
                        v.ttc_stop_hold_remaining;
                    const auto motion = rule_engine_->motionOverrideFor(v.id);
                    d.planned_motion = motion.motion;
                    d.planned_motion_target_s = motion.target_s;
                    d.reason = v.reason;
                    frame.agents.push_back(std::move(d));
                }
                frame.rule_state = rule_engine_->snapshot();
                if (s == 1) {
                    frame.rolling_dynamic_decision =
                        rule_engine_->lastRollingDynamicDecision();
                }
                plan_frames->push_back(std::move(frame));
            }
            advanceVehicles(dt);
            record(s);
        }

        //将沙盒预测造成所有的改动恢复
        agents_ = sa;
        visited_slots_ = sv;
        coord_log_suppressed_ = true;
        rule_engine_->restore(sr);
        rule_engine_->restoreLiveA1IntrusionCorrections(
            live_a1_intrusion_corrections);
        allocator_->restore(sl);
        coord_log_suppressed_ = false;
        sim_mode_ = prev;
        setCoordLogContext(previous_log_source, previous_log_plan,
                           previous_log_frame, previous_rollout_step);
    }

    std::vector<RealPlanAgentIdentity> captureRealPlanAgentIdentity() const {
        std::vector<RealPlanAgentIdentity> identity;
        identity.reserve(agents_.size());
        for (const VehicleAgent& v : agents_) {
            identity.emplace_back(
                v.path_gen, static_cast<int>(v.mode),
                static_cast<int>(v.mission_phase),
                static_cast<int>(v.leg_target), v.pending_dropoff_valid,
                v.pending_dropoff_slot);
        }
        return identity;
    }

    std::vector<DepartureTransactionIdentity>
    captureDepartureTransactionIdentity() const {
        return rule_engine_->a1DepartureTransactionIdentity();
    }

    RecoveryIdentity captureRecoveryIdentity() const {
        const auto& recovery = rule_engine_->recoveryDirective();
        return std::make_tuple(
            static_cast<int>(recovery.phase), recovery.retreat_vehicle_id,
            recovery.pass_vehicle_id, recovery.retreat_path_gen,
            recovery.pass_path_gen, recovery.retreat_target_s,
            recovery.retreat_distance);
    }

    void rememberRealPlanIdentity() {
        real_plan_agents_ = captureRealPlanAgentIdentity();
        real_plan_departure_transactions_ =
            captureDepartureTransactionIdentity();
        real_plan_recovery_ = captureRecoveryIdentity();
    }

    bool realPlanNeedsRefresh() const {
        if (!real_plan_valid_ || force_horizon_refresh_) return true;
        if (sim_time_ - real_plan_start_time_ >=
            rb_horizon_refresh_period_ - 1e-9) {
            return true;
        }
        const auto& future_a1_commitment =
            rule_engine_->futureA1Commitment();
        if (future_a1_commitment.valid()) {
            const VehicleAgent* owner =
                agentById_c(future_a1_commitment.owner_id);
            if (owner == nullptr ||
                owner->path_gen != future_a1_commitment.owner_path_gen) {
                return true;
            }
        }
        if (captureRealPlanAgentIdentity() != real_plan_agents_) return true;
        if (captureRecoveryIdentity() != real_plan_recovery_) return true;
        return captureDepartureTransactionIdentity() !=
               real_plan_departure_transactions_;
    }

    void buildRollingHorizonPlan(
        std::vector<sandbox_msgs::Trajectory>& trajs,
        std::vector<bool>& hold, bool install_simulation_plan,
        std::vector<size_t>* path_gen_cut_indices = nullptr) {
        // Normal rolling simulation prepares previews in publishHorizon(),
        // while batch mode calls this function directly. Keep both paths
        // behaviorally identical.
        prepareA1DropoffPreviewsForHorizon();

        forklift_planner::multi_vehicle::RuleEngine::A1ArrivalKinematics
            a1_kinematics;
        a1_kinematics.dt = 1.0 / pp_.update_rate;
        a1_kinematics.enabled = [](int) { return true; };
        a1_kinematics.desired_speed = [this](const VehicleAgent& vehicle) {
            return std::min(
                rule_engine_->speedForAction(VehicleAction::NOMINAL),
                curvatureSpeed(vehicle));
        };
        a1_kinematics.limited_speed =
            [this](double current_speed, double desired_speed, double dt) {
                return limitedSpeed(current_speed, desired_speed, dt);
            };
        a1_kinematics.pickup_leg_track =
            [this](int slot,
                   forklift_planner::multi_vehicle::PathTrack& out) {
                return allocator_->getPickupLegTrack(slot, out);
            };
        rule_engine_->refreshA1PlanningContext(
            agents_, cfg_.a1_owner_horizon, sim_time_, a1_kinematics);

        std::vector<SimPlanFrame> frames;
        rollWorldModel(rb_horizon_, trajs, hold, &frames,
                       /*first_step_task_state_is_current=*/true,
                       path_gen_cut_indices);
        const size_t frame_count = frames.size();
        ++sim_plan_id_;
        if (install_simulation_plan) {
            sim_plan_frames_ = std::move(frames);
            sim_plan_cursor_ = 0;
            sim_plan_valid_ = !sim_plan_frames_.empty();
            sim_plan_start_time_ = sim_time_;
        } else {
            real_plan_valid_ = !frames.empty();
            real_plan_start_time_ = sim_time_;
            if (real_plan_valid_) {
                real_period_ordinary_decision_ =
                    frames.front().rolling_dynamic_decision;
            } else {
                real_period_ordinary_decision_ = {};
            }
            rememberRealPlanIdentity();
        }
        if (debug_timeline_start_ >= 0.0 &&
            sim_time_ >= debug_timeline_start_ - rb_horizon_ &&
            sim_time_ <= debug_timeline_end_) {
            for (size_t vehicle = 0; vehicle < trajs.size(); ++vehicle) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(3)
                     << "[TIMELINE_PRED] plan=" << sim_plan_id_
                     << " start=" << sim_time_ << " V" << vehicle;
                const auto& points = trajs[vehicle].points;
                const size_t stride = std::max<size_t>(
                    1, static_cast<size_t>(std::lround(pp_.update_rate)));
                for (size_t sample = 0; sample < points.size(); sample += stride) {
                    const auto& point = points[sample];
                    line << " t" << point.time << "=(" << point.x << ","
                         << point.y << ",v=" << point.velocity << ")";
                }
                coordLog(line.str());
            }
        }
        ROS_INFO("[%s_plan] built plan=%llu start=%.2f horizon=%.2f "
                 "frames=%zu commit_frames=%d",
                 install_simulation_plan ? "sim" : "real",
                 static_cast<unsigned long long>(sim_plan_id_),
                 sim_time_, rb_horizon_, frame_count,
                 rb_horizon_refresh_);
        for (const VehicleAgent& v : agents_) {
            if (v.mode != VehicleMode::DWELL ||
                v.mission_phase != MissionPhase::PICKUP_DWELL ||
                !v.pending_dropoff_valid) {
                continue;
            }
            ROS_WARN("[multi_patrol][A1 EXIT HORIZON] V%d target=B%d "
                     "dwell_part=%.2fs departure_window=%.2fs "
                     "total_horizon=%.2fs",
                     v.id, v.pending_dropoff_slot,
                     std::min(v.dwell_remaining, rb_horizon_),
                     std::max(0.0, rb_horizon_ - v.dwell_remaining),
                     rb_horizon_);
        }
    }

    void buildSimulationHorizonPlan(
        std::vector<sandbox_msgs::Trajectory>& trajs,
        std::vector<bool>& hold) {
        buildRollingHorizonPlan(trajs, hold,
                                /*install_simulation_plan=*/true);
    }

    void buildRealHorizonPlan(
        std::vector<sandbox_msgs::Trajectory>& trajs,
        std::vector<bool>& hold,
        std::vector<size_t>& path_gen_cut_indices) {
        buildRollingHorizonPlan(trajs, hold,
                                /*install_simulation_plan=*/false,
                                &path_gen_cut_indices);
    }

    bool simulationPlanNeedsRefresh() const {
        if (!sim_plan_valid_ || force_horizon_refresh_) return true;
        const auto& future_a1_commitment =
            rule_engine_->futureA1Commitment();
        if (future_a1_commitment.valid()) {
            const VehicleAgent* owner =
                agentById_c(future_a1_commitment.owner_id);
            if (owner == nullptr ||
                owner->path_gen != future_a1_commitment.owner_path_gen) {
                return true;
            }
        }
        if (sim_plan_cursor_ >= sim_plan_frames_.size()) return true;
        if (sim_plan_cursor_ >= static_cast<size_t>(rb_horizon_refresh_)) {
            return true;
        }
        const SimPlanFrame& frame = sim_plan_frames_[sim_plan_cursor_];
        if (frame.agents.size() != agents_.size()) return true;
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (frame.agents[i].path_gen != agents_[i].path_gen ||
                frame.agents[i].mode != agents_[i].mode) {
                return true;
            }
        }
        return false;
    }

    bool executeSimulationPlanSample() {
        if (simulationPlanNeedsRefresh()) return false;
        setCoordLogContext("REAL", sim_plan_id_,
                           static_cast<int>(sim_plan_cursor_), -1);
        const SimPlanFrame& frame = sim_plan_frames_[sim_plan_cursor_];
        const auto before_rule_state = rule_engine_->snapshot();
        for (const auto& incoming : frame.rule_state.reservations) {
            const auto before =
                before_rule_state.reservations.find(incoming.first);
            if (before == before_rule_state.reservations.end()) {
                ++executed_rolling_metrics_.reservation_creates;
                const std::string& reason = incoming.second.create_reason;
                if (reason == "ordinary_dynamic") {
                    ++executed_rolling_metrics_.
                        reservation_create_ordinary_dynamic;
                } else if (reason == "a1_related") {
                    ++executed_rolling_metrics_.reservation_create_a1;
                } else if (reason == "terminal") {
                    ++executed_rolling_metrics_.reservation_create_terminal;
                } else if (reason == "already_inside") {
                    ++executed_rolling_metrics_.
                        reservation_create_already_inside;
                } else if (reason == "braking_safety") {
                    ++executed_rolling_metrics_.
                        reservation_create_braking_safety;
                } else if (reason == "multi_vehicle_legacy") {
                    ++executed_rolling_metrics_.
                        reservation_create_multi_vehicle;
                } else {
                    ++executed_rolling_metrics_.reservation_create_other;
                }
            } else {
                ++executed_rolling_metrics_.existing_reservation_holds;
                if (before->second.owner_id != incoming.second.owner_id) {
                    ++executed_rolling_metrics_.reservation_updates;
                }
            }
        }
        for (const auto& current : before_rule_state.reservations) {
            if (frame.rule_state.reservations.count(current.first) == 0) {
                ++executed_rolling_metrics_.reservation_deletes;
            }
        }
        rule_engine_->restore(frame.rule_state, false);
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            const SimPlannedAgentDecision& d = frame.agents[i];
            v.action = d.action;
            v.requested_action = d.requested_action;
            v.blocker_id = d.blocker_id;
            v.wait_time = d.wait_time;
            v.action_hold_remaining = d.action_hold_remaining;
            v.ttc_stop_hold_remaining = d.ttc_stop_hold_remaining;
            v.reason = d.reason;
        }
        const RecoveryIdentity recovery_before = captureRecoveryIdentity();
        const double live_dt = 1.0 / pp_.update_rate;
        rule_engine_->refreshA1IntrusionCorrections(agents_, live_dt);
        for (size_t i = 0; i < agents_.size(); ++i) {
            const auto live_motion =
                rule_engine_->motionOverrideFor(agents_[i].id);
            if (live_motion.motion != frame.agents[i].planned_motion ||
                (live_motion.motion == forklift_planner::multi_vehicle::
                                           RecoveryMotion::RETREAT &&
                 std::abs(live_motion.target_s -
                          frame.agents[i].planned_motion_target_s) > 1e-9)) {
                force_horizon_refresh_ = true;
            }
        }
        rule_engine_->observeDeadlock(agents_, live_dt, true);
        rule_engine_->applyRecoveryDirectiveToOutput(agents_);
        if (captureRecoveryIdentity() != recovery_before) {
            force_horizon_refresh_ = true;
        }
        if (sim_plan_cursor_ == 0) {
            const auto& decision = frame.rolling_dynamic_decision;
            marker_pub_->setRollingDecision(decision);
            if (previous_ordinary_conflict_active_ &&
                decision.baseline_evaluated && !decision.valid) {
                ++executed_rolling_metrics_.eventually_resolved;
            }
            if (decision.baseline_evaluated) {
                previous_ordinary_conflict_active_ = decision.valid;
            }
            if (decision.valid) {
                switch (decision.band) {
                    case forklift_planner::multi_vehicle::
                        DynamicInterventionBand::FAR:
                        ++executed_rolling_metrics_.far_periods;
                        break;
                    case forklift_planner::multi_vehicle::
                        DynamicInterventionBand::MID:
                        ++executed_rolling_metrics_.mid_periods;
                        break;
                    case forklift_planner::multi_vehicle::
                        DynamicInterventionBand::NEAR:
                        ++executed_rolling_metrics_.near_periods;
                        break;
                }
                if (decision.legacy_fallback) {
                    ++executed_rolling_metrics_.legacy_periods;
                }
                if (decision.band == forklift_planner::multi_vehicle::
                                         DynamicInterventionBand::FAR &&
                    decision.selected_action_a == VehicleAction::NOMINAL &&
                    decision.selected_action_b == VehicleAction::NOMINAL) {
                    ++executed_rolling_metrics_.far_to_nominal;
                }
                if (decision.band == forklift_planner::multi_vehicle::
                                         DynamicInterventionBand::MID &&
                    (decision.selected_action_a == VehicleAction::YIELD ||
                     decision.selected_action_b == VehicleAction::YIELD)) {
                    ++executed_rolling_metrics_.mid_to_yield;
                }
                if (decision.band == forklift_planner::multi_vehicle::
                                         DynamicInterventionBand::NEAR &&
                    (decision.selected_action_a == VehicleAction::CREEP ||
                     decision.selected_action_b == VehicleAction::CREEP)) {
                    ++executed_rolling_metrics_.near_to_creep;
                }
                for (const auto& target : decision.targets) {
                    if (target.action == VehicleAction::CREEP &&
                        target.previous_action == VehicleAction::NOMINAL) {
                        ++executed_rolling_metrics_.nominal_to_creep;
                    }
                    if (target.action == VehicleAction::STOP) {
                        const size_t previous_index =
                            static_cast<size_t>(target.previous_action);
                        if (previous_index < executed_rolling_metrics_.
                                                 emergency_stop_from.size()) {
                            ++executed_rolling_metrics_.
                                emergency_stop_from[previous_index];
                        }
                    }
                }
            }
            std::ostringstream line;
            line << "[ROLLING-DECISION] plan=" << sim_plan_id_ << " band="
                 << (decision.valid
                         ? forklift_planner::multi_vehicle::
                               dynamicInterventionBandName(decision.band)
                         : "CLEAR_OR_SPECIAL")
                 << " targets=";
            for (size_t i = 0; i < frame.agents.size(); ++i) {
                const VehicleAction target = frame.agents[i].requested_action;
                const size_t action_index = static_cast<size_t>(target);
                if (action_index < executed_rolling_metrics_.target_actions.size()) {
                    ++executed_rolling_metrics_.target_actions[action_index];
                }
                if (i != 0) line << "/";
                line << actionName(target);
            }
            line << " legacy=" << (decision.legacy_fallback ? 1 : 0);
            if (decision.baseline_first_overlap_t) {
                line << " baseline_first_overlap_t="
                     << *decision.baseline_first_overlap_t;
            }
            line << " candidate_rollout=NOT_EVALUATED";
            coordLogWithContext(line.str(), "REAL", sim_plan_id_, 0, -1);
        }
        ++sim_plan_cursor_;
        return true;
    }

    bool executeRealRollingDecision(double dt) {
        if (!real_plan_valid_) return false;

        std::vector<std::tuple<bool, int, double>> motion_before;
        motion_before.reserve(agents_.size());
        for (const VehicleAgent& vehicle : agents_) {
            const auto motion = rule_engine_->motionOverrideFor(vehicle.id);
            motion_before.emplace_back(
                motion.a1_intrusion, static_cast<int>(motion.motion),
                motion.target_s);
        }
        const double elapsed =
            std::max(0.0, sim_time_ - real_plan_start_time_);
        const double remaining_horizon =
            std::max(dt, cfg_.prediction_horizon - elapsed);
        rule_engine_->decide(agents_, dt, remaining_horizon,
                             /*reuse_ordinary_coordination=*/true,
                             &real_period_ordinary_decision_);
        for (size_t i = 0; i < agents_.size(); ++i) {
            const auto motion = rule_engine_->motionOverrideFor(agents_[i].id);
            const auto motion_after = std::make_tuple(
                motion.a1_intrusion, static_cast<int>(motion.motion),
                motion.target_s);
            if (motion_after != motion_before[i]) {
                force_horizon_refresh_ = true;
                break;
            }
        }
        marker_pub_->setRollingDecision(real_period_ordinary_decision_);

        // A1 departure transactions are live state. If one is created,
        // released, or changes identity during this measured tick, start a
        // fresh rolling period on the next 0.1 s tick. Ordinary TTC evolution
        // is deliberately absent from this invalidation test.
        if (captureDepartureTransactionIdentity() !=
            real_plan_departure_transactions_) {
            force_horizon_refresh_ = true;
            ROS_INFO_THROTTLE(
                1.0,
                "[real_plan] departure transaction changed; refreshing on "
                "next measured tick");
        }
        return true;
    }

    // Freeze the already-required A1->B leg before cloning the world for a
    // rolling-horizon rollout. This is real-state task preparation, not a
    // simulated assignment: prepareDropoffLeg() only fills the pending_* fields
    // and leaves the currently executing B->A1 leg ACTIVE/TO_A1. Once the
    // rollout reaches A1, its existing state-machine path consumes that frozen
    // information after PICKUP_DWELL via activatePreparedDropoffLeg().
    void prepareA1DropoffPreviewsForHorizon() {
        if (!cfg_.use_a1_cycle || sim_mode_) return;

        for (VehicleAgent& v : agents_) {
            if (v.mode != VehicleMode::ACTIVE ||
                v.mission_phase != MissionPhase::TO_A1 ||
                v.leg_target != LegTargetKind::A1 ||
                v.track.empty() ||
                v.pending_dropoff_valid) {
                continue;
            }

            allocator_->prepareDropoffLeg(v, agents_);
        }
    }

    // 滚动时域发布:推演 rb_horizon_ 秒,把每车时间参数化轨迹发到 /traj_i(刷新=滚动)。
    // hold 车(整段不动)发单点轨迹(size=1)作静止标志 → 控制器 idle 不控制。
    // 滚动时域发布：重新推演并覆盖每辆车的短时轨迹。
    void publishHorizon() {
        // The estop callback has already published current-pose hold
        // trajectories. Never overwrite them with a moving real-mode horizon.
        if (cfg_.real_mode && rb_estop_) return;

        std::vector<sandbox_msgs::Trajectory> trajs;
        std::vector<bool> hold;
        std::vector<size_t> path_gen_cut_indices;

        // Give the cloned world only future tasks that have already been
        // selected and stored in real state. The rollout itself remains unable
        // to create a new B target while sim_mode_ is true.
        prepareA1DropoffPreviewsForHorizon();
        
        // =======世界模型推演，传入（预测时长，预测得到每辆车未来轨迹，预测得到每辆车未来是否保持静止）=============
        if (cfg_.real_mode) {
            // Generate frames with simulation's frozen ordinary-decision
            // semantics, but keep them local: real execution never restores
            // predicted frame state over measured vehicle progress.
            buildRealHorizonPlan(trajs, hold, path_gen_cut_indices);
        } else {
            buildSimulationHorizonPlan(trajs, hold);
        }


        const ros::Time now = ros::Time::now();
        const double tnow = now.toSec();
        std::vector<bool> guard(trajs.size(), false);
        if (cfg_.real_mode) {
            guard = realHardGuard();
        }

          //==================（对每辆车先在Rviz中删除旧轨迹）==========================
        visualization_msgs::MarkerArray arr;
        for (size_t i = 0; i < trajs.size(); ++i) {
            visualization_msgs::Marker del;
            del.header.frame_id = pp_.frame_id;
            del.header.stamp = now;
            del.ns = "horizon_traj";
            del.id = agents_[i].id;
            del.action = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(del);

            const VehicleAgent& v = agents_[i];

            // ========（若车辆正在休眠，则发布单点静止轨迹，不刷新轨迹）=============
            if ((v.mode == VehicleMode::DWELL || v.dwell_remaining > 1e-6) &&
                !v.pending_dropoff_valid) {
                if (i < traj_pubs_.size()) {
                    sandbox_msgs::Trajectory hold;
                    hold.target = v.id;
                    hold.header.frame_id = "world";
                    hold.header.stamp = now;

                    const RoughWp p = poseForCollision(v, v.path_s);
                    sandbox_msgs::TrajectoryPoint tp;
                    tp.x = p.x;
                    tp.y = p.y;
                    tp.yaw = p.theta;
                    tp.velocity = 0.0;
                    tp.time = 0.0;
                    hold.points.push_back(tp);

                    publishTrajectoryWithPathGen(i, hold);
            }

        continue;
}
            // ============（在实车条件下，若车辆位置摆放不正确，则不发布指令）========================
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;


            // ============（若车辆急停、超时、保护，则轨迹退化为单点）========================
            const bool stale = cfg_.real_mode && cfg_.real_pose_timeout > 0.0 &&
                               (tnow - rb_last_seen_[i]) > cfg_.real_pose_timeout;
            if (rb_estop_ || stale || guard[i]) hold[i] = true;

            trajs[i].header.stamp = now;
            if (hold[i] && !trajs[i].points.empty()) {
                trajs[i].points.resize(1);
            }

            //=================（画新的Rviz轨迹，并发布轨迹给控制器）=======================
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = now;
            m.ns = "horizon_traj";
            m.id = v.id;
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.02;
            m.color = agents_[i].color;
            m.color.a = 0.9;
            for (const auto& p : trajs[i].points) {
                geometry_msgs::Point gp;
                gp.x = p.x;
                gp.y = p.y;
                gp.z = 0.09;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);

            if (i < traj_pubs_.size()) {
                sandbox_msgs::Trajectory control_traj = trajs[i];
                if (cfg_.real_mode && i < path_gen_cut_indices.size() &&
                    path_gen_cut_indices[i] < control_traj.points.size()) {
                    control_traj.points.resize(path_gen_cut_indices[i]);
                }
                publishTrajectoryWithPathGen(i, control_traj);
            }
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    bool publishFullTrajectories() {
        std::vector<sandbox_msgs::Trajectory> trajs;
        std::vector<bool> hold;
        rollWorldModel(rb_full_horizon_, trajs, hold);   // 按全程上限推演一次(尾部静止点稍后裁掉)
        const ros::Time now = ros::Time::now();
        visualization_msgs::MarkerArray arr;
        bool all_done = true;
        for (size_t i = 0; i < trajs.size(); ++i) {
            if (agents_[i].track.empty()) continue;      // 无路径的车不计入(不卡总进度)
            if (one_shot_done_[i]) continue;             // 已发过的车不重发(latched 已在控制器手上)
            if (cfg_.real_mode && !real_pose_ok_[i]) {                     // 动捕未就位 → 这辆暂不发,标记未完成下拍补
                ROS_WARN_THROTTLE(1.0, "[real][one_shot] V%d 动捕未就位 → 暂不发轨迹(就位后补发)", agents_[i].id);
                all_done = false;
                continue;
            }
            trimTrailingStationary(trajs[i]);            // 裁掉到点后的尾部静止点,只留一个停止点
            trajs[i].header.stamp = now;
            if (i < traj_pubs_.size()) publishTrajectoryWithPathGen(i, trajs[i]);             // latch 发一次,控制器自主跟到底
            one_shot_done_[i] = true;
            logFullTraj(i, trajs[i]);
            // RViz:整条轨迹画成该车颜色 LINE_STRIP(ns=horizon_traj,沿用现有显示)
            if (trajs[i].points.size() >= 2) {
                visualization_msgs::Marker m;
                m.header.frame_id = pp_.frame_id; m.header.stamp = now;
                m.ns = "horizon_traj"; m.id = agents_[i].id;
                m.type = visualization_msgs::Marker::LINE_STRIP;
                m.action = visualization_msgs::Marker::ADD;
                m.pose.orientation.w = 1.0; m.scale.x = 0.02;
                m.color = agents_[i].color; m.color.a = 0.9;
                for (const auto& p : trajs[i].points) {
                    geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = 0.09;
                    m.points.push_back(gp);
                }
                arr.markers.push_back(m);
            }
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
        return all_done;
    }

    // 急停:给每车发单点 hold 轨迹(当前真实位姿,v=0)。外部 pure_pursuit 收到 size=1 的轨迹后
    // 锁在该点附近停车(不再前进)。一次性纯盲跟下这是唯一的软件急停手段(协调层已不干预纵向)。
    void publishHoldAll() {
        const ros::Time now = ros::Time::now();
        int sent = 0;
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;          // 没真实位姿就别发垃圾点
            sandbox_msgs::Trajectory t;
            t.target = agents_[i].id;
            t.header.frame_id = "world";
            t.header.stamp = now;
            sandbox_msgs::TrajectoryPoint p;
            p.x = real_x_[i]; p.y = real_y_[i]; p.yaw = real_yaw_[i];
            p.velocity = 0.0; p.time = 0.0;
            t.points.push_back(p);
            publishTrajectoryWithPathGen(i, t);      // latch:控制器停在原地
            ++sent;
        }
        ROS_ERROR("[real][one_shot] *** 急停:已对 %d 辆车发单点 hold 轨迹 → 控制器停车 ***", sent);
    }

    // 急停解除:从当前真实位置重新推演全程并重发。不能简单重发原轨迹——外部控制器按轨迹 time
    // 跟踪,重发会把 start_time_ 归零、车在中途却从 t=0 等起 → 卡死。重置 one_shot 标志,让主循环
    // 调 publishFullTrajectories 重新 rollWorldModel(从 realAdvance 同步好的当前 path_s 出发,
    // time=0 即对齐"现在")。
    void resumeFromEstop() {
        std::fill(one_shot_done_.begin(), one_shot_done_.end(), false);
        one_shot_published_ = false;
        ROS_WARN("[real][one_shot] *** 急停解除:从当前真实位置重新推演全程,准备重发 ***");
    }

    // 裁掉到达终点后的尾部静止点(velocity≈0):全程推演为留余量按大 horizon 跑,到点后会拖一长串
    // 原地不动的点,白白撑大轨迹消息。保留末尾一个 velocity=0 的停止点,控制器据此判 reached。
    void trimTrailingStationary(sandbox_msgs::Trajectory& t) {
        if (t.points.size() < 3) return;
        int k = (int)t.points.size() - 1;
        while (k > 0 && std::fabs(t.points[k].velocity) < 1e-3) --k;  // 最后一个仍在动的点
        const int stop = std::min(k + 1, (int)t.points.size() - 1);   // 紧随其后的停止点(在终点)
        sandbox_msgs::TrajectoryPoint sp = t.points[stop];
        sp.velocity = 0.0;
        t.points.resize(k + 1);
        t.points.push_back(sp);
    }

    // 一次性轨迹诊断日志:点数 / 时长 / 路径全长 / 有无倒车段 / 是否真跑到终点(否则 full_horizon 太短)。
    void logFullTraj(size_t i, const sandbox_msgs::Trajectory& t) {
        if (t.points.empty()) return;
        const double dur = t.points.back().time;
        bool has_rev = false;
        for (const auto& p : t.points) if (p.velocity < -1e-3) { has_rev = true; break; }
        const double len = agents_[i].track.empty() ? 0.0 : agents_[i].track.length();
        const bool reached = std::fabs(t.points.back().velocity) < 1e-3;
        ROS_WARN("[real][one_shot] V%d 发整条轨迹 → /traj_%d: 点数=%zu 时长=%.1fs 全长=%.2fm 倒车段=%s%s",
                 agents_[i].id, agents_[i].id, t.points.size(), dur, len,
                 has_rev ? "有" : "无",
                 reached ? "" : "  ⚠ 末点仍在动:full_horizon 太短,加大 ~full_horizon");
    }

    //==============《任务指派与路径生成函数》===================================
    void logSlotDeparture(
        const VehicleAgent& vehicle, const VehicleAgent* owner,
        const char* result, const std::string& reason,
        const VehicleAgent& candidate,
        const forklift_planner::multi_vehicle::RuleEngine::
            SlotDepartureAdmission& admission) {
        std::ostringstream line;
        line << "[SLOT_DEPARTURE] time=" << readableSimTime(sim_time_)
             << " vehicle=V" << vehicle.id << " owner=";
        if (owner != nullptr) line << "V" << owner->id;
        else line << "none";
        line << " result=" << result
             << " reason=" << reason
             << " candidate_path_gen=" << candidate.path_gen
             << " slot_departure_clear_s=" << std::fixed
             << std::setprecision(3) << candidate.slot_departure_clear_s
             << " vehicle_mode=" << modeName(vehicle.mode)
             << " vehicle_speed=" << vehicle.current_speed
             << " vehicle_path_s=" << vehicle.path_s
             << " blocker=";
        if (admission.blocker_id >= 0) line << "V" << admission.blocker_id;
        else line << "none";
        line << " a1_prefix_conflict="
             << (admission.a1_departure_conflict ? 1 : 0)
             << " ordinary_road_conflict="
             << (admission.ordinary_road_conflict ? 1 : 0);
        if (admission.ordinary_road_conflict) {
            line << " first_t=" << admission.first_conflict_t
                 << " candidate_s=" << admission.candidate_conflict_s
                 << " interaction="
                 << (admission.interaction_type ==
                             forklift_planner::multi_vehicle::
                                 PairInteractionType::OPPOSING
                         ? "OPPOSING"
                         : admission.interaction_type ==
                                   forklift_planner::multi_vehicle::
                                       PairInteractionType::SAME_DIRECTION
                               ? "SAME_DIRECTION" : "CROSSING");
        }
        if (owner != nullptr) {
            line << " owner_phase=" << missionPhaseName(owner->mission_phase)
                 << " owner_path_gen=" << owner->path_gen;
        }
        if (owner != nullptr) {
            line << " geometry="
                 << (admission.a1.owner_uses_pending_preview
                         ? "pending_preview" : "actual_to_b")
                 << " protected_zones="
                 << admission.a1.protected_zone_count
                 << " actual_occupancy_priority="
                 << (admission.a1.actual_occupancy_priority ? 1 : 0)
                 << " spatial_stop_launch_infeasible="
                 << (admission.a1.spatial_stop_launch_infeasible ? 1 : 0)
                 << " source_slot_hold="
                 << (admission.a1.source_slot_hold ? 1 : 0);
            if (admission.a1.waiter_stop_s >= 0.0) {
                line << " waiter_stop_s="
                     << admission.a1.waiter_stop_s;
            }
        }
        ROS_WARN("%s", line.str().c_str());
        coordLogWithContext(line.str(), "REAL", sim_plan_id_, -1, -1);
    }

    // 是否允许传入的车辆id离库，能则准备路径，否则继续留在库位
    bool launchPickupLegWithA1Admission(VehicleAgent& vehicle) {
        VehicleAgent candidate = vehicle;
        if (!allocator_->assignPickupLeg(candidate, /*emit_log=*/false)) {
            vehicle.mission_phase = MissionPhase::TO_A1;
            vehicle.leg_target = LegTargetKind::A1;
            vehicle.mode = VehicleMode::NEED_TASK;
            return false;
        }
        //读取车辆id是否在库位B0-B9
        const bool is_b0_b9 = vehicle.current_slot >= 0 && vehicle.current_slot <= 9;

        //读取未来的A1 owner
        const auto& future_a1_commitment =rule_engine_->futureA1Commitment();
        
        //判断该id车辆是否为onwer，且对应是此次B->A1，若是才生效
        const bool owns_a1 = future_a1_commitment.valid() && future_a1_commitment.owner_id == vehicle.id && future_a1_commitment.owner_path_gen == candidate.path_gen;
        //若本次车辆不是，则禁止出库，但持续尝试竞争owner
        if (is_b0_b9 && !owns_a1) 
        {
            //将车辆状态机设为卸货休眠位，并且路径目标端为库位B
            vehicle.mode = VehicleMode::DWELL;
            vehicle.mission_phase = MissionPhase::UNLOAD_DWELL;
            vehicle.leg_target = LegTargetKind::B_SLOT;

            vehicle.dwell_remaining = 0.0;
            vehicle.action = VehicleAction::STOP;
            vehicle.requested_action = VehicleAction::STOP;
            vehicle.current_speed = 0.0;
            vehicle.reason = "a1_b0_b9_wait_owner";
            return false;
        }

        VehicleAgent* owner = nullptr;
        if (future_a1_commitment.valid() && future_a1_commitment.owner_id != vehicle.id)
            owner = agentById(future_a1_commitment.owner_id);

        const auto admission = rule_engine_->checkSlotDepartureAdmission(
            owner, candidate, agents_, rb_horizon_);
        const bool hold = !admission.clear;
        const std::string hold_reason = admission.ordinary_road_conflict
            ? "ordinary_immediate_conflict"
            : admission.a1.source_slot_hold
                ? "a1_source_slot_admission"
            : admission.a1.spatial_stop_launch_infeasible
                ? "a1_stop_s_before_slot_clear"
                : "a1_departure_prefix_conflict";
        auto held = a1_launch_holds_.find(vehicle.id);
        if (hold) {
            const bool changed = held == a1_launch_holds_.end() ||
                held->second.blocker_id != admission.blocker_id ||
                held->second.reason != hold_reason;
            if (held == a1_launch_holds_.end()) {
                A1LaunchHoldState state;
                state.since = sim_time_;
                state.blocker_id = admission.blocker_id;
                state.candidate_path_gen = candidate.path_gen;
                state.reason = hold_reason;
                a1_launch_holds_[vehicle.id] = state;
                ++a1_launch_metrics_.holds;
                if (admission.a1_departure_conflict) {
                    ++a1_launch_metrics_.a1_prefix_holds;
                }
                if (admission.ordinary_road_conflict) {
                    ++a1_launch_metrics_.ordinary_road_holds;
                }
            } else {
                ++a1_launch_metrics_.retries;
                if (changed) {
                    held->second.blocker_id = admission.blocker_id;
                    held->second.reason = hold_reason;
                }
            }
            if (changed) {
                logSlotDeparture(vehicle, owner, "HOLD", hold_reason,
                                 candidate, admission);
            }
            vehicle.mode = VehicleMode::DWELL;
            vehicle.dwell_remaining = 0.0;
            vehicle.action = VehicleAction::STOP;
            vehicle.requested_action = VehicleAction::STOP;
            vehicle.current_speed = 0.0;
            vehicle.reason = "slot_departure_hold_V" +
                             std::to_string(admission.blocker_id);
            return false;
        }

        std::string allow_reason = "no_service_owner";
        if (owner != nullptr) {
            allow_reason = admission.a1.actual_occupancy_priority
                ? "actual_occupancy_priority"
                : "slot_departure_clear";
        }
        if (held != a1_launch_holds_.end()) {
            const double duration = sim_time_ - held->second.since;
            a1_launch_metrics_.max_hold_duration = std::max(
                a1_launch_metrics_.max_hold_duration, duration);
            ++a1_launch_metrics_.released_after_hold;
            allow_reason = "slot_departure_clear_after_retry";
            a1_launch_holds_.erase(held);
        }

        const bool assigned = allocator_->assignPickupLeg(vehicle);
        if (assigned) {
            ++a1_launch_metrics_.allows;
            logSlotDeparture(vehicle, owner, "ALLOW", allow_reason,
                             vehicle, admission);
        }
        return assigned;
    }

    void updateDwellAndTasks(double dt) {
        for (VehicleAgent& v : agents_) {

            //使用B->A1->B模式
            if (cfg_.use_a1_cycle) {
                //1.车辆需要任务
                if (v.mode == VehicleMode::NEED_TASK) {
                    if (sim_mode_) continue;
                    const int old_gen = v.path_gen;
                    if (v.mission_phase == MissionPhase::TO_A1) {
                        launchPickupLegWithA1Admission(v);
                    } else {
                        allocator_->assignNextTask(v, agents_);
                    }
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }
                //2.车辆属于休眠状态
                if (v.mode != VehicleMode::DWELL) continue;

                v.dwell_remaining = std::max(0.0, v.dwell_remaining - dt);
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.current_speed = 0.0;
                if (v.dwell_remaining > 1e-9) continue;

                if (sim_mode_) {
                    if (v.mission_phase == MissionPhase::PICKUP_DWELL && v.pending_dropoff_valid) {
                        allocator_->activatePreparedDropoffLeg(v, /*emit_log=*/false);
                    } else {
                        v.dwell_remaining = 0.0;
                    }
                    continue;
                }

                //3.车辆处于TO A1并且车辆存在a1_launch_holds
                if (v.mission_phase == MissionPhase::TO_A1 && a1_launch_holds_.find(v.id) != a1_launch_holds_.end()) {
                    const int old_gen = v.path_gen;
                    launchPickupLegWithA1Admission(v);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }

                //4.车辆处于库位卸货休眠
                if (v.mission_phase == MissionPhase::PICKUP_DWELL) {
                    const int old_gen = v.path_gen;
                    allocator_->assignDropoffLeg(v, agents_);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }

                //5.车辆等待卸货
                if (v.mission_phase == MissionPhase::WAIT_DROPOFF_TASK) {
                    const int old_gen = v.path_gen;
                    allocator_->assignDropoffLeg(v, agents_);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }

                if (v.mission_phase == MissionPhase::UNLOAD_DWELL) {
                    const bool completed_transport = v.loaded;
                    v.loaded = false;
                    if (completed_transport) ++v.task_count;
                    if (one_shot_) {
                        v.action = VehicleAction::STOP;
                        v.requested_action = VehicleAction::STOP;
                        v.reason = "one_shot_complete";
                        continue;
                    }

                    const int old_gen = v.path_gen;
                    launchPickupLegWithA1Admission(v);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }
                continue;
            }

            // ********2. 若该车状态为需要任务，则尝试指派任务 ************
            // NEED_TASK 的车每拍重试派活——分配可能因"当下所有路都与在途车对穿"而暂时失败,
            // 但别车一移动局面就变,必须重试,否则车永久饿死(实测 6 车卡死的根因之一)。
            if (v.mode == VehicleMode::NEED_TASK) {
                if (sim_mode_) continue;
                allocator_->assignNextTask(v, agents_);         //***关键任务指派函数和路径生成 ******/
                continue;
            }


            //********** 3.若车不为休眠状态，则直接跳过 *********************
            if (v.mode != VehicleMode::DWELL) continue;

            //**********4.反之，若车在休眠状态，则停止动作，计算倒计睡眠时间，同时根据设置执行操作*/
            v.dwell_remaining = std::max(0.0, v.dwell_remaining - dt);
            v.action = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.current_speed = 0.0;
                    
                //********* 4.1 若车已过睡眠时间，并且为一次性规划，则跳过不再派发任务 ******/
            if (v.dwell_remaining <= 1e-9) {
                if (sim_mode_) {
                    v.dwell_remaining = 0.0;
                    continue;
                }
                if (one_shot_) {        // 一次性 demo:到达目标后永久停,不再派活(8车各跑一程 A→B)
                    v.action = VehicleAction::STOP;
                    v.requested_action = VehicleAction::STOP;
                    continue;
                }

                //******** 4.2 若车已过睡眠时间，并且为不间断跑，则切换速度，并派发任务 ******/
                v.loaded = !v.loaded;
                allocator_->assignNextTask(v, agents_);
            }
        }
    }

    //==============================================

    void handleLegArrival(VehicleAgent& v) {
        v.current_speed = 0.0;
        v.wait_time = 0.0;
        v.mode = VehicleMode::DWELL;
        v.action = VehicleAction::STOP;
        v.requested_action = VehicleAction::STOP;

        if (cfg_.use_a1_cycle && v.leg_target == LegTargetKind::A1) {
            const MissionPhase old_phase = v.mission_phase;
            // A1 is a virtual pickup point, not a B slot. Do not write its
            // virtual id into current_slot or visited_slots_.
            v.loaded = false;
            v.mission_phase = MissionPhase::PICKUP_DWELL;
            v.dwell_remaining = cfg_.pickup_dwell_time;
            v.reason = "pickup_dwell";
            if (!sim_mode_) {
                char line[300];
                std::snprintf(
                    line, sizeof(line),
                    "[A1_ARRIVAL] V%d phase=%s->%s pending_dropoff=%d "
                    "dropoff_slot=%d track_size=%zu path_gen=%d",
                    v.id, missionPhaseName(old_phase),
                    missionPhaseName(v.mission_phase),
                    v.pending_dropoff_valid ? 1 : 0,
                    v.pending_dropoff_slot,
                    v.pending_dropoff_track.path().size(), v.path_gen);
                coordLog(line);
            }
            // Simulation-only behavior for now: choose and reserve B at the
            // real A1 arrival so the next rollout can contain 5 s pickup plus
            // the future A1 exit. Do not change the proven real-vehicle task
            // timing until the simulation design has been validated.
            if (!sim_mode_ && !cfg_.real_mode) {
                if (allocator_->prepareDropoffLeg(v, agents_)) {
                    force_horizon_refresh_ = true;
                } else {
                    ROS_ERROR("[multi_patrol][A1 EXIT PREPARE FAILED] V%d "
                              "has no reservable A1->B task at pickup start",
                              v.id);
                }
            }
            return;
        }

        v.current_slot = v.target_slot;
        if (v.current_slot >= 0 &&
            v.current_slot < static_cast<int>(visited_slots_.size())) {
            visited_slots_[static_cast<size_t>(v.current_slot)] = true;
        }

        if (cfg_.use_a1_cycle) {
            v.mission_phase = MissionPhase::UNLOAD_DWELL;
            v.dwell_remaining = cfg_.unload_dwell_time;
            v.reason = "unload_dwell";
        } else {
            ++v.task_count;
            v.dwell_remaining = cfg_.dwell_time;
            v.reason = "dwell";
        }
    }

    void advanceVehicles(double dt) {
        std::vector<double> next_s(agents_.size(), 0.0);
        std::vector<double> next_speed(agents_.size(), 0.0);
        std::vector<double> planned_s(agents_.size(), 0.0);
        std::vector<bool> blocked(agents_.size(), false);

        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            next_s[i] = v.path_s;
            planned_s[i] = v.path_s;
            next_speed[i] = v.current_speed;
            if (!v.active()) continue;

            const auto motion_override = rule_engine_->motionOverrideFor(v.id);
            const auto recovery_motion = motion_override.motion;
            if (recovery_motion ==
                forklift_planner::multi_vehicle::RecoveryMotion::HOLD) {
                next_speed[i] = 0.0;
                continue;
            }

            // 规划速度=动作档,再被曲率限速卡住(与实车 coord_speed 同一套,sim 才能真实验证)。
            const VehicleAction motion_action =
                recovery_motion ==
                        forklift_planner::multi_vehicle::RecoveryMotion::RETREAT
                    ? VehicleAction::CREEP : v.action;
            const double target_speed = recovery_motion ==
                    forklift_planner::multi_vehicle::RecoveryMotion::RETREAT
                ? cfg_.deadlock_retreat_speed
                : rule_engine_->speedForAction(motion_action);
            const double desired_speed = std::min(target_speed,
                                                  curvatureSpeed(v));
            next_speed[i] = limitedSpeed(v.current_speed, desired_speed, dt);
            if (recovery_motion ==
                forklift_planner::multi_vehicle::RecoveryMotion::RETREAT) {
                next_s[i] = std::max(
                    motion_override.target_s,
                    v.path_s - next_speed[i] * dt);
            } else {
                next_s[i] = std::min(v.track.length(),
                                     v.path_s + next_speed[i] * dt);
            }
            planned_s[i] = next_s[i];
        }

        auto plannedS = [&](size_t idx) {
            return planned_s[idx];
        };

        auto bodyAt = [&](size_t idx, double path_s) {
            const RoughWp pose = poseForCollision(agents_[idx], path_s);
            return forklift_planner::multi_vehicle::makeBody(pose, mp_, 0.0);
        };

        auto overlapsAt = [&](size_t i, double s_i, size_t j, double s_j) {
            return forklift_planner::multi_vehicle::overlaps(bodyAt(i, s_i),
                                                             bodyAt(j, s_j));
        };

        auto canPlace = [&](size_t idx, double candidate_s) {
            for (size_t k = 0; k < agents_.size(); ++k) {
                if (k == idx) continue;
                if (agents_[k].mode != VehicleMode::ACTIVE &&
                    agents_[k].mode != VehicleMode::DWELL) {
                    continue;
                }
                if (overlapsAt(idx, candidate_s, k, plannedS(k))) return false;
            }
            return true;
        };

        auto canPlaceIgnoringPair = [&](size_t idx, double candidate_s,
                                        size_t pair_other) {
            for (size_t k = 0; k < agents_.size(); ++k) {
                if (k == idx || k == pair_other) continue;
                if (agents_[k].mode != VehicleMode::ACTIVE &&
                    agents_[k].mode != VehicleMode::DWELL) {
                    continue;
                }
                if (overlapsAt(idx, candidate_s, k, plannedS(k))) return false;
            }
            return true;
        };

        auto tryClearBlocker = [&](size_t idx, int blocker_id) {
            VehicleAgent& v = agents_[idx];
            if (!v.active()) return false;
            if (rule_engine_->motionOverrideFor(v.id).motion !=
                forklift_planner::multi_vehicle::RecoveryMotion::NORMAL) {
                return false;
            }
            if (std::abs(next_s[idx] - v.path_s) > 1e-9) return false;

            const double creep_speed =
                limitedSpeed(v.current_speed,
                             rule_engine_->speedForAction(VehicleAction::CREEP),
                             dt);
            const double candidate_s =
                std::min(v.track.length(), v.path_s + creep_speed * dt);
            if (candidate_s <= v.path_s + 1e-9) return false;
            if (!canPlace(idx, candidate_s)) return false;

            blocked[idx] = false;
            next_speed[idx] = creep_speed;
            next_s[idx] = candidate_s;
            planned_s[idx] = candidate_s;
            v.action = VehicleAction::CREEP;
            v.requested_action = VehicleAction::CREEP;
            v.reason = "clear_blocker_V" + std::to_string(blocker_id);
            return true;
        };

        auto blockVehicle = [&](size_t idx) {
            if (!agents_[idx].active()) return false;
            const bool changed =
                !blocked[idx] || std::abs(planned_s[idx] - agents_[idx].path_s) > 1e-9 ||
                next_speed[idx] > 1e-9;
            blocked[idx] = true;
            planned_s[idx] = agents_[idx].path_s;
            next_s[idx] = agents_[idx].path_s;
            next_speed[idx] = 0.0;
            return changed;
        };

        auto resolvePlannedOverlaps = [&]() {
        const size_t max_guard_iterations =
            std::max<size_t>(4, agents_.size() * agents_.size() * 2);
        for (size_t iter = 0; iter < max_guard_iterations; ++iter) {
            bool changed = false;
            bool any_overlap = false;

            for (size_t i = 0; i < agents_.size(); ++i) {
                if (agents_[i].mode != VehicleMode::ACTIVE &&
                    agents_[i].mode != VehicleMode::DWELL) {
                    continue;
                }

                for (size_t j = i + 1; j < agents_.size(); ++j) {
                    if (agents_[j].mode != VehicleMode::ACTIVE &&
                        agents_[j].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (!overlapsAt(i, plannedS(i), j, plannedS(j))) {
                        continue;
                    }

                    any_overlap = true;
                    const bool i_active = agents_[i].active();
                    const bool j_active = agents_[j].active();
                    if (i_active && !j_active) {
                        changed = blockVehicle(i) || changed;
                    } else if (!i_active && j_active) {
                        changed = blockVehicle(j) || changed;
                    } else if (i_active && j_active) {
                        const bool i_moves = std::abs(
                            next_s[i] - agents_[i].path_s) > 1e-9;
                        const bool j_moves = std::abs(
                            next_s[j] - agents_[j].path_s) > 1e-9;
                        const bool i_only_safe =
                            i_moves &&
                            !overlapsAt(i, next_s[i], j, agents_[j].path_s);
                        const bool j_only_safe =
                            j_moves &&
                            !overlapsAt(i, agents_[i].path_s, j, next_s[j]);
                        const bool both_next_safe =
                            i_moves && j_moves &&
                            !overlapsAt(i, next_s[i], j, next_s[j]) &&
                            canPlaceIgnoringPair(i, next_s[i], j) &&
                            canPlaceIgnoringPair(j, next_s[j], i);
                        const bool both_stop_safe =
                            !overlapsAt(i, agents_[i].path_s, j, agents_[j].path_s);

                        if (both_next_safe) {
                            const bool changed_i =
                                blocked[i] ||
                                std::abs(planned_s[i] - next_s[i]) > 1e-9;
                            const bool changed_j =
                                blocked[j] ||
                                std::abs(planned_s[j] - next_s[j]) > 1e-9;
                            blocked[i] = false;
                            blocked[j] = false;
                            planned_s[i] = next_s[i];
                            planned_s[j] = next_s[j];
                            changed = changed_i || changed_j || changed;
                        } else if (i_only_safe && !j_only_safe) {
                            changed = blockVehicle(j) || changed;
                            tryClearBlocker(i, agents_[j].id);
                        } else if (!i_only_safe && j_only_safe) {
                            changed = blockVehicle(i) || changed;
                            tryClearBlocker(j, agents_[i].id);
                        } else if (i_only_safe && j_only_safe) {
                            const int winner = rule_engine_->priorityWinner(
                                agents_[i], agents_[j]);
                            if (winner == agents_[i].id) {
                                changed = blockVehicle(j) || changed;
                                tryClearBlocker(i, agents_[j].id);
                            } else if (winner == agents_[j].id) {
                                changed = blockVehicle(i) || changed;
                                tryClearBlocker(j, agents_[i].id);
                            } else {
                                // Tie-break rule disabled: no winner, stop both.
                                changed = blockVehicle(i) || changed;
                                changed = blockVehicle(j) || changed;
                            }
                        } else if (both_stop_safe) {
                            changed = blockVehicle(i) || changed;
                            changed = blockVehicle(j) || changed;
                        } else {
                            const int winner = rule_engine_->priorityWinner(
                                agents_[i], agents_[j]);
                            if (winner == agents_[i].id) {
                                changed = blockVehicle(j) || changed;
                                if (!tryClearBlocker(i, agents_[j].id)) {
                                    changed = blockVehicle(i) || changed;
                                }
                            } else if (winner == agents_[j].id) {
                                changed = blockVehicle(i) || changed;
                                if (!tryClearBlocker(j, agents_[i].id)) {
                                    changed = blockVehicle(j) || changed;
                                }
                            } else {
                                // Tie-break rule disabled: no winner, stop both.
                                changed = blockVehicle(i) || changed;
                                changed = blockVehicle(j) || changed;
                            }
                        }
                    }
                    if (!sim_mode_) {  // 前瞻仿真中只要其物理挡停效果,不计数/不打日志
                        ++hard_guard_events_;
                        hard_guard_pairs_.insert(
                            {std::min(agents_[i].id, agents_[j].id),
                             std::max(agents_[i].id, agents_[j].id)});
                        if (first_guard_tick_ == 0) first_guard_tick_ = tick_count_;
                        ROS_ERROR_THROTTLE(
                            1.0,
                            "[multi_patrol] hard collision guard: V%d vs V%d; "
                            "minimal stop applied",
                            agents_[i].id, agents_[j].id);
                    }
                }
            }

            if (!any_overlap || !changed) {
                break;
            }
        }
        };

        resolvePlannedOverlaps();

        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            if (!v.active()) continue;

            if (blocked[i]) {
                v.current_speed = 0.0;
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.reason = "hard_collision_guard";
                // Keep the physical blocker edge for diagnostics and for the
                // next live two-vehicle deadlock observation.
                const double fwd_s = std::min(
                    v.track.length(),
                    v.path_s + rule_engine_->speedForAction(VehicleAction::CREEP) * dt);
                v.blocker_id = -1;
                for (size_t k = 0; k < agents_.size(); ++k) {
                    if (k == i) continue;
                    if (agents_[k].mode != VehicleMode::ACTIVE &&
                        agents_[k].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (overlapsAt(i, fwd_s, k, plannedS(k))) {
                        v.blocker_id = agents_[k].id;
                        break;
                    }
                }
                continue;
            }

            v.current_speed = next_speed[i];
            v.path_s = next_s[i];

            if (v.a1_departure_committed &&
                v.path_s >= v.a1_departure_priority_until_s - 1e-9) {
                v.a1_departure_committed = false;
                if (!sim_mode_) {
                    ROS_INFO("[multi_patrol][A1 EXIT PRIORITY RELEASE] V%d "
                             "s=%.3f threshold=%.3f",
                             v.id, v.path_s,
                             v.a1_departure_priority_until_s);
                }
            }

            if (v.path_s >= v.track.length() - 1e-9) {
                handleLegArrival(v);
                // 批处理(长测)里关掉每次到位的 INFO——24h×8车×数千任务=2万+条,会把
                // 关键的"首撞/首楔"现场 dump 在 rosout 滚动里冲掉。实时/RViz 模式保留。
                if (cfg_batch_ticks_ == 0 && !sim_mode_) {
                    const std::string destination =
                        v.leg_target == LegTargetKind::A1
                            ? "A1"
                            : "B" + std::to_string(v.current_slot);
                    ROS_INFO("[multi_patrol] tick=%llu sim_t=%.2f V%d arrived %s; "
                             "dwell %.2fs; load=%s phase=%d",
                             static_cast<unsigned long long>(tick_count_), sim_time_,
                             v.id, destination.c_str(), v.dwell_remaining,
                             v.loaded ? "loaded" : "empty",
                             static_cast<int>(v.mission_phase));
                }
            }
        }
    }

    void logAgentStatus() {
        const ros::Time now = ros::Time::now();
        for (size_t i = 0; i < agents_.size(); ++i) {
            const VehicleAgent& v = agents_[i];
            const bool changed =
                v.mode != last_logged_mode_[i] ||
                v.action != last_logged_action_[i] ||
                v.reason != last_logged_reason_[i] ||
                v.blocker_id != last_logged_blocker_[i] ||
                v.task_count != last_logged_task_count_[i] ||
                v.mission_phase != last_logged_mission_phase_[i];
            const bool stopped_active =
                v.mode == VehicleMode::ACTIVE && v.action == VehicleAction::STOP;
            const bool periodic =
                stopped_active &&
                (last_status_log_time_[i].isZero() ||
                 (now - last_status_log_time_[i]).toSec() >= 2.0);
            if (!changed && !periodic) continue;

            const double length = v.track.empty() ? 0.0 : v.track.length();
            const double rem = v.track.empty() ? 0.0 : v.remainingS();
            char buf[512];
            const std::string readable_time = readableSimTime(sim_time_);
            std::snprintf(
                buf, sizeof(buf),
                "[multi_patrol][state] tick=%llu sim_t=%s V%d "
                "mode=%s phase=%s action=%s reason=%s "
                "blocker=%d task=%d slot=%d->%d s=%.3f/%.3f rem=%.3f "
                "speed=%.3f wait=%.2f dwell=%.2f",
                static_cast<unsigned long long>(tick_count_),
                readable_time.c_str(),
                v.id, modeName(v.mode), missionPhaseName(v.mission_phase),
                actionName(v.action),
                v.reason.empty() ? "-" : v.reason.c_str(), v.blocker_id,
                v.task_count, v.current_slot, v.target_slot, v.path_s,
                length, rem, v.current_speed, v.wait_time,
                v.dwell_remaining);
            const std::string console_line = contextualLog(
                buf, "REAL", coord_log_plan_id_, coord_log_frame_id_, -1);
            ROS_INFO("%s", console_line.c_str());
            coordLog(buf);

            last_logged_mode_[i] = v.mode;
            last_logged_action_[i] = v.action;
            last_logged_reason_[i] = v.reason;
            last_logged_blocker_[i] = v.blocker_id;
            last_logged_task_count_[i] = v.task_count;
            last_logged_mission_phase_[i] = v.mission_phase;
            last_status_log_time_[i] = now;
        }
    }

    // TEMPORARY: dump relative geometry of any vehicle stuck (speed~0, wait>5s)
    // against its blocker, to classify head-on vs follower-misclassification vs
    // priority circularity. Remove once the V6/V7 deadlock root cause is fixed.
    void logStuckDiagnostics() {
        const ros::Time now = ros::Time::now();
        auto motionHeading = [](const VehicleAgent& v) {
            constexpr double kPi = 3.14159265358979323846;
            double h = v.track.poseAtS(v.path_s).theta;
            if (v.track.typeAtS(v.path_s) == WpType::REVERSE) h += kPi;
            return h;
        };
        for (size_t i = 0; i < agents_.size(); ++i) {
            const VehicleAgent& v = agents_[i];
            if (v.mode != VehicleMode::ACTIVE) continue;
            if (v.current_speed > 1e-3 || v.wait_time < 5.0) continue;
            if (!last_diag_time_[i].isZero() &&
                (now - last_diag_time_[i]).toSec() < 3.0) continue;
            last_diag_time_[i] = now;

            const RoughWp pv = v.track.poseAtS(v.path_s);
            const int wt = static_cast<int>(v.track.typeAtS(v.path_s));
            const VehicleAgent* blocker = agentById_c(v.blocker_id);
            if (blocker == nullptr) {
                ROS_DEBUG("[DIAG stuck] V%d wait=%.1f reason=%s wp=%d "
                         "pose=(%.3f,%.3f) blocker=none",
                         v.id, v.wait_time, v.reason.c_str(), wt, pv.x, pv.y);
                continue;
            }
            const VehicleAgent& b = *blocker;
            const RoughWp pb = b.track.poseAtS(b.path_s);
            const double hv = motionHeading(v);
            const double hb = motionHeading(b);
            const double dx = pb.x - pv.x;
            const double dy = pb.y - pv.y;
            const double dot = std::cos(hv) * std::cos(hb) +
                               std::sin(hv) * std::sin(hb);
            const double fwd = dx * std::cos(hv) + dy * std::sin(hv);
            const double lat = std::abs(-dx * std::sin(hv) + dy * std::cos(hv));
            const double gap = std::hypot(dx, dy) - mp_.vehicle_length;
            ROS_DEBUG("[DIAG stuck] V%d wait=%.1f reason=%s wp=%d | "
                     "blkV%d(act=%s spd=%.3f wp=%d) dot=%.2f fwd=%.3f lat=%.3f "
                     "gap=%.3f vw=%.3f",
                     v.id, v.wait_time, v.reason.c_str(), wt, b.id,
                     actionName(b.action), b.current_speed,
                     static_cast<int>(b.track.typeAtS(b.path_s)), dot, fwd, lat,
                     gap, mp_.vehicle_width);
        }
    }

    //==================== 最重要的节拍函数======================================
    // Diagnostic-only branch for the unresolved A1 case: another vehicle is
    // already physically inside the visible portion of a prepared A1 exit.
    // This method never issues a recovery action.
    void diagnoseA1ExitIntrusions() {
        if (sim_mode_ || cfg_.real_mode || !cfg_.use_a1_cycle) return;

        std::set<std::pair<int, int>> current;
        const double step = std::max(0.02, cfg_.prediction_step);
        const double footprint_margin = 0.5 * cfg_.conflict_margin;

        for (const VehicleAgent& owner : agents_) {
            const bool pending_owner =
                owner.mode == VehicleMode::DWELL &&
                owner.mission_phase == MissionPhase::PICKUP_DWELL &&
                owner.pending_dropoff_valid &&
                !owner.pending_dropoff_track.empty();
            const bool active_owner =
                owner.active() && owner.a1_departure_committed;
            if (!pending_owner && !active_owner) {
                continue;
            }

            const double dwell_before_motion =
                pending_owner ? owner.dwell_remaining : 0.0;
            const double motion_time =
                std::max(0.0, rb_horizon_ - dwell_before_motion);
            if (motion_time <= 1e-9) continue;

            VehicleAgent preview = owner;
            if (pending_owner) {
                preview.track = owner.pending_dropoff_track;
                preview.path_s = 0.0;
                preview.current_speed = 0.0;
            }
            preview.mode = VehicleMode::ACTIVE;

            struct ExitSample {
                double t;
                double s;
                forklift_planner::multi_vehicle::OBB body;
            };
            std::vector<ExitSample> exit_samples;
            exit_samples.push_back(ExitSample{
                dwell_before_motion, preview.path_s,
                forklift_planner::multi_vehicle::makeBody(
                    preview.track.poseAtS(preview.path_s), mp_,
                    footprint_margin)});
            double elapsed = 0.0;
            while (elapsed < motion_time - 1e-9 &&
                   preview.path_s < preview.track.length() - 1e-9) {
                const double sample_dt = std::min(step, motion_time - elapsed);
                const double desired = std::min(
                    rule_engine_->speedForAction(VehicleAction::NOMINAL),
                    curvatureSpeed(preview));
                preview.current_speed =
                    limitedSpeed(preview.current_speed, desired, sample_dt);
                preview.path_s = std::min(
                    preview.track.length(),
                    preview.path_s + preview.current_speed * sample_dt);
                elapsed += sample_dt;
                exit_samples.push_back(ExitSample{
                    dwell_before_motion + elapsed, preview.path_s,
                    forklift_planner::multi_vehicle::makeBody(
                        preview.track.poseAtS(preview.path_s), mp_,
                        footprint_margin)});
            }

            for (const VehicleAgent& other : agents_) {
                if (other.id == owner.id ||
                    (other.mode != VehicleMode::ACTIVE &&
                     other.mode != VehicleMode::DWELL) ||
                    other.track.empty()) {
                    continue;
                }
                const forklift_planner::multi_vehicle::OBB other_body =
                    forklift_planner::multi_vehicle::makeBody(
                        poseForCollision(other, other.path_s), mp_,
                        footprint_margin);
                const ExitSample* hit = nullptr;
                for (const ExitSample& sample : exit_samples) {
                    if (forklift_planner::multi_vehicle::overlaps(
                            sample.body, other_body)) {
                        hit = &sample;
                        break;
                    }
                }
                if (hit == nullptr) continue;

                const std::pair<int, int> key{owner.id, other.id};
                current.insert(key);
                if (active_a1_exit_intrusions_.count(key) == 0) {
                    char line[420];
                    std::snprintf(
                        line, sizeof(line),
                        "[multi_patrol][A1 EXIT INTRUSION ENTER] owner=V%d "
                        "target=B%d blocker=V%d dwell_remaining=%.2fs "
                        "visible_exit_t=%.2fs owner_exit_s=%.3f "
                        "blocker_phase=%d blocker_s=%.3f action=DIAG_ONLY",
                        owner.id,
                        pending_owner ? owner.pending_dropoff_slot
                                      : owner.target_slot,
                        other.id, dwell_before_motion, hit->t, hit->s,
                        static_cast<int>(other.mission_phase), other.path_s);
                    const std::string console_line = contextualLog(
                        line, "REAL", coord_log_plan_id_,
                        coord_log_frame_id_, -1);
                    ROS_WARN("%s", console_line.c_str());
                    coordLog(line);
                }
            }
        }

        for (const auto& old : active_a1_exit_intrusions_) {
            if (current.count(old) != 0) continue;
            char line[180];
            std::snprintf(
                line, sizeof(line),
                "[multi_patrol][A1 EXIT INTRUSION CLEAR] owner=V%d blocker=V%d",
                old.first, old.second);
            const std::string console_line = contextualLog(
                line, "REAL", coord_log_plan_id_, coord_log_frame_id_, -1);
            ROS_WARN("%s", console_line.c_str());
            coordLog(line);
        }
        active_a1_exit_intrusions_.swap(current);
    }

    void tick(const ros::TimerEvent&) {
        const double dt = 1.0 / pp_.update_rate;        //控制周期与仿真系统推移周期一致
        ++tick_count_;          //记录系统运行了多少隔周期
        sim_time_ += dt;        //仿真时间增加一个固定时间步长
        setCoordLogContext("REAL", sim_plan_id_, coord_log_frame_id_, -1);


        // 1. 实车模式---未摆放好姿态模式
        if (cfg_.real_mode) {
            if (!rb_started_) {
                marker_pub_->publish(
                    agents_, visited_slots_, rule_engine_->conflicts(),
                    marker_pub_->hasSubscribers()
                        ? rule_engine_->conflictResourceMarkers(agents_)
                        : std::vector<forklift_planner::multi_vehicle::
                              ConflictMarker>{},
                    rule_engine_->futureA1Commitment(),
                    rule_engine_->a1DepartureClusters(),
                    rule_engine_->recoveryDirective());   //发布车辆、地图
                publishRealTrailMarkers();  //发布真实车身尾迹
                logPlacementStatus();       //打印摆车状态
                return;
            }

        //2. 实车模式---选择一次性发布完整轨迹
            if (rb_one_shot_traj_) {
                if (rb_estop_ && !rb_estop_prev_) publishHoldAll();         //刚刚进入急停，给所有车发单点轨迹
                else if (!rb_estop_ && rb_estop_prev_) resumeFromEstop();   //刚刚解除急停，就从当前位置重新发轨迹
                rb_estop_prev_ = rb_estop_;

                if (!rb_estop_) {               //若没有急停
                    if (!one_shot_published_) one_shot_published_ = publishFullTrajectories();
                    updateDwellAndTasks(dt);        //检测到没有发送轨迹，一次性发布整条轨迹
                    rule_engine_->decide(agents_, dt);
                    marker_pub_->setRollingDecision(
                        rule_engine_->lastRollingDynamicDecision());
                }
                realAdvance(dt);        //根据真实车身位置重新定位
                logAgentStatus();
                marker_pub_->publish(
                    agents_, visited_slots_, rule_engine_->conflicts(),
                    marker_pub_->hasSubscribers()
                        ? rule_engine_->conflictResourceMarkers(agents_)
                        : std::vector<forklift_planner::multi_vehicle::
                              ConflictMarker>{},
                    rule_engine_->futureA1Commitment(),
                    rule_engine_->a1DepartureClusters(),
                    rule_engine_->recoveryDirective());
                publishRealTrailMarkers();
                return;
            }

        //3. 实车模式----滚动时域规划
            // Task time advances once per tick. Arrival is evaluated only
            // afterwards from the latest /object projection, so a newly
            // entered DWELL phase is not decremented again in the same tick.
            updateDwellAndTasks(dt);
            realAdvance(dt);
            if (!rb_estop_ && realPlanNeedsRefresh()) {
                publishHorizon();
                force_horizon_refresh_ = false;
            }
            if (!executeRealRollingDecision(dt)) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[real_plan] no frozen rolling decision; using one "
                    "current-step decision");
                rule_engine_->decide(agents_, dt);
                marker_pub_->setRollingDecision(
                    rule_engine_->lastRollingDynamicDecision());
            }
            if (!rb_estop_ &&
                captureRecoveryIdentity() != real_plan_recovery_) {
                publishHorizon();
                force_horizon_refresh_ = false;
            }
            updateSnapshotWedgeTrigger();
            publishRealOutputs(dt);
            marker_pub_->publish(
                agents_, visited_slots_, rule_engine_->conflicts(),
                marker_pub_->hasSubscribers()
                    ? rule_engine_->conflictResourceMarkers(agents_)
                    : std::vector<forklift_planner::multi_vehicle::
                          ConflictMarker>{},
                rule_engine_->futureA1Commitment(),
                rule_engine_->a1DepartureClusters(),
                rule_engine_->recoveryDirective());
            publishRealTrailMarkers();
            return;
        }

        //=======仿真模式=======
        // dt = 1.0 / pp_.update_rate; dt= 1 / 10 = 0.1s。

        // 从当前仿真状态推进一拍。滚动模式下，普通协调只在构建10秒计划时
        // 运行；随后20拍(2秒)逐拍执行冻结计划。0.1秒层只保留任务事件、
        // 运动学推进和 advanceVehicles 内不可关闭的物理碰撞兜底。
        updateDwellAndTasks(dt);
        if (rb_one_shot_traj_) {
            rule_engine_->decide(agents_, dt);
            marker_pub_->setRollingDecision(
                rule_engine_->lastRollingDynamicDecision());
        } else {
            if (simulationPlanNeedsRefresh()) {
                publishHorizon();
                force_horizon_refresh_ = false;
            }
            if (!executeSimulationPlanSample()) {
                // A task/path event may invalidate a just-built frame. Rebuild
                // once immediately; only fall back to a current decision if
                // planning itself produced no executable frame.
                publishHorizon();
                force_horizon_refresh_ = false;
                if (!executeSimulationPlanSample()) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[sim_plan] no executable frame; using one safe "
                        "current-step decision");
                    rule_engine_->decide(agents_, dt);
                    marker_pub_->setRollingDecision(
                        rule_engine_->lastRollingDynamicDecision());
                }
            }
        }
        advanceVehicles(dt);
        diagnoseA1ExitIntrusions();
        recordDebugTimelineTick();


        //4. 仿真模式---一次性触发完成
        updateSnapshotWedgeTrigger();

        if (rb_one_shot_traj_) {    
            if (!one_shot_published_) one_shot_published_ = publishFullTrajectories();
        }

        logAgentStatus();
        logStuckDiagnostics();
        marker_pub_->publish(
            agents_, visited_slots_, rule_engine_->conflicts(),
            marker_pub_->hasSubscribers()
                ? rule_engine_->conflictResourceMarkers(agents_)
                : std::vector<forklift_planner::multi_vehicle::
                      ConflictMarker>{},
            rule_engine_->futureA1Commitment(),
            rule_engine_->a1DepartureClusters(),
            rule_engine_->recoveryDirective());
    }
    
    //===========================================================================


    void publishSimTrackMarkers() {
        visualization_msgs::MarkerArray arr;
        for (size_t i = 0; i < agents_.size(); ++i) {
            const RoughPath& path = agents_[i].track.path();
            if (path.size() < 2) continue;
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id; m.header.stamp = ros::Time::now();
            m.ns = "sim_track"; m.id = agents_[i].id;
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0; m.scale.x = 0.02;
            m.color = agents_[i].color; m.color.a = 0.9;
            for (const auto& p : path) {
                geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = 0.08;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    void publishRealTrailMarkers() {
        visualization_msgs::MarkerArray arr;
        const ros::Time now = ros::Time::now();
        for (size_t i = 0; i < real_trails_.size(); ++i) {
            if (real_trails_[i].size() < 2) continue;
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = now;
            m.ns = "real_trail";
            m.id = agents_[i].id;
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.018;
            if (i < agents_.size()) {
                m.color = agents_[i].color;
            } else {
                m.color = rgba(1.0f, 1.0f, 1.0f, 1.0f);
            }
            m.color.a = 1.0;
            for (const auto& p : real_trails_[i]) m.points.push_back(p);
            arr.markers.push_back(m);
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    // ───────── 实车模式:I/O 建立 + 摆位打印 ─────────
    void setupRealIO() {
        const int n = static_cast<int>(agents_.size());
        traj_pubs_.resize(n);
        speed_pubs_.resize(n);
        state_pubs_.resize(n);
        real_x_.assign(n, 0.0);
        real_y_.assign(n, 0.0);
        real_yaw_.assign(n, 0.0);
        real_pose_ok_.assign(n, false);
        rb_prev_path_s_.assign(n, 0.0);
        rb_speed_windows_.assign(
            n, forklift_planner::multi_vehicle::ArcLengthSpeedWindow(0.4));
        rb_speed_identity_.assign(n, std::make_tuple(-1, -1, -1));
        rb_motion_x_.assign(n, 0.0);
        rb_motion_y_.assign(n, 0.0);
        rb_motion_pose_valid_.assign(n, false);
        rb_cmd_speed_.assign(n, 0.0);
        rb_last_seen_.assign(n, 0.0);
        rb_published_gen_.assign(n, -1);
        one_shot_done_.assign(n, false);
        rb_track_gen_.assign(n, -1);
        rb_logged_gen_.assign(n, -1);
        real_trails_.assign(n, {});
        for (int i = 0; i < n; ++i) {
            const int vehicle_id = agents_[static_cast<size_t>(i)].id;
            ros::AdvertiseOptions traj_options;
            traj_options.init<sandbox_msgs::Trajectory>(
                "/traj_" + std::to_string(vehicle_id), 1);
            // ROS1 normally overwrites Header.seq with its publication counter.
            // This topic deliberately owns seq as the stable mission path_gen.
            traj_options.has_header = false;
            traj_options.latch = true;
            traj_pubs_[i] = nh_.advertise(traj_options);
            speed_pubs_[i] = nh_.advertise<std_msgs::Float64>(
                "/coord_speed_" + std::to_string(vehicle_id), 1,
                /*latch=*/false);
            state_pubs_[i] = nh_.advertise<std_msgs::String>(
                "/coord_state_" + std::to_string(vehicle_id), 1,
                /*latch=*/false);
        }
        object_sub_ = nh_.subscribe("/object", 20,
                                    &MultiVehiclePatrolNode::objectCallback, this);
        // 启动/急停键(由独立键盘节点 estop_key.py 发):Enter→/rb_start 开跑;空格→/estop 切换急停。
        // 独立节点是因为 roslaunch 起的本节点拿不到终端 stdin,键盘要在自己的终端 tab 里读。
        start_sub_ = nh_.subscribe("/rb_start", 1,
                                   &MultiVehiclePatrolNode::rbStartCallback, this);
        estop_sub_ = nh_.subscribe("/estop", 1,
                                   &MultiVehiclePatrolNode::estopCallback, this);
        // AD 滚动时域参数:推演/发布时长 + 刷新周期(拍)。
    
        rb_horizon_ = cfg_.rolling_horizon;
        rb_horizon_refresh_period_ = cfg_.rolling_refresh_period;
        rb_horizon_refresh_ = std::max(1, (int)std::lround(rb_horizon_refresh_period_ * pp_.update_rate));
       
        ROS_WARN("[real] AD rolling horizon: horizon=%.1fs, refresh every %d ticks",
         rb_horizon_, rb_horizon_refresh_);
        // 一次性整条轨迹模式(默认开):start 后推演全程发一次 latch,对接原版 pure_pursuit 按时间跟踪。
        
        rb_one_shot_traj_ = cfg_.one_shot_traj;
        ros::param::param("~full_horizon", rb_full_horizon_, rb_full_horizon_);
        ROS_WARN("[real] 轨迹发布模式: %s (full_horizon=%.0fs)",
                 rb_one_shot_traj_ ? "一次性整条/纯盲跟" : "AD滚动时域", rb_full_horizon_);
        // 起始摆位标记:发到现有 marker topic(不同 ns),RViz 不改配置即可显示足迹+ID+朝向。
        start_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 1, /*latch=*/true);
        horizon_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);   // 推演轨迹可视化(同topic不同ns,RViz直接显示)
        // 方案一:打印每辆车应摆放的真实坐标(track 起点)。
        ROS_WARN("==== 实车摆位(请把每辆车按编号摆到下列位置, 单位 m, yaw 弧度)====");
        std::ofstream ofs(realbridge_positions_file_);
        ofs << "# 实车摆位(按编号)。id slot x y yaw,单位 m/rad。\n";
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) { ROS_WARN("  车 %d: (无路径)", v.id); continue; }
            const auto p0 = v.track.poseAtS(0.0);
            ROS_WARN("  车 %d → 库位 %d, (x=%.3f, y=%.3f, yaw=%.3f)",
                     v.id, v.current_slot, p0.x, p0.y, p0.theta);
            char line[160];
            std::snprintf(line, sizeof(line), "%3d %4d %7.3f %7.3f %7.3f\n",
                          v.id, v.current_slot, p0.x, p0.y, p0.theta);
            ofs << line;
        }
        ofs.close();
        publishStartMarkers();  // RViz 画出每车起点:足迹框 + ID 数字 + 朝向箭头
    }

    // 起始摆位 RViz 标记:每车在 track 起点画 足迹框(LINE_STRIP)+ ID 文字 + 朝向箭头,
    // 各车独立颜色,latched 发到 /forklift_planner/markers(ns=start_*,与运行期 marker 不冲突)。
    void publishStartMarkers() {
        auto colorFor = [](int id) {
            static const float c[8][3] = {{1,0,0},{0,1,0},{0,0.5,1},{1,0.85,0},
                                          {1,0,1},{0,1,1},{1,0.5,0},{0.7,0.7,0.7}};
            std_msgs::ColorRGBA col; col.a = 1.0;
            col.r = c[id % 8][0]; col.g = c[id % 8][1]; col.b = c[id % 8][2];
            return col;
        };
        visualization_msgs::MarkerArray arr;
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) continue;
            const RoughWp p0 = v.track.poseAtS(0.0);
            // 车身足迹:用半透明 CUBE(车身几何中心 + 朝向 + 车长×车宽),抬到 z 上方避免被
            // 地图遮住(之前 LINE_STRIP 太细且与地图同平面 → 看不见)。半透明可对位。
            const RoughWp bc = forklift_planner::multi_vehicle::bodyCenterPose(p0, mp_);
            visualization_msgs::Marker box;
            box.header.frame_id = pp_.frame_id;
            box.header.stamp = ros::Time::now();
            box.ns = "start_footprint"; box.id = v.id;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.position.x = bc.x; box.pose.position.y = bc.y; box.pose.position.z = 0.02;
            box.pose.orientation.z = std::sin(bc.theta / 2.0);
            box.pose.orientation.w = std::cos(bc.theta / 2.0);
            box.scale.x = mp_.vehicle_length; box.scale.y = mp_.vehicle_width; box.scale.z = 0.03;
            box.color = colorFor(v.id); box.color.a = 0.35;  // 半透明
            arr.markers.push_back(box);
            // ID 文字
            visualization_msgs::Marker txt;
            txt.header = box.header;
            txt.ns = "start_id"; txt.id = v.id;
            txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::Marker::ADD;
            txt.pose.position.x = p0.x; txt.pose.position.y = p0.y; txt.pose.position.z = 0.12;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.10;
            txt.color = colorFor(v.id);
            txt.text = std::to_string(v.id);
            arr.markers.push_back(txt);
            // 朝向箭头:从车身中心沿车头方向画一条明显的箭头(抬到车身上方 z=0.06,
            // ARROW 用两点时 scale.x=杆径/scale.y=箭头径/scale.z=箭头长——之前 scale.z=0 没箭头)。
            visualization_msgs::Marker ar;
            ar.header = box.header;
            ar.ns = "start_heading"; ar.id = v.id;
            ar.type = visualization_msgs::Marker::ARROW;
            ar.action = visualization_msgs::Marker::ADD;
            geometry_msgs::Point a0, a1;
            a0.x = bc.x; a0.y = bc.y; a0.z = 0.06;
            a1.x = bc.x + 0.18 * std::cos(p0.theta);
            a1.y = bc.y + 0.18 * std::sin(p0.theta); a1.z = 0.06;
            ar.points.push_back(a0); ar.points.push_back(a1);
            ar.scale.x = 0.02; ar.scale.y = 0.045; ar.scale.z = 0.05;
            ar.color = colorFor(v.id); ar.color.a = 1.0;
            ar.pose.orientation.w = 1.0;
            arr.markers.push_back(ar);
        }
        // 地图原点+XY轴一并经 latched 话题先发(real_mode tick 要等所有车动捕就绪才publish,
        // 摆车前看不到轴 → 在这儿先发,标定时立刻可见)。
        marker_pub_->addOriginAxes(arr);
        start_marker_pub_.publish(arr);
        ROS_WARN("[multi_patrol] 已在 RViz 画出 8 车起始摆位(足迹+ID+朝向)+地图原点XY轴,对着摆即可。");
        ROS_WARN("[multi_patrol] 摆好后在【键盘节点终端】按 Enter 开跑;运行中按 空格 = 急停(再按解除)。");
    }

    // /object(动捕,mm,后轮中心=后轴参考)→ 各车真实位姿(转米)
    void objectCallback(const sandbox_msgs::AprilObject::ConstPtr& msg) {
        if (msg->type != sandbox_msgs::AprilObject::VEHICLE) return;
        const auto agent_it = std::find_if(
            agents_.begin(), agents_.end(),
            [msg](const VehicleAgent& vehicle) { return vehicle.id == msg->id; });
        if (agent_it == agents_.end()) return;
        const size_t i = static_cast<size_t>(agent_it - agents_.begin());
        real_x_[i] = msg->x / 1000.0;  // mm→m(nokov 发 mm,pure_pursuit 也 /1000)
        real_y_[i] = msg->y / 1000.0;
        real_yaw_[i] = msg->yaw;
        real_pose_ok_[i] = true;
        rb_last_seen_[i] = ros::Time::now().toSec();  // 动捕看门狗:记最后一次见到的时刻
        // RViz 显示真实位姿(实际位置,非投影):同步进 agent 供 marker 用。
        agents_[i].real_pose_valid = true;
        agents_[i].real_x = real_x_[i];
        agents_[i].real_y = real_y_[i];
        agents_[i].real_yaw = real_yaw_[i];

        if (i < real_trails_.size()) {
            geometry_msgs::Point p;
            p.x = real_x_[i];
            p.y = real_y_[i];
            p.z = 0.12;
            auto& trail = real_trails_[i];
            const bool moved_enough =
                trail.empty() || std::hypot(p.x - trail.back().x, p.y - trail.back().y) > 0.01;
            if (moved_enough) {
                trail.push_back(p);
                constexpr size_t kMaxRealTrailPoints = 2000;
                while (trail.size() > kMaxRealTrailPoints) trail.pop_front();
            }
        }
    }

    // Enter:摆位完成、开跑。未全部就位也允许启动(未就位的车由动捕看门狗摁停),但会告警。
    void rbStartCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data || rb_started_) return;
        int missing = 0;
        for (size_t i = 0; i < real_pose_ok_.size(); ++i) {
            if (!real_pose_ok_[i]) ++missing;
        }
        rb_started_ = true;
        if (missing > 0)
            ROS_WARN("[real] 收到启动,但还有 %d 辆车动捕未就位 → 它们会被看门狗摁停,直到被看到。", missing);
        ROS_WARN("[real] *** 已启动:开始协调推进 ***");
    }

    // 空格:急停切换。true=全车瞬时停;再按一下=false 恢复协调速度。
    void estopCallback(const std_msgs::Bool::ConstPtr& msg) {
        const bool was_estopped = rb_estop_;
        rb_estop_ = msg->data;
        if (rb_estop_) {
            // Close the coordinated-speed path immediately and publish the
            // existing current-pose hold trajectory for the PP path.
            for (size_t i = 0; i < speed_pubs_.size(); ++i) {
                rb_cmd_speed_[i] = 0.0;
                std_msgs::Float64 zero;
                zero.data = 0.0;
                speed_pubs_[i].publish(zero);
            }
            publishHoldAll();
        } else if (was_estopped) {
            // Never resume from the prediction frozen before estop. The next
            // measured-state tick rebuilds and publishes a fresh horizon.
            real_plan_valid_ = false;
            force_horizon_refresh_ = true;
        }
        ROS_ERROR("[real] *** 急停 %s ***", rb_estop_ ? "已触发(全车停)" : "已解除(恢复)");
    }

    // 摆位阶段节流播报:哪些车动捕已到位 / 还缺哪些(否则启动门控是"哑"的,现场不知在等谁)。
    void logPlacementStatus() {
        std::string seen, miss;
        for (size_t i = 0; i < real_pose_ok_.size(); ++i) {
            (real_pose_ok_[i] ? seen : miss) +=
                "V" + std::to_string(agents_[i].id) + " ";
        }
        if (miss.empty())
            ROS_WARN_THROTTLE(2.0, "[real] 全部就位 ✓ [%s] —— 去【启动/急停键盘】终端按 Enter 启动"
                              "(在打印本日志的终端里按 Enter 无效!)", seen.c_str());
        else
            ROS_WARN_THROTTLE(2.0, "[real] 摆位中:已到位 [%s] 还缺 [%s](等动捕看到)。"
                              "齐了去【启动/急停键盘】终端按 Enter。", seen.c_str(), miss.c_str());
    }

    // 替代 advanceVehicles 的「位置推进」:实车位置取自 /object 投影,不做 sim 积分/硬护栏。
    // 到库→DWELL 逐字节复刻 advanceVehicles 705-714,保证和 sim 同样的"到点停 10s"。
    void realAdvance(double dt) {
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            if (v.mode != VehicleMode::ACTIVE || v.track.empty()) continue;
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;  // 动捕还没看到这辆 → 别拿 (0,0) 投影出垃圾 path_s
            if (rb_track_gen_[i] != v.path_gen) {  // 新任务/新路径 → path_s 归零
                rb_track_gen_[i] = v.path_gen;
                rb_prev_path_s_[i] = 0.0;
            }
            const auto speed_identity = std::make_tuple(
                v.path_gen, static_cast<int>(v.mission_phase),
                static_cast<int>(v.leg_target) + 10 * static_cast<int>(
                    rule_engine_->motionOverrideFor(v.id).motion));
            if (rb_speed_identity_[i] != speed_identity) {
                rb_speed_windows_[i].clear(0.0);
                rb_speed_identity_[i] = speed_identity;
                rb_motion_pose_valid_[i] = false;
            }
            const double previous_path_s = rb_prev_path_s_[i];
            const auto motion_override = rule_engine_->motionOverrideFor(v.id);
            const auto recovery_motion = motion_override.motion;
            const int progress_direction =
                recovery_motion ==
                    forklift_planner::multi_vehicle::RecoveryMotion::RETREAT
                    ? -1 : 1;
            const double lo = progress_direction < 0
                ? std::max(motion_override.target_s,
                           rb_prev_path_s_[i] - 0.50)
                : std::max(0.0, rb_prev_path_s_[i] - 0.10);
            const double hi = progress_direction < 0
                ? rb_prev_path_s_[i]
                : std::min(v.track.length(), rb_prev_path_s_[i] + 0.50);
            bool motion_heading_valid = false;
            double motion_heading = 0.0;
            if (rb_motion_pose_valid_[i]) {
                const double dx = real_x_[i] - rb_motion_x_[i];
                const double dy = real_y_[i] - rb_motion_y_[i];
                if (std::hypot(dx, dy) >= 0.003) {
                    motion_heading = std::atan2(dy, dx);
                    motion_heading_valid = true;
                }
            }
            const auto projection =
                forklift_planner::multi_vehicle::selectRealProjection(
                    v.track, real_x_[i], real_y_[i], real_yaw_[i],
                    previous_path_s, v.current_speed, dt, lo, hi,
                    motion_heading_valid, motion_heading, 0.08,
                    progress_direction);
            const double new_s = projection.path_s;
            const double sample_time = ros::Time::now().toSec();
            const auto speed = rb_speed_windows_[i].update(
                sample_time, new_s, previous_path_s, dt, cfg_.max_speed,
                progress_direction);
            v.current_speed = speed.window_speed;
            v.path_s = new_s;
            rb_prev_path_s_[i] = new_s;
            rb_motion_x_[i] = real_x_[i];
            rb_motion_y_[i] = real_y_[i];
            rb_motion_pose_valid_[i] = true;
            logRealProjectionSample(i, previous_path_s, new_s, lo, hi,
                                    speed, projection);
            // 到库→DWELL —— 复刻 advanceVehicles 705-714(实车容差 cfg_.real_arrive_tol)
            if (progress_direction > 0 &&
                v.path_s >= v.track.length() - cfg_.real_arrive_tol) {
                // 关键(联动 rule_engine):sim 里 DWELL 车 path_s≈length,故其碰撞足迹
                // poseAtS(path_s)=槽位;rule_engine 73/85/737/742 处直接用 poseAtS(path_s)
                // 算静止车足迹(没套 DWELL?length:path_s)。实车若停在 length-5cm,这些足迹
                // 就比 sim 偏 5cm → 与停驻车的冲突判定偏离仿真。故到点即把 path_s 夹到 length,
                // 让"协调眼里的 DWELL 车"=精确槽位,与 sim 逐字节一致(实际 5cm 物理差归控制器管)。
                v.path_s = v.track.length();
                rb_prev_path_s_[i] = v.track.length();
                handleLegArrival(v);
                rb_speed_windows_[i].clear(0.0);
                rb_speed_identity_[i] = std::make_tuple(-1, -1, -1);
                rb_motion_pose_valid_[i] = false;
            }
        }
        // Keep one row per measured 0.1 s tick while an enabled vehicle is
        // waiting/dwelling too. No projection is performed in these modes;
        // NaN search bounds distinguish them from active projection samples.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (size_t i = 0; i < agents_.size(); ++i) {
            const VehicleAgent& v = agents_[i];
            if (!real_pose_ok_[i]) continue;
            if (v.mode == VehicleMode::ACTIVE && !v.track.empty()) continue;
            rb_speed_windows_[i].clear(0.0);
            forklift_planner::multi_vehicle::ArcLengthSpeedResult speed;
            speed.window_speed = v.current_speed;
            forklift_planner::multi_vehicle::RealProjectionResult projection;
            logRealProjectionSample(i, v.path_s, v.path_s, nan, nan,
                                    speed, projection);
        }
    }

    // 实车硬护栏(第二层兜底,替代 sim 里 advanceVehicles 的 hard_collision_guard):
    // 用真实 /object 位姿算两两足迹(充气 real_emergency_margin),重叠即双方急停。
    // 预测层(decide)是第一层;它漏判时这层兜住,防真车相撞。返回每车是否需急停。
    // 设计取舍:margin<预测层间距→正常不触;只在两车逼近到 <margin 才停(双停=安全优先,
    // 死活留给协调/死锁检测理顺)。DWELL/idle 车也算静态障碍(用其真实位姿)。
    std::vector<bool> realHardGuard() {
        std::vector<bool> estop(agents_.size(), false);
        const double m = cfg_.real_emergency_margin;
        if (m <= 0.0) return estop;  // 0 = 关闭
        auto realBody = [&](size_t i) {
            RoughWp p; p.x = real_x_[i]; p.y = real_y_[i];
            p.theta = real_yaw_[i]; p.type = WpType::FORWARD;
            return forklift_planner::multi_vehicle::makeBody(p, mp_, m);
        };
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;
            const auto bi = realBody(i);
            for (size_t j = i + 1; j < agents_.size(); ++j) {
                if (!real_pose_ok_[j]) continue;
                if (forklift_planner::multi_vehicle::overlaps(bi, realBody(j))) {
                    // 只急停"在动的"那辆(ACTIVE);停驻/空闲车本就静止,标它无意义且会污染状态。
                    // 静态车是障碍方,移动车才是要被拦下的一方(若两辆都在动则都停)。
                    bool any = false;
                    if (agents_[i].mode == VehicleMode::ACTIVE) { estop[i] = true; any = true; }
                    if (agents_[j].mode == VehicleMode::ACTIVE) { estop[j] = true; any = true; }
                    if (any) {
                        ROS_ERROR_THROTTLE(0.5, "[real] 硬护栏急停: V%d 与 V%d 实测足迹逼近"
                                           "(<%.2fm)。预测层疑似漏判,查 logger 车间距/coord_flag。",
                                           agents_[i].id, agents_[j].id, m);
                    }
                }
            }
        }
        return estop;
    }

    // 曲率限速(规划侧运动学):弯道允许速度 v≤√(a_lat/κ)。用 path_s 附近三点位置估 κ(Menger,
    // 对动捕/cusp 比航向差稳);限到 [creep, +∞)避免曲率尖点把车停死。返回该车此刻的速度上限。
    double curvatureSpeed(const VehicleAgent& v) const {
        if (cfg_.lat_accel_max <= 0.0 || v.track.empty()) return 1e9;  // 关闭
        const double L = v.track.length(), ds = 0.05;
        const double s = std::min(std::max(v.path_s, 0.0), L);
        const auto A = v.track.poseAtS(std::max(0.0, s - ds));
        const auto B = v.track.poseAtS(s);
        const auto C = v.track.poseAtS(std::min(L, s + ds));
        const double abx=B.x-A.x, aby=B.y-A.y, acx=C.x-A.x, acy=C.y-A.y;
        const double lab=std::hypot(abx,aby), lbc=std::hypot(C.x-B.x,C.y-B.y), lac=std::hypot(acx,acy);
        if (lab<1e-4||lbc<1e-4||lac<1e-4) return 1e9;
        const double kappa = 2.0*std::fabs(abx*acy - aby*acx)/(lab*lbc*lac);  // Menger κ
        if (kappa < 1e-3) return 1e9;
        const double vc = std::sqrt(cfg_.lat_accel_max / kappa);
        const double v_floor = cfg_.nominal_speed * cfg_.creep_ratio;          // 弯再急也不低于 creep,不停死
        return std::max(vc, v_floor);
    }

    // 发 /traj_i(路径,换任务才重发) + /coord_speed_i(带符号实时速度,每拍)
    void publishRealOutputs(double dt) {
        const double now = ros::Time::now().toSec();
        const std::vector<bool> estop = realHardGuard();  // 第二层:真实足迹逼近→急停
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            // 注:几何 /traj 已由 publishHorizon(滚动时域时间参数化轨迹)发布,这里不再发 /traj。
            // 速度幅值=协调动作档,再被曲率限速卡住(规划侧运动学:弯道降速)。方向=路径段(倒车负)。STOP→0。
            const auto recovery_motion =
                rule_engine_->motionOverrideFor(v.id).motion;
            const VehicleAction motion_action =
                recovery_motion ==
                        forklift_planner::multi_vehicle::RecoveryMotion::RETREAT
                    ? VehicleAction::CREEP : v.action;
            double mag = recovery_motion ==
                    forklift_planner::multi_vehicle::RecoveryMotion::HOLD
                ? 0.0
                : (recovery_motion ==
                           forklift_planner::multi_vehicle::RecoveryMotion::RETREAT
                       ? cfg_.deadlock_retreat_speed
                       : rule_engine_->speedForAction(motion_action));
            mag = std::min(mag, curvatureSpeed(v));   // 曲率限速(lat_accel_max,0=关)
            // 方向:当前段倒车,或【前方一小段即将进入倒车段】→ 负(倒车)。后者关键:realAdvance 的
            // path_s 单调只增,前进逼近 FORWARD→REVERSE 的 cusp 时,path_s 越不过 cusp(前进会冲偏、
            // 投影卡在 cusp)→ 若只看 typeAtS(path_s) 永远 FORWARD → 车冲过该倒车处不倒(sim 积分不暴露)。
            // 故前瞻 0.10m:逼近 cusp 即提前给负速度,车减速→cusp 停→倒入倒车段→path_s 越过,死锁解开。
            const int progress_direction = recovery_motion ==
                    forklift_planner::multi_vehicle::RecoveryMotion::RETREAT
                ? -1 : 1;
            const double dir =
                forklift_planner::multi_vehicle::signedPathMotionDirection(
                    v.track, v.path_s, progress_direction);
            double target = dir * mag;
            // 动捕看门狗:该车位姿失联(>0.5s)→ 强制目标速度 0。否则控制器拿陈旧位姿
            // 盲走、底盘又无超时 → 跑飞。只压这一辆的输出速度,协调逻辑/其它车不受影响
            //(协调端看它 path_s 不动,自会让旁车等它,安全)。
            // real_pose_timeout ≤ 0 → 关闭动捕失联停(与 emerg_margin=0、cmd_timeout≤0 统一约定)
            const bool stale = cfg_.real_pose_timeout > 0.0 &&
                               (now - rb_last_seen_[i]) > cfg_.real_pose_timeout;
            const char* flag = "OK";
            bool instant_zero = false;
            if (rb_estop_) {          // 操作员空格急停:最高优先级,瞬时 0(不走斜坡,要快)
                target = 0.0; instant_zero = true; flag = "EKEY";
            } else if (estop[i]) {    // 第二层硬护栏:真实足迹逼近 → 急停
                target = 0.0;
                flag = "ESTOP";
            } else if (stale) {       // 动捕失联 → 强制 0,防控制器拿陈旧位姿盲走跑飞
                target = 0.0;
                flag = "STALE";
                ROS_WARN_THROTTLE(1.0, "[real] 车 %zu 动捕失联 %.1fs → 强制STOP",
                                  i, now - rb_last_seen_[i]);
            }
            // 加速度限制(与 sim 一致):按 max_accel/max_decel 斜坡逼近目标,而非阶跃。
            // 这样真车速度剖面 = 仿真,停在协调按 max_decel 算的停止线处;cusp 处平滑过 0 反向。
            // 注:硬护栏/STALE 走斜坡降速(避免甩动,距离近时本就低速);操作员急停 EKEY 例外,瞬时归 0。
            if (instant_zero) rb_cmd_speed_[i] = 0.0;
            else rb_cmd_speed_[i] = limitedSpeed(rb_cmd_speed_[i], target, dt);
            std_msgs::Float64 sp;
            sp.data = rb_cmd_speed_[i];
            speed_pubs_[i].publish(sp);

            // 只读协调状态。格式 "mode,action,blk,wait,slot->tgt,flag,seg,s/len" 供 logger 落列。
            // seg=当前路径段方向(FWD/REV,来自 typeAtS(path_s));s/len=路径进度——排查"该倒车却前进"。
            const char* seg = (dir < 0.0) ? "REV" : "FWD";
            const double len = v.track.empty() ? 0.0 : v.track.length();
            char buf[140];
            std::snprintf(buf, sizeof(buf), "%s,%s,%d,%.1f,%d->%d,%s,%s,%.2f/%.2f",
                          modeName(v.mode), actionName(v.action), v.blocker_id,
                          v.wait_time,
                          v.current_slot, v.target_slot, flag, seg, v.path_s, len);
            std_msgs::String st;
            st.data = buf;
            state_pubs_[i].publish(st);
            // 在动时节流打印:进度+段向+协调速度,实时看"倒车段是否被识别、coord_speed 是否变负"。
            if (v.mode == VehicleMode::ACTIVE)
                ROS_INFO_THROTTLE(1.0, "[real] V%d s=%.2f/%.2f seg=%s coord_speed=%+.2f action=%s",
                                  v.id, v.path_s, len, seg, rb_cmd_speed_[i], actionName(v.action));
        }
    }

    ros::NodeHandle nh_;
    ros::Timer timer_;

    MapParam mp_;
    PlannerParam pp_;
    forklift_planner::multi_vehicle::MultiVehicleConfig cfg_;

    std::unique_ptr<ForkliftMap> map_;
    std::unique_ptr<PathGenerator> generator_;
    std::unique_ptr<forklift_planner::multi_vehicle::TaskAllocator> allocator_;
    std::unique_ptr<forklift_planner::multi_vehicle::RuleEngine> rule_engine_;
    bool one_shot_ = false;  // false: continuously execute B->A1->B transports
    std::unique_ptr<forklift_planner::multi_vehicle::MarkerPublisher> marker_pub_;
    std::unique_ptr<forklift_planner::multi_vehicle::TrafficResourceMap> resource_map_;
    std::vector<VehicleAgent> agents_;
    std::vector<bool> visited_slots_;
    std::string debug_log_dir_;
    std::string coord_log_file_;
    bool coord_log_enabled_ = true;
    bool stress_watchdog_enabled_ = false;
    bool stress_quiet_ = false;
    double stress_progress_timeout_ = 120.0;
    std::string stress_result_file_;
    std::string stress_failure_file_;
    std::string onset_log_file_;
    std::string realbridge_positions_file_;
    std::ofstream coord_log_;
    std::vector<std::ofstream> real_projection_logs_;
    std::string coord_log_source_ = "REAL";
    uint64_t coord_log_plan_id_ = 0;
    int coord_log_frame_id_ = -1;
    int coord_log_rollout_step_ = -1;
    bool coord_log_suppressed_ = false;
    uint64_t rollout_log_id_ = 0;

    // ── 实车模式(real_mode)I/O ──────────────────────────────────────────────
    ros::Subscriber object_sub_;                       // /object 动捕位姿(mm)
    ros::Publisher start_marker_pub_;                  // 起始摆位标记(足迹框+ID+朝向,latched)
    std::vector<ros::Publisher> traj_pubs_, speed_pubs_;// /traj_i + /coord_speed_i
    std::vector<ros::Publisher> state_pubs_;            // /coord_state_i(只读调试:停车原因)
    ros::Publisher horizon_marker_pub_;                 // 推演 5s 轨迹的 RViz 可视化(LINE_STRIP/车)
    ros::Publisher snapshot_trigger_pub_;
    bool snapshot_debug_enabled_ = false;
    double debug_timeline_start_ = -1.0;
    double debug_timeline_end_ = -1.0;
    std::string snapshot_trigger_topic_;
    bool snapshot_wedge_active_ = false;
    unsigned long long snapshot_last_trigger_tick_ = 0;
    ros::Subscriber start_sub_, estop_sub_;             // /rb_start(Enter开跑) /estop(空格急停切换)
    bool rb_started_ = false;                           // 摆位完成、按Enter后才推进
    bool rb_estop_ = false;                             // 操作员急停:true=全车瞬时停
    bool rb_estop_prev_ = false;                        // 上一拍急停态(检测按下/解除边沿,one_shot急停用)
    
    double rb_horizon_ = 10.0;                           // AD 滚动时域:每次推演/发布的未来时长(s)
    double rb_horizon_refresh_period_ = 2.0;
    int rb_horizon_refresh_ = 20;                        // 每多少拍重新推演刷新一次(5拍=0.5s)
    bool rb_one_shot_traj_ = false;                      // 一次性整条轨迹模式(默认):start后推演全程发一次latch
    bool force_horizon_refresh_ = false;                 // 新航段安装后立即覆盖发布

    // Simulation executes the predicted frames. Real mode only keeps the
    // measured-state plan identity and frame-0 ordinary decision below.
    std::vector<SimPlanFrame> sim_plan_frames_;
    size_t sim_plan_cursor_ = 0;
    bool sim_plan_valid_ = false;
    uint64_t sim_plan_id_ = 0;
    double sim_plan_start_time_ = 0.0;
    bool real_plan_valid_ = false;
    double real_plan_start_time_ = 0.0;
    forklift_planner::multi_vehicle::RuleEngine::RollingDynamicDecision
        real_period_ordinary_decision_;
    std::vector<RealPlanAgentIdentity> real_plan_agents_;
    std::vector<DepartureTransactionIdentity>
        real_plan_departure_transactions_;
    RecoveryIdentity real_plan_recovery_{};
    A1LaunchMetrics a1_launch_metrics_;
    std::map<int, A1LaunchHoldState> a1_launch_holds_;
    ExecutedRollingDecisionMetrics executed_rolling_metrics_;
    bool previous_ordinary_conflict_active_ = false;
    
    double rb_full_horizon_ = 180.0;                    // 一次性模式:全程推演上限时长(s),尾部静止点会裁掉
    bool one_shot_published_ = false;                   // 一次性轨迹是否已全部发完(防重复发)
    std::vector<bool> one_shot_done_;                   // 各车一次性轨迹是否已发(动捕晚到的车就位后补发)
    std::vector<int> rb_logged_gen_;                    // 已打印倒车段诊断的 path_gen(每车,防重复刷屏)
    std::vector<double> real_x_, real_y_, real_yaw_;   // 各车真实位姿(已 /1000 转米)
    std::vector<std::deque<geometry_msgs::Point>> real_trails_; // RViz 实车走过的真实轨迹(ns=real_trail)
    std::vector<bool> real_pose_ok_;                   // 动捕是否已收到该车
    std::vector<double> rb_prev_path_s_;               // 上一拍 path_s(局部投影)
    std::vector<forklift_planner::multi_vehicle::ArcLengthSpeedWindow>
        rb_speed_windows_;
    std::vector<std::tuple<int, int, int>> rb_speed_identity_;
    std::vector<double> rb_motion_x_, rb_motion_y_;
    std::vector<bool> rb_motion_pose_valid_;
    std::vector<double> rb_cmd_speed_;                 // 上一拍发出的速度命令(带符号),斜坡限速用
    std::vector<double> rb_last_seen_;                 // 各车动捕最后到达时刻(s),看门狗用
    // 动捕失联超时、到点容差 → 已提为 ROS 参数 cfg_.real_pose_timeout / real_arrive_tol,
    // 现场可不重编译直接调。到点容差需 > PP 的 2cm 硬停容差,否则 PP 停在 2cm 短处而 path_s
    // 到不了 length → 永不 DWELL → 不触发下一个任务(卡死);默认 5cm。
    std::vector<int> rb_published_gen_, rb_track_gen_; // 已发布的 path_gen / 上次见到的 path_gen
    std::vector<VehicleMode> last_logged_mode_;
    std::vector<MissionPhase> last_logged_mission_phase_;
    std::vector<VehicleAction> last_logged_action_;
    std::vector<std::string> last_logged_reason_;
    std::vector<int> last_logged_blocker_;
    std::vector<int> last_logged_task_count_;
    std::vector<ros::Time> last_status_log_time_;
    std::vector<ros::Time> last_diag_time_;  // TEMPORARY: [DIAG stuck] throttle
    unsigned long long tick_count_ = 0;
    std::set<std::pair<int, int>> active_a1_exit_intrusions_;
    double sim_time_ = 0.0;

    // 无头批处理(快速回归)统计。确定性仿真:同种子同代码必得同结果,故脱离 RViz、
    // 不按实时狂跑 N 拍即可在几秒内覆盖数小时仿真,直接数碰撞。
    unsigned long long hard_guard_events_ = 0;        // 硬护栏触发(碰撞)累计次数
    unsigned long long first_guard_tick_ = 0;         // 首次碰撞所在 tick(0=从未)
    std::set<std::pair<int, int>> hard_guard_pairs_;  // 涉及碰撞的车对
    bool sim_mode_ = false;                           // 前瞻仿真中:屏蔽计数/日志副作用

public:
    void recordDebugTimelineTick() {
        if (debug_timeline_start_ < 0.0 ||
            sim_time_ + 1e-9 < debug_timeline_start_ ||
            sim_time_ - 1e-9 > debug_timeline_end_) {
            return;
        }
        for (const VehicleAgent& vehicle : agents_) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[TIMELINE_STATE] tick=" << tick_count_
                 << " sim_t=" << sim_time_
                 << " plan=" << sim_plan_id_
                 << " frame=" << coord_log_frame_id_
                 << " V" << vehicle.id
                 << " phase=" << missionPhaseName(vehicle.mission_phase)
                 << " gen=" << vehicle.path_gen
                 << " task=" << vehicle.task_count
                 << " slot=" << vehicle.current_slot
                 << "->" << vehicle.target_slot
                 << " s=" << vehicle.path_s
                 << " speed=" << vehicle.current_speed
                 << " action=" << actionName(vehicle.action)
                 << " reason=" << vehicle.reason
                 << " blocker=" << vehicle.blocker_id
                 << " wait=" << vehicle.wait_time;
            coordLog(line.str());
        }
        const auto resources = rule_engine_->conflictResourceMarkers(agents_);
        for (const auto& marker : resources) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[TIMELINE_ZONE] tick=" << tick_count_
                 << " sim_t=" << sim_time_
                 << " kind="
                 << (marker.kind == forklift_planner::multi_vehicle::
                         ConflictMarkerKind::CONFLICT_RESERVATION
                         ? "RESERVED" : "POTENTIAL")
                 << " pair=V" << marker.vehicle_a << "/V" << marker.vehicle_b
                 << " raw=" << marker.raw_zone_index
                 << " active=" << marker.active_zone_index
                 << " V" << marker.vehicle_a << "=[" << marker.s_a_enter
                 << "," << marker.s_a_exit << "]"
                 << " V" << marker.vehicle_b << "=[" << marker.s_b_enter
                 << "," << marker.s_b_exit << "]"
                 << " holder=" << marker.holder_id
                 << " waiter=" << marker.waiter_id;
            coordLog(line.str());
        }
        for (const auto& marker : rule_engine_->conflicts()) {
            if (marker.kind != forklift_planner::multi_vehicle::
                                   ConflictMarkerKind::CROSSING_OR_OPPOSING) {
                continue;
            }
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[TIMELINE_EVENT] tick=" << tick_count_
                 << " sim_t=" << sim_time_
                 << " pair=V" << marker.vehicle_a << "/V" << marker.vehicle_b
                 << " raw=" << marker.raw_zone_index
                 << " active=" << marker.active_zone_index
                 << " first_t=" << marker.t
                 << " overlaps=" << marker.timed_overlaps.size()
                 << " holder=" << marker.holder_id
                 << " waiter=" << marker.waiter_id;
            if (!marker.timed_overlaps.empty()) {
                line << " overlap_t=[" << marker.timed_overlaps.front().t
                     << "," << marker.timed_overlaps.back().t << "]";
            }
            coordLog(line.str());
        }
    }

    void requestDebugSnapshot(const std::string& event) {
        if (!snapshot_debug_enabled_ ||
            snapshot_last_trigger_tick_ == tick_count_) {
            return;
        }
        std_msgs::String message;
        std::ostringstream text;
        text << std::fixed << std::setprecision(1)
             << "event=" << event
             << " seed=" << cfg_.random_seed
             << " vehicle_count=" << cfg_.vehicle_count
             << " sim_t=" << sim_time_
             << " tick=" << tick_count_;
        message.data = text.str();
        snapshot_trigger_pub_.publish(message);
        snapshot_last_trigger_tick_ = tick_count_;
        ROS_WARN("[RVIZ-SNAPSHOT] trigger requested: %s", message.data.c_str());
    }

    void updateSnapshotWedgeTrigger() {
        if (!snapshot_debug_enabled_) return;
        double max_wait = 0.0;
        for (const VehicleAgent& vehicle : agents_) {
            max_wait = std::max(max_wait, vehicle.wait_time);
        }
        if (max_wait <= 1.0) {
            snapshot_wedge_active_ = false;
        } else if (max_wait > 25.0 && !snapshot_wedge_active_) {
            snapshot_wedge_active_ = true;
            requestDebugSnapshot("FIRST-WEDGE");
        }
    }

    // 一辆车的紧凑状态行(诊断用,信息尽量全)。
    std::string vehLine(const VehicleAgent& v) const {
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "  V%d mode=%d act=%d reason=%s blk=%d task=%d slot=%d->%d "
                 "s=%.3f/%.3f rem=%.3f spd=%.3f wait=%.1f gen=%d",
                 v.id, (int)v.mode, (int)v.action, v.reason.c_str(), v.blocker_id,
                 v.task_count, v.current_slot,
                 v.target_slot, v.path_s, v.track.length(), v.remainingS(),
                 v.current_speed, v.wait_time, v.path_gen);
        return buf;
    }
    std::string fleetSnapshot() const {
        std::string s = "tick=" + std::to_string(tick_count_);
        for (const VehicleAgent& v : agents_) s += "\n" + vehLine(v);
        return s;
    }
    VehicleAgent* agentById(int id) {
        for (VehicleAgent& v : agents_)
            if (v.id == id) return &v;
        return nullptr;
    }
    // 打印某对车的冲突几何(委托 RuleEngine,拿到 se/sx/same_dir/committed/owner/following)。
    void dumpPair(int ia, int ib) {
        VehicleAgent* a = agentById(ia);
        VehicleAgent* b = agentById(ib);
        if (a && b) rule_engine_->debugDumpConflict(*a, *b);
    }

    // 死锁看门狗(C-第1步):跟 blocker_id 等待图找一个「持续死锁环」——环内每辆都
    // ACTIVE+STOP 且 wait_time≥min_wait(确为持续、非瞬时)。返回环成员 id(按链序),无则空。
    // 死锁恢复(C):检测「所有」持续环的成员 → 选其中等待最久(最该救)且不在冷却期的车,
    // 从当前位姿重规划到空库位脱困。不倒车、不强推。冷却防止反复重规划同一辆(churn)。
    const VehicleAgent* agentById_c(int id) const {
        for (const VehicleAgent& v : agents_) if (v.id == id) return &v;
        return nullptr;
    }

    // 一次性全面 dump 首个持续死锁簇:每个成员的路径要点 + 簇内两两冲突几何(same_dir 决定
    // 对向/同向 → 判定单向环流能否治)。只打一次,只读。用于源头修复的精确诊断。
    // 持久 onset 文件:把关键现场同时写到统一的 debug_log_dir。
    // 长测排错专用——只在出问题那一刻写,故文件小、不刷屏。
    void onsetLog(const std::string& s) {
        const std::string console_line = contextualLog(
            s, "REAL", coord_log_plan_id_, coord_log_frame_id_, -1);
        ROS_ERROR("%s", console_line.c_str());
        coordLog(s);
        std::ofstream f(onset_log_file_, std::ios::app);
        if (f) f << s << "\n";
    }
    // 把碰撞/楔死前的全队历史(含 gen=path_gen:刚被 recovery 重规划过则 gen 跳变=churn 撞)
    // 写进持久文件,供事后根因。
    void onsetDumpHist(const std::string& header, const std::deque<std::string>& hist) {
        std::ofstream f(onset_log_file_, std::ios::app);
        if (!f) return;
        f << "\n========== " << header << " ==========\n";
        coordLog("========== " + header + " ==========");
        for (const std::string& snap : hist) f << snap << "\n";
        for (const std::string& snap : hist) coordLog(snap);
        f.flush();
    }

    std::string stressSnapshot(bool include_geometry) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(3)
            << "tick=" << tick_count_ << " sim_t=" << sim_time_;
        for (const VehicleAgent& v : agents_) {
            out << "\n  V" << v.id
                << " mode=" << modeName(v.mode)
                << " phase=" << missionPhaseName(v.mission_phase)
                << " gen=" << v.path_gen
                << " task=" << v.task_count
                << " slot=" << v.current_slot << "->" << v.target_slot
                << " s=" << v.path_s
                << " speed=" << v.current_speed
                << " action=" << actionName(v.action)
                << " requested=" << actionName(v.requested_action)
                << " blocker=" << v.blocker_id
                << " wait=" << v.wait_time
                << " reason=" << v.reason;
        }

        const auto state = rule_engine_->snapshot();
        for (const auto& item : state.reservations) {
            const auto& key = item.first;
            const auto& r = item.second;
            out << "\n  reservation=V" << key.first << "/V" << key.second
                << " owner=V" << r.owner_id
                << " gen=" << r.gen_lo << "/" << r.gen_hi
                << " lo=[" << r.enter_lo << "," << r.exit_lo << "]"
                << " hi=[" << r.enter_hi << "," << r.exit_hi << "]"
                << " raw=" << r.raw_zone_index;
        }
        for (const auto& item : state.a1.departure_clusters) {
            const auto& c = item.second;
            out << "\n  departure_cluster=V" << item.first.first << "/V"
                << item.first.second << " owner=V" << c.owner_id
                << " owner_gen=" << c.owner_path_gen
                << " other=V" << c.other_id
                << " other_gen=" << c.other_path_gen
                << " active=" << (c.active ? 1 : 0)
                << " intervals=" << c.intervals.size()
                << " stop_boundary=" << c.waiter_stop_boundary_s
                << " stop_s=" << c.waiter_stop_s
                << " release=" << c.owner_release_exit_s << "/"
                << c.other_release_exit_s;
        }
        const auto& future = rule_engine_->futureA1Commitment();
        out << "\n  future_a1=";
        if (future.valid()) {
            out << "owner=V" << future.owner_id
                << " gen=" << future.owner_path_gen
                << " arrival=" << future.predicted_a1_arrival_time
                << " to_b=" << future.predicted_to_b_time;
        } else {
            out << "none";
        }

        if (include_geometry) {
            const auto markers = rule_engine_->conflictResourceMarkers(agents_);
            for (const auto& m : markers) {
                out << "\n  zone="
                    << (m.kind == forklift_planner::multi_vehicle::
                            ConflictMarkerKind::CONFLICT_RESERVATION
                            ? "RESERVED" : "POTENTIAL")
                    << " pair=V" << m.vehicle_a << "/V" << m.vehicle_b
                    << " raw=" << m.raw_zone_index
                    << " active=" << m.active_zone_index
                    << " a=[" << m.s_a_enter << "," << m.s_a_exit << "]"
                    << " b=[" << m.s_b_enter << "," << m.s_b_exit << "]"
                    << " holder=" << m.holder_id
                    << " waiter=" << m.waiter_id;
            }
        }
        return out.str();
    }

    void ensureParentDirectory(const std::string& file) const {
        if (file.empty()) return;
        const std::filesystem::path parent =
            std::filesystem::path(file).parent_path();
        if (parent.empty()) return;
        std::error_code error;
        std::filesystem::create_directories(parent, error);
        if (error) {
            ROS_ERROR("[stress] cannot create result directory %s: %s",
                      parent.string().c_str(), error.message().c_str());
        }
    }

    void writeStressResult(const std::string& status,
                           const std::string& failure_type,
                           const std::vector<double>& max_wait,
                           unsigned long long wedge_episodes) const {
        if (stress_result_file_.empty()) return;
        ensureParentDirectory(stress_result_file_);
        std::ofstream out(stress_result_file_, std::ios::trunc);
        if (!out) {
            ROS_ERROR("[stress] cannot write result file %s",
                      stress_result_file_.c_str());
            return;
        }
        out << "status=" << status << "\n"
            << "failure_type=" << (failure_type.empty() ? "none" : failure_type)
            << "\nseed=" << cfg_.random_seed
            << "\nvehicle_count=" << agents_.size()
            << "\nticks=" << tick_count_
            << "\nsim_time_s=" << std::fixed << std::setprecision(3) << sim_time_
            << "\nhard_guard_events=" << hard_guard_events_
            << "\nwedge_episodes=" << wedge_episodes << "\n";
        for (size_t i = 0; i < agents_.size(); ++i) {
            out << "V" << agents_[i].id << "_tasks=" << agents_[i].task_count
                << "\nV" << agents_[i].id << "_max_wait_s="
                << (i < max_wait.size() ? max_wait[i] : 0.0) << "\n";
        }
    }

    void writeStressFailure(const std::string& failure_type,
                            const std::deque<std::string>& ring) const {
        if (stress_failure_file_.empty()) return;
        ensureParentDirectory(stress_failure_file_);
        std::ofstream out(stress_failure_file_, std::ios::trunc);
        if (!out) {
            ROS_ERROR("[stress] cannot write failure file %s",
                      stress_failure_file_.c_str());
            return;
        }
        out << "failure_type=" << failure_type
            << "\nseed=" << cfg_.random_seed
            << "\nring_seconds=120"
            << "\nprogress_timeout_s=" << stress_progress_timeout_
            << "\n\n===== PRE-FAILURE RING =====\n";
        for (const std::string& frame : ring) out << frame << "\n---\n";
        out << "===== FAILURE GEOMETRY =====\n"
            << stressSnapshot(true) << "\n";
    }

    // 找「所有」持续死锁环的成员并集:对每辆车跟 blocker 链,若绕回自身则其环成员全部入集。
    // 批处理模式:紧凑循环跑 ticks 拍(跳过 marker)。维护近 N 拍环形历史;首次碰撞那拍
    // dump 全队历史+碰撞对几何,并对其后 kPost 拍逐拍详打;结尾 dump 永久楔死现场。
    bool runBatch(unsigned long long ticks) {
        const double dt = 1.0 / pp_.update_rate;
        if (!stress_watchdog_enabled_) {
            std::ofstream(onset_log_file_, std::ios::trunc);
        }
        const unsigned long long progress = ticks / 10 ? ticks / 10 : 1;
        constexpr size_t kHist = 80;    // 碰撞前回看的拍数
        constexpr unsigned long long kPost = 150;  // 碰撞后逐拍详打的拍数
        std::deque<std::string> hist;
        bool first_dumped = false;
        bool wedge_dumped = false;
        unsigned long long verbose_until = 0;
        const size_t stress_ring_limit = std::max<size_t>(
            1, static_cast<size_t>(std::ceil(120.0 * pp_.update_rate)));
        std::deque<std::string> stress_ring;
        std::vector<double> max_wait_by_vehicle(agents_.size(), 0.0);
        std::vector<double> last_progress_time(agents_.size(), sim_time_);
        std::vector<double> previous_path_s(agents_.size(), 0.0);
        std::vector<int> previous_path_gen(agents_.size(), -1);
        std::vector<int> previous_task_count(agents_.size(), -1);
        std::vector<MissionPhase> previous_phase(agents_.size(),
                                                 MissionPhase::DIRECT_TO_B);
        std::vector<std::array<double, 5>> action_seconds(agents_.size());
        std::vector<unsigned long long> action_transitions(agents_.size(), 0);
        std::vector<VehicleAction> previous_action(agents_.size(),
                                                   VehicleAction::STOP);
        std::vector<bool> previous_action_valid(agents_.size(), false);
        for (size_t i = 0; i < agents_.size(); ++i) {
            previous_path_s[i] = agents_[i].path_s;
            previous_path_gen[i] = agents_[i].path_gen;
            previous_task_count[i] = agents_[i].task_count;
            previous_phase[i] = agents_[i].mission_phase;
        }
        bool wedge_active = false;
        unsigned long long wedge_episodes = 0;
        unsigned long long completed_ticks = 0;

        auto fail_stress = [&](const std::string& failure_type) {
            writeStressFailure(failure_type, stress_ring);
            writeStressResult("FAIL", failure_type, max_wait_by_vehicle,
                              wedge_episodes);
            ROS_ERROR("[stress] FAIL seed=%d type=%s tick=%llu sim_t=%.3f",
                      cfg_.random_seed, failure_type.c_str(), tick_count_,
                      sim_time_);
            return true;
        };

        for (unsigned long long k = 0; k < ticks && ros::ok(); ++k) {
            ++tick_count_;
            sim_time_ += dt;
            updateDwellAndTasks(dt);
            if (simulationPlanNeedsRefresh()) {
                std::vector<sandbox_msgs::Trajectory> trajs;
                std::vector<bool> hold;
                buildSimulationHorizonPlan(trajs, hold);
                force_horizon_refresh_ = false;
            }
            if (!executeSimulationPlanSample()) {
                std::vector<sandbox_msgs::Trajectory> trajs;
                std::vector<bool> hold;
                buildSimulationHorizonPlan(trajs, hold);
                force_horizon_refresh_ = false;
                if (!executeSimulationPlanSample()) {
                    rule_engine_->decide(agents_, dt);
                    marker_pub_->setRollingDecision(
                        rule_engine_->lastRollingDynamicDecision());
                }
            }
            for (size_t i = 0; i < agents_.size(); ++i) {
                const VehicleAgent& v = agents_[i];
                if (v.mode != VehicleMode::ACTIVE) {
                    previous_action_valid[i] = false;
                    continue;
                }
                const size_t action_index = static_cast<size_t>(v.action);
                if (action_index < action_seconds[i].size()) {
                    action_seconds[i][action_index] += dt;
                }
                if (previous_action_valid[i] &&
                    previous_action[i] != v.action) {
                    ++action_transitions[i];
                }
                previous_action[i] = v.action;
                previous_action_valid[i] = true;
            }
            const unsigned long long guards_before = hard_guard_events_;
            advanceVehicles(dt);
            diagnoseA1ExitIntrusions();
            recordDebugTimelineTick();
            const bool new_collision = hard_guard_events_ > guards_before;
            ++completed_ticks;

            bool path_generation_changed = false;
            double current_max_wait = 0.0;
            for (size_t i = 0; i < agents_.size(); ++i) {
                const VehicleAgent& v = agents_[i];
                max_wait_by_vehicle[i] =
                    std::max(max_wait_by_vehicle[i], v.wait_time);
                current_max_wait = std::max(current_max_wait, v.wait_time);
                const bool progressed =
                    std::abs(v.path_s - previous_path_s[i]) > 1e-4 ||
                    v.path_gen != previous_path_gen[i] ||
                    v.task_count != previous_task_count[i] ||
                    v.mission_phase != previous_phase[i];
                path_generation_changed = path_generation_changed ||
                    v.path_gen != previous_path_gen[i];
                if (progressed || v.mode != VehicleMode::ACTIVE) {
                    last_progress_time[i] = sim_time_;
                }
                previous_path_s[i] = v.path_s;
                previous_path_gen[i] = v.path_gen;
                previous_task_count[i] = v.task_count;
                previous_phase[i] = v.mission_phase;
            }
            if (current_max_wait > 25.0 && !wedge_active) {
                wedge_active = true;
                ++wedge_episodes;
            } else if (current_max_wait <= 1e-9) {
                wedge_active = false;
            }

            if (stress_watchdog_enabled_) {
                stress_ring.push_back(stressSnapshot(path_generation_changed));
                if (stress_ring.size() > stress_ring_limit) stress_ring.pop_front();

                if (new_collision) return fail_stress("HARD_GUARD");

                for (size_t i = 0; i < agents_.size(); ++i) {
                    if (agents_[i].mode == VehicleMode::ACTIVE &&
                        sim_time_ - last_progress_time[i] >=
                            stress_progress_timeout_) {
                        return fail_stress("NO_PROGRESS_V" +
                                           std::to_string(agents_[i].id));
                    }
                }
            }

            if (!stress_quiet_) {
                hist.push_back(fleetSnapshot());
                if (hist.size() > kHist) hist.pop_front();
            }

            if (!stress_quiet_ && new_collision && !first_dumped) {
                first_dumped = true;
                verbose_until = tick_count_ + kPost;
                std::string cp;
                for (const auto& q : hard_guard_pairs_)
                    cp += "V" + std::to_string(q.first) + "-V" +
                          std::to_string(q.second) + " ";
                onsetLog("[FIRST-COLLISION] @tick=" + std::to_string(tick_count_) +
                         " sim_t=" + std::to_string((long long)sim_time_) +
                         "s 涉及对=[" + cp + "]");
                onsetDumpHist("FIRST-COLLISION 前 " + std::to_string(hist.size()) +
                              " 拍历史(看 gen 是否刚跳变=刚被脱困重置)", hist);
                ROS_WARN("[HIST] ====== 碰撞前 %zu 拍全队历史 ======", hist.size());
                for (const std::string& snap : hist) ROS_WARN("[HIST]\n%s", snap.c_str());
                ROS_WARN("[HIST] ====== 碰撞对冲突几何 ======");
                for (const auto& q : hard_guard_pairs_) dumpPair(q.first, q.second);
            }

            // 楔死现场一次性诊断:任一车 wait 首次超阈值 → 回放历史 + 全队 + 卡死车几何。
            if (!stress_quiet_ && !wedge_dumped) {
                int sid = -1; double mw = 0.0;
                for (const VehicleAgent& v : agents_)
                    if (v.wait_time > mw) { mw = v.wait_time; sid = v.id; }
                if (mw > 25.0 && sid >= 0) {
                    wedge_dumped = true;
                    ROS_ERROR("[FIRST-WEDGE] @tick=%llu sim_t=%.1fs 最久=V%d wait=%.1fs"
                              " —— 回放前 %zu 拍历史 + 卡死车几何 ===",
                              tick_count_, sim_time_, sid, mw, hist.size());
                    for (const std::string& snap : hist)
                        ROS_WARN("[WHIST]\n%s", snap.c_str());
                    for (const VehicleAgent& v : agents_)
                        if (v.id != sid) dumpPair(sid, v.id);
                }
            }

            if (!stress_quiet_ && tick_count_ <= verbose_until) {
                ROS_WARN("[POST]\n%s", fleetSnapshot().c_str());
                for (const auto& q : hard_guard_pairs_) dumpPair(q.first, q.second);
            }

            updateSnapshotWedgeTrigger();

            if (!stress_quiet_ && (k + 1) % progress == 0) {
                ROS_INFO("[batch] %llu/%llu ticks (sim_t=%.0fs) hard_guard=%llu",
                         k + 1, ticks, sim_time_, hard_guard_events_);
            }
        }

        if (stress_watchdog_enabled_ && completed_ticks != ticks) {
            return fail_stress("INFRASTRUCTURE_STOP");
        }

        if (stress_watchdog_enabled_) {
            writeStressResult("PASS", "", max_wait_by_vehicle,
                              wedge_episodes);
            ROS_WARN("[stress] PASS seed=%d ticks=%llu sim_t=%.3f",
                     cfg_.random_seed, tick_count_, sim_time_);
            return false;
        }

        // 结尾:找等待最久(永久楔死)的车,dump 它与所有其它车的冲突几何 + 全队快照。
        double max_wait = 0.0;
        int stuck_id = -1;
        for (const VehicleAgent& v : agents_)
            if (v.wait_time > max_wait) { max_wait = v.wait_time; stuck_id = v.id; }
        ROS_WARN("[END] ====== 结尾全队快照 ======\n%s", fleetSnapshot().c_str());
        if (stuck_id >= 0 && max_wait > 20.0) {
            ROS_WARN("[END] 永久楔死嫌疑=V%d (wait=%.1fs),其与各车冲突几何:",
                     stuck_id, max_wait);
            for (const VehicleAgent& v : agents_)
                if (v.id != stuck_id) dumpPair(stuck_id, v.id);
        }
        std::string pairs;
        for (const auto& p : hard_guard_pairs_)
            pairs += "V" + std::to_string(p.first) + "-V" +
                     std::to_string(p.second) + " ";
        ROS_WARN("[batch] ==== 汇总: ticks=%llu sim_t=%.0fs | 碰撞(hard_guard)事件=%llu "
                 "首次@tick=%llu 涉及对=[%s] | 最大wait=%.1fs(V%d) ====",
                 tick_count_, sim_time_, hard_guard_events_, first_guard_tick_,
                 pairs.c_str(), max_wait, stuck_id);
        for (size_t i = 0; i < agents_.size(); ++i) {
            const double active_seconds = std::accumulate(
                action_seconds[i].begin(), action_seconds[i].end(), 0.0);
            const double nominal_ratio = active_seconds > 1e-9
                ? action_seconds[i][static_cast<size_t>(
                      VehicleAction::NOMINAL)] / active_seconds
                : 0.0;
            ROS_WARN("[BATCH_ACTION_METRICS] V%d tasks=%d max_wait=%.1f "
                     "active_s=%.1f STOP=%.1f CREEP=%.1f YIELD=%.1f "
                     "NOMINAL=%.1f BOOST=%.1f nominal_ratio=%.6f "
                     "transitions=%llu",
                     agents_[i].id, agents_[i].task_count,
                     max_wait_by_vehicle[i],
                     active_seconds,
                     action_seconds[i][static_cast<size_t>(
                         VehicleAction::STOP)],
                     action_seconds[i][static_cast<size_t>(
                         VehicleAction::CREEP)],
                     action_seconds[i][static_cast<size_t>(
                         VehicleAction::YIELD)],
                     action_seconds[i][static_cast<size_t>(
                         VehicleAction::NOMINAL)],
                     action_seconds[i][static_cast<size_t>(
                         VehicleAction::BOOST)],
                     nominal_ratio, action_transitions[i]);
        }
        const auto& dynamic = rule_engine_->dynamicSpeedMetrics();
        const double bridge_average_backtrack =
            dynamic.bridge_checked_pairs == 0
                ? 0.0
                : static_cast<double>(dynamic.bridge_backtrack_samples) /
                      (2.0 * dynamic.bridge_checked_pairs);
        ROS_WARN("[BATCH_DYN_SPEED_EVALUATED] baseline_conflicts=%llu "
                 "crossing=%llu opposing=%llu same_direction=%llu "
                 "bridge_checked=%llu bridge_related_a=%llu "
                 "bridge_related_b=%llu bridge_corrected_pairs=%llu "
                 "bridge_backtrack_avg=%.2f bridge_backtrack_max=%llu "
                 "bridge_nearest_evaluations=%llu "
                 "far=%llu mid=%llu near=%llu emergency_stop=%llu "
                 "yield_eval=%llu yield_clear=%llu yield_delayed=%llu "
                 "creep_eval=%llu creep_clear=%llu creep_delayed=%llu "
                 "selected_conflict_remaining=%llu a1_fallback=%llu "
                 "existing_reservation=%llu nominal_recovery=%llu "
                 "reservation_create=%llu reservation_update=%llu "
                 "reservation_delete=%llu ordinary_create=%llu "
                 "a1_create=%llu terminal_create=%llu inside_create=%llu "
                 "braking_create=%llu "
                 "multi_vehicle_create=%llu other_create=%llu "
                 "duplicate_pair_authority=%llu",
                 dynamic.baseline_conflicts, dynamic.crossing_conflicts,
                 dynamic.opposing_conflicts,
                 dynamic.same_direction_conflicts,
                 dynamic.bridge_checked_pairs,
                 dynamic.bridge_related_a,
                 dynamic.bridge_related_b,
                 dynamic.bridge_corrected_pairs,
                 bridge_average_backtrack,
                 dynamic.bridge_max_backtrack_samples,
                 dynamic.bridge_nearest_evaluations,
                 dynamic.far_decisions, dynamic.mid_decisions,
                 dynamic.near_decisions,
                 dynamic.emergency_stop_decisions,
                 dynamic.yield_evaluations,
                 dynamic.yield_conflict_free, dynamic.yield_delayed,
                 dynamic.creep_evaluations,
                 dynamic.creep_conflict_free, dynamic.creep_delayed,
                 dynamic.selected_conflict_remaining,
                 dynamic.a1_fallbacks,
                 dynamic.existing_reservation_skips,
                 dynamic.nominal_recoveries,
                 dynamic.reservation_creates,
                 dynamic.reservation_updates,
                 dynamic.reservation_deletes,
                 dynamic.reservation_create_ordinary_dynamic,
                 dynamic.reservation_create_a1,
                 dynamic.reservation_create_terminal,
                 dynamic.reservation_create_already_inside,
                 dynamic.reservation_create_braking_safety,
                 dynamic.reservation_create_multi_vehicle,
                 dynamic.reservation_create_other,
                 dynamic.duplicate_pair_authority_overrides);
        const auto& executed = executed_rolling_metrics_;
        ROS_WARN("[BATCH_DYN_SPEED_EXECUTED] far_periods=%llu "
                 "mid_periods=%llu near_periods=%llu legacy_periods=%llu "
                 "far_to_nominal=%llu mid_to_yield=%llu "
                 "near_to_creep=%llu nominal_to_creep=%llu "
                 "nominal_to_stop=%llu yield_to_stop=%llu "
                 "creep_to_stop=%llu selected_rollout_clear=%llu "
                 "yield_delayed=%llu creep_delayed=%llu "
                 "eventually_resolved=%llu "
                 "target_STOP=%llu target_CREEP=%llu target_YIELD=%llu "
                 "target_NOMINAL=%llu target_BOOST=%llu "
                 "reservation_create=%llu reservation_update=%llu "
                 "reservation_delete=%llu existing_reservation=%llu "
                 "ordinary_create=%llu a1_create=%llu terminal_create=%llu "
                 "inside_create=%llu braking_create=%llu "
                 "multi_vehicle_create=%llu "
                 "other_create=%llu",
                 executed.far_periods, executed.mid_periods,
                 executed.near_periods, executed.legacy_periods,
                 executed.far_to_nominal, executed.mid_to_yield,
                 executed.near_to_creep, executed.nominal_to_creep,
                 executed.emergency_stop_from[static_cast<size_t>(
                     VehicleAction::NOMINAL)],
                 executed.emergency_stop_from[static_cast<size_t>(
                     VehicleAction::YIELD)],
                 executed.emergency_stop_from[static_cast<size_t>(
                     VehicleAction::CREEP)],
                 executed.selected_rollout_clear,
                 executed.yield_delayed, executed.creep_delayed,
                 executed.eventually_resolved,
                 executed.target_actions[static_cast<size_t>(
                     VehicleAction::STOP)],
                 executed.target_actions[static_cast<size_t>(
                     VehicleAction::CREEP)],
                 executed.target_actions[static_cast<size_t>(
                     VehicleAction::YIELD)],
                 executed.target_actions[static_cast<size_t>(
                     VehicleAction::NOMINAL)],
                 executed.target_actions[static_cast<size_t>(
                     VehicleAction::BOOST)],
                 executed.reservation_creates,
                 executed.reservation_updates,
                 executed.reservation_deletes,
                 executed.existing_reservation_holds,
                 executed.reservation_create_ordinary_dynamic,
                 executed.reservation_create_a1,
                 executed.reservation_create_terminal,
                 executed.reservation_create_already_inside,
                 executed.reservation_create_braking_safety,
                 executed.reservation_create_multi_vehicle,
                 executed.reservation_create_other);
        const auto& a1_service_metrics = rule_engine_->a1ServiceMetrics();
        double max_service_duration = a1_service_metrics.max_duration;
        if (a1_service_metrics.active_since >= 0.0) {
            max_service_duration = std::max(
                max_service_duration,
                sim_time_ - a1_service_metrics.active_since);
        }
        double max_launch_hold = a1_launch_metrics_.max_hold_duration;
        for (const auto& entry : a1_launch_holds_) {
            max_launch_hold = std::max(
                max_launch_hold, sim_time_ - entry.second.since);
        }
        ROS_WARN("[BATCH_A1_SERVICE] create=%llu hold=%llu change=%llu "
                 "release=%llu invalidate=%llu arrival_preemptions=%llu "
                 "faster_candidate_observed=%llu max_duration=%.1f",
                 a1_service_metrics.creates,
                 a1_service_metrics.holds,
                 a1_service_metrics.changes,
                 a1_service_metrics.releases,
                 a1_service_metrics.invalidates,
                 a1_service_metrics.arrival_ranking_preemptions,
                 a1_service_metrics.faster_candidate_observations,
                 max_service_duration);
        ROS_WARN("[BATCH_A1_LAUNCH] allow=%llu hold=%llu "
                 "a1_prefix_hold=%llu ordinary_road_hold=%llu "
                 "retries=%llu released_after_hold=%llu max_hold=%.1f "
                 "active_holds=%zu",
                 a1_launch_metrics_.allows,
                 a1_launch_metrics_.holds,
                 a1_launch_metrics_.a1_prefix_holds,
                 a1_launch_metrics_.ordinary_road_holds,
                 a1_launch_metrics_.retries,
                 a1_launch_metrics_.released_after_hold,
                 max_launch_hold, a1_launch_holds_.size());
        ROS_WARN("[BATCH_RUNTIME] requested_ticks=%llu completed_ticks=%llu "
                 "real_sim_t=%.1f dt=%.3f wedge_episodes=%llu",
                 ticks, completed_ticks, sim_time_, dt, wedge_episodes);
        return hard_guard_events_ > 0;
    }
    bool batchMode() const { return cfg_batch_ticks_ > 0; }
    unsigned long long batchTicks() const { return cfg_batch_ticks_; }

private:
    unsigned long long cfg_batch_ticks_ = 0;
};

int main(int argc, char** argv) {
    // 用环境 locale(通常 UTF-8)初始化 C/C++ 本地化,否则默认 "C" locale 会把日志里的
    // 中文打成 ???。一句即可,根治 rosconsole/printf 中文乱码。
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "multi_vehicle_patrol_node");
    MultiVehiclePatrolNode node;

    if (node.batchMode()) {
        // 无头快速回归:狂跑后退出。返回码 1=出现碰撞,0=干净(便于脚本判定)。
        const bool collided = node.runBatch(node.batchTicks());
        ros::shutdown();
        return collided ? 1 : 0;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
