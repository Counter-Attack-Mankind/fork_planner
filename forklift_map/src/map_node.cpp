#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_map/forklift_map.h"
#include "forklift_map/common/clothoid.h"  // 与规划器共用的曲率连续拐弯

// ─── color helpers ────────────────────────────────────────────────────────────
static std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c;
}
static std_msgs::ColorRGBA hex(uint32_t rgb, float a = 1.0f) {
    return rgba(((rgb>>16)&0xFF)/255.f, ((rgb>>8)&0xFF)/255.f, (rgb&0xFF)/255.f, a);
}


namespace Color {
    static auto kFloor         = hex(0x141414);          // 地面     深黑（防反光、耐脏）
    static auto kShelfSingle   = hex(0x9E9E9E);          // 顶/底单排货架 中灰（防反光）
    static auto kShelfDouble   = hex(0x9E9E9E);          // 行1/2/3双排   中灰
    static auto kSlot          = hex(0x9CCC65);          // 货位填充      浅绿
    static auto kSlotEdge      = hex(0x558B2F);          // 货位轮廓      深绿描边（灰底上可见）
    static auto kBoundary      = hex(0xBFBFBF);          // 边界线        浅灰（黑底上可见）
    static auto kLane          = hex(0xFFD400);          // 车道中心线    黄（黑底黄线）
    static auto kSlotPoseArrow = hex(0x00E5FF);          // 货位朝向箭头  青色
    static auto kSlotPoseText  = hex(0xFFFFFF);          // 货位位姿文字  白色
}

// ─── geometry_msgs::Point helper ─────────────────────────────────────────────
static geometry_msgs::Point pt(double x, double y, double z = 0.0) {
    geometry_msgs::Point p; p.x=x; p.y=y; p.z=z; return p;
}

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kSlotGapHalf = 0.012;  // 双排货位之间的黑缝半宽（隔板厚度）
static constexpr double kBoundaryLineWidth = 0.015;
static constexpr double kContentInset = kBoundaryLineWidth;

struct Rect2D {
    double x = 0.0, y = 0.0, w = 0.0, h = 0.0;
};

static double clampToContent(double v, double max_v) {
    return std::min(std::max(v, kContentInset), max_v - kContentInset);
}

static geometry_msgs::Point clippedPt(const MapParam& p, double x, double y, double z) {
    return pt(clampToContent(x, p.field_width),
              clampToContent(y, p.field_height),
              z);
}

static bool clipRectToContent(const MapParam& p,
                              double x, double y, double w, double h,
                              Rect2D& out) {
    double x0 = clampToContent(x, p.field_width);
    double y0 = clampToContent(y, p.field_height);
    double x1 = clampToContent(x + w, p.field_width);
    double y1 = clampToContent(y + h, p.field_height);
    if (x1 <= x0 || y1 <= y0) return false;
    out.x = x0;
    out.y = y0;
    out.w = x1 - x0;
    out.h = y1 - y0;
    return true;
}

// ─── marker factory helpers ───────────────────────────────────────────────────
static visualization_msgs::Marker baseMarker(const std::string& frame,
                                             const std::string& ns, int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = ros::Time(0);
    m.ns = ns; m.id = id;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
}

// CUBE (shelf block / floor)
static void addCube(visualization_msgs::MarkerArray& arr,
                    const std::string& frame, const std::string& ns, int id,
                    double cx, double cy, double cz,
                    double sx, double sy, double sz,
                    const std_msgs::ColorRGBA& col) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::CUBE;
    m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = cz;
    m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
    m.color = col;
    arr.markers.push_back(m);
}

// LINE_STRIP (closed rectangle, slot / shelf divider / boundary)
static void addLineStrip(visualization_msgs::MarkerArray& arr,
                         const std::string& frame, const std::string& ns, int id,
                         const std::vector<geometry_msgs::Point>& pts,
                         double width, const std_msgs::ColorRGBA& col) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = width;
    m.color = col;
    m.points = pts;
    arr.markers.push_back(m);
}

// Camera-facing text (slot number)
// ARROW from slot origin to dock heading direction.
static void addArrow(visualization_msgs::MarkerArray& arr,
                     const std::string& frame, const std::string& ns, int id,
                     double x0, double y0, double z,
                     double theta, double length,
                     double shaft_diameter, double head_diameter,
                     const std_msgs::ColorRGBA& col) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::ARROW;
    m.points.push_back(pt(x0, y0, z));
    m.points.push_back(pt(x0 + length * std::cos(theta),
                          y0 + length * std::sin(theta), z));
    m.scale.x = shaft_diameter;
    m.scale.y = head_diameter;
    m.scale.z = head_diameter * 1.4;
    m.color = col;
    arr.markers.push_back(m);
}
static void addText(visualization_msgs::MarkerArray& arr,
                    const std::string& frame, const std::string& ns, int id,
                    double x, double y, double z, double height,
                    const std::string& text,
                    const std_msgs::ColorRGBA& col) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.z = height;
    m.text = text;
    m.color = col;
    arr.markers.push_back(m);
}

static void addClippedLineStrip(visualization_msgs::MarkerArray& arr,
                                const MapParam& p,
                                const std::string& frame,
                                const std::string& ns, int id,
                                const std::vector<geometry_msgs::Point>& pts,
                                double width, const std_msgs::ColorRGBA& col) {
    std::vector<geometry_msgs::Point> clipped;
    clipped.reserve(pts.size());
    for (const auto& point : pts) {
        clipped.push_back(clippedPt(p, point.x, point.y, point.z));
    }
    addLineStrip(arr, frame, ns, id, clipped, width, col);
}

// 共性拐弯：曲率连续的回旋曲线（clothoid），与规划器同一份算法、同一套参数。
// 角点 (corner_x, corner_y)；走廊沿 x（行进来向 din_sign=±1），通道沿 y（channel_dir=±1）。
// 在场地范围内才绘制（越界方向跳过——靠墙拐不出去）。
struct TurnParam { double max_curvature; double ramp_len; double ds; };

// 90° 拐弯在给定可用切线长 max_tangent 下、自适应后的实际切线长（与规划器
// fit_clear_turn 同样：空间不够就压缩 ramp）。返回 0 表示该空间放不下拐弯。
static double fittedTangent90(const TurnParam& tp, double max_tangent) {
    const auto c = forklift_geom::fit_clothoid_turn(
        kPi * 0.5, tp.max_curvature, tp.ramp_len, max_tangent, tp.ds);
    if (c.pts.empty()) return 0.0;
    return std::max(c.t_in, c.t_out);
}

static void addClothoidTurn(visualization_msgs::MarkerArray& arr,
                            const MapParam& p,
                            const std::string& frame, const std::string& ns, int id,
                            double corner_x, double corner_y,
                            int din_sign, int channel_dir,
                            const TurnParam& tp, double max_tangent, double z,
                            double width, const std_msgs::ColorRGBA& col) {
    // din=(din_sign,0) → dout=(0,channel_dir)，转角 ±90°
    const double signed_angle =
        (din_sign * channel_dir > 0) ? (kPi * 0.5) : (-kPi * 0.5);
    const auto curve = forklift_geom::fit_clothoid_turn(
        signed_angle, tp.max_curvature, tp.ramp_len, max_tangent, tp.ds);
    if (curve.pts.size() < 2) return;

    // 入向起点沿走廊回退 t_in；靠墙越界则不画
    const double start_x = corner_x - din_sign * curve.t_in;
    if (start_x < kContentInset || start_x > p.field_width - kContentInset) return;

    const double hi = (din_sign > 0) ? 0.0 : kPi;
    const double ch = std::cos(hi), sh = std::sin(hi);
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = width;
    m.color = col;
    for (const auto& q : curve.pts) {
        const double wx = start_x + ch * q.x - sh * q.y;
        const double wy = corner_y + sh * q.x + ch * q.y;
        m.points.push_back(pt(wx, wy, z));
    }
    arr.markers.push_back(m);
}

// ─── Main builder class ───────────────────────────────────────────────────────
class MapNode {
public:
    MapNode() : nh_("~") {
        MapParam param = MapParam::fromROSParam(nh_);
        map_ = std::make_unique<ForkliftMap>(param);
        frame_ = param.frame_id;

        // map_param.yaml is loaded into the global /forklift_map namespace.
        ros::NodeHandle param_nh;
        param_nh.param("forklift_map/show_slot_ids", show_slot_ids_, true);
        param_nh.param("forklift_map/show_slot_poses", show_slot_poses_, true);
        param_nh.param("forklift_map/show_coordinate_axes",show_coordinate_axes_,true);

        // latched publisher - new RViz subscribers get the map immediately
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_map/markers", 1, /*latch=*/true);

        buildMarkers();

        ros::Duration(0.5).sleep();  // give publisher time to connect
        publish();
        republish_timer_ = nh_.createTimer(
            ros::Duration(1.0), &MapNode::republishTimerCb, this);
        ROS_INFO("[forklift_map] Published %zu markers on /forklift_map/markers",
                 arr_.markers.size());
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  pub_;
    ros::Timer republish_timer_;
    std::unique_ptr<ForkliftMap> map_;
    std::string frame_;
    visualization_msgs::MarkerArray arr_;
    int next_id_ = 0;
    bool show_slot_ids_ = true;
    bool show_slot_poses_ = true;
    bool show_coordinate_axes_ = true;

    int newId() { return next_id_++; }

    void buildMarkers() {
        arr_.markers.clear();
        next_id_ = 0;
        addFloor();
        addShelves();
        addSlots();
        addRoads();   // 黄色车道中心线 + 共性拐弯圆弧（无箭头、无末端弧、无尖点）
        addBoundary();

        if (show_coordinate_axes_)
            addCoordinateAxes();
    
    }

    void publish() {
        pub_.publish(arr_);
    }

    void republishTimerCb(const ros::TimerEvent&) {
        publish();
    }

    // ── Floor background ────────────────────────────────────────────────────
    void addFloor() {
        const auto& p = map_->param();
        addCube(arr_, frame_, "floor", newId(),
                p.field_width * 0.5, p.field_height * 0.5, 0.0005,
                p.field_width - 2.0 * kContentInset,
                p.field_height - 2.0 * kContentInset,
                0.001,
                Color::kFloor);
    }

    // ── Shelf blocks ────────────────────────────────────────────────────────
    void addShelves() {
        const auto& p = map_->param();
        for (const auto& s : map_->shelf_blocks()) {
            Rect2D r;
            if (!clipRectToContent(p, s.x, s.y, s.w, s.h, r)) continue;
            auto col = s.is_light_purple ? Color::kShelfSingle : Color::kShelfDouble;
            addCube(arr_, frame_,
                    s.is_light_purple ? "shelf_single" : "shelf_double",
                    newId(),
                    r.x + r.w * 0.5, r.y + r.h * 0.5, 0.003,
                    r.w, r.h, 0.005,
                    col);
        }
    }

    // ── Field boundary ──────────────────────────────────────────────────────
    void addBoundary() {
        const auto& p = map_->param();
        double z = 0.030, lw = kBoundaryLineWidth;
        double W = p.field_width, H = p.field_height;
        double b = kBoundaryLineWidth * 0.5;
        addLineStrip(arr_, frame_, "boundary", newId(),
                     {pt(b,b,z), pt(W-b,b,z), pt(W-b,H-b,z), pt(b,H-b,z), pt(b,b,z)},
                     lw, Color::kBoundary);
    }

    // ---x,y坐标轴显示----------
    void addCoordinateAxes() {
        const auto& p = map_->param();
        constexpr double z = 0.045;
        constexpr double tick = 0.5;

        // 坐标轴超过地图边界的长度
        constexpr double axis_extend = 0.18;
        // 刻度长度
        constexpr double tick_len = 0.06;
        // 数字大小
        constexpr double label_size = 0.085;
        // 坐标轴名称大小
        constexpr double axis_name_size = 0.11;
        // 红色坐标轴
        const auto axis_color = rgba(1.0f, 0.0f, 0.0f, 1.0f);

        //==================== X axis ====================

        // 原点与地图原点(0,0)完全重合
        addLineStrip(
            arr_, frame_, "coord_axis", newId(),
            {
                pt(0.0, 0.0, z),
                pt(p.field_width + axis_extend, 0.0, z)
            },
            0.015, axis_color
        );


        //==================== Y axis ====================

        // 原点与地图原点(0,0)完全重合
        addLineStrip(
            arr_, frame_, "coord_axis", newId(),
            {
                pt(0.0, 0.0, z),
                pt(0.0, p.field_height + axis_extend, z)
            },
            0.015, axis_color
        );

        //==================== X ticks ====================

        for (double x = 0.0;x <= p.field_width + 1e-6;x += tick)
        {
            // X轴刻度向下画
            addLineStrip(
                arr_, frame_, "coord_tick_x", newId(),
                {
                    pt(x, 0.0, z),
                    pt(x, -tick_len, z)
                },
                0.010, axis_color
            );

            std::ostringstream ss;
            ss << std::fixed << std::setprecision(1) << x;

            // X轴数字放在直线下面
            addText(arr_, frame_, "coord_label_x", newId(),x,-tick_len - 0.05,z + 0.02,label_size,ss.str(),axis_color);
        }

        //==================== Y ticks ====================

        for (double y = 0.0;
            y <= p.field_height + 1e-6;
            y += tick)
        {
            // Y轴刻度向左画
            addLineStrip(
                arr_, frame_, "coord_tick_y", newId(),
                {
                    pt(0.0, y, z),
                    pt(-tick_len, y, z)
                },
                0.010, axis_color
            );

            std::ostringstream ss;
            ss << std::fixed << std::setprecision(1) << y;

            // Y轴数字放在直线左边
            addText(arr_, frame_, "coord_label_y", newId(),-tick_len - 0.08,y,z + 0.02,label_size,ss.str(),axis_color);
        }


        //==================== Axis names ====================
        addText(
            arr_, frame_, "coord_axis_name", newId(),
            p.field_width + axis_extend - 0.03,
            -0.04,
            z + 0.03,
            axis_name_size,
            "X",
            axis_color
        );

        // Y
        addText(
            arr_, frame_, "coord_axis_name", newId(),
            -0.04,
            p.field_height + axis_extend - 0.03,
            z + 0.03,
            axis_name_size,
            "Y",
            axis_color
        );
    }
    // 按 row_id 返回该货位所占货架格子的物理 Y 区间 [y0,y1]。
    //   单排（顶/底货架）：整排深度；双排（行1/2/3）：上半/下半排。
    static void shelfCellY(const MapParam& p, int row_id, double& y0, double& y1) {
        const double g = kSlotGapHalf;  // 双排上/下排各从中线缩 g，留出黑缝=隔板
        switch (row_id) {
            case 0: y0 = p.y8();                  y1 = p.field_height;          break; // 顶货架(单排)
            case 1: y0 = (p.y6()+p.y7())*0.5 + g; y1 = p.y7();                  break; // 行1上排
            case 2: y0 = p.y6();                  y1 = (p.y6()+p.y7())*0.5 - g; break; // 行1下排
            case 3: y0 = (p.y4()+p.y5())*0.5 + g; y1 = p.y5();                  break; // 行2上排
            case 4: y0 = p.y4();                  y1 = (p.y4()+p.y5())*0.5 - g; break; // 行2下排
            case 5: y0 = (p.y2()+p.y3())*0.5 + g; y1 = p.y3();                  break; // 行3上排
            case 6: y0 = p.y2();                  y1 = (p.y2()+p.y3())*0.5 - g; break; // 行3下排
            default:y0 = 0.0;                     y1 = p.bottom_shelf_depth;    break; // 底货架(单排)
        }
    }

    // ── 货位（空位的槽）：亮粉色填充 + 浅粉描边，严格贴合货架物理格子 ──────────
    //   X = 货位宽(vehicle_width，与 col_centers 设计一致)，Y = 该排货架物理深度区间。
    void addSlots() {
        const auto& p = map_->param();
        double hw = p.vehicle_width * 0.5;
        double z = 0.012, lw = 0.006;

        for (const auto& s : map_->slots()) {
            double y0, y1;
            shelfCellY(p, s.row_id, y0, y1);
            Rect2D r;
            if (!clipRectToContent(p, s.cx - hw, y0,
                                   p.vehicle_width, y1 - y0, r)) continue;
            // 填充
            addCube(arr_, frame_, "slots", newId(),
                    r.x + r.w * 0.5, r.y + r.h * 0.5, z,
                    r.w, r.h, 0.001, Color::kSlot);
            // 描边
            addLineStrip(arr_, frame_, "slots", newId(),
                         {pt(r.x,     r.y,     z + 0.001),
                          pt(r.x+r.w, r.y,     z + 0.001),
                          pt(r.x+r.w, r.y+r.h, z + 0.001),
                          pt(r.x,     r.y+r.h, z + 0.001),
                          pt(r.x,     r.y,     z + 0.001)},
                         lw, Color::kSlotEdge);
            if (show_slot_ids_) {
                addText(arr_, frame_, "slot_ids", newId(),
                        r.x + r.w * 0.5, r.y + r.h * 0.5, z + 0.010,
                        0.070, std::to_string(s.id), hex(0x102008));
            }
            if (show_slot_poses_) {
                const double arrow_len = std::max(0.055, p.arrow_size);
                addArrow(arr_, frame_, "slot_pose_arrow", newId(),
                         s.cx, s.cy, z + 0.020, s.dock_theta,
                         arrow_len, 0.010, 0.026, Color::kSlotPoseArrow);

                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2)
                   << "(" << s.cx << "," << s.cy << ") "
                   << std::setprecision(0) << s.dock_theta * 180.0 / kPi << "deg";
                addText(arr_, frame_, "slot_pose_text", newId(),
                        s.cx, s.cy + 0.055 * ((std::sin(s.dock_theta) >= 0.0) ? 1.0 : -1.0),
                        z + 0.030, 0.035, ss.str(), Color::kSlotPoseText);
            }
        }
    }

    // ── 路网骨架：黄色车道中心线 + 共性拐弯（曲率连续 clothoid）────────────────
    //   只画"共性"：所有车都会走的横向走廊、连接走廊的纵向通道、场地进出场口，
    //   以及多辆车都会经过的拐角（曲率连续曲线，与规划器同一份算法/参数）。
    //   不画：方向箭头、进每个货位的末端曲线、停车节点弧、尖点。
    //   纵向通道 x 取值、拐弯参数均与 path_generator 路由一致（共性路径骨架）。
    void addRoads() {
        const auto& p = map_->param();
        const double z  = 0.018;
        const double lw = p.road_line_width;
        const double W  = p.field_width;
        const double H  = p.field_height;
        const double h_off = std::min(0.14, p.corridor_height * 0.22);

        // 拐弯参数：直接取自 MapParam（单一参数源，与规划器同源同算法）
        const TurnParam tp{p.turn_max_curvature(), p.turn_ramp_len(), p.turn_ds()};
        const double cy[5] = {0.0, p.corridor1_cy(), p.corridor2_cy(),
                              p.corridor3_cy(), p.corridor4_cy()};  // 1..4

        // 横向走廊：每条走廊画两条方向中心线（上行/下行），全宽贯通
        for (int i = 1; i <= 4; ++i) {
            addLineStrip(arr_, frame_, "roads_h", newId(),
                         {clippedPt(p, 0, cy[i] + h_off, z),
                          clippedPt(p, W, cy[i] + h_off, z)}, lw, Color::kLane);
            addLineStrip(arr_, frame_, "roads_h", newId(),
                         {clippedPt(p, 0, cy[i] - h_off, z),
                          clippedPt(p, W, cy[i] - h_off, z)}, lw, Color::kLane);
        }

        const double row1_off = p.row1_left_aisle   * 0.25;  // == dual_lane_offset
        const double row3_off = p.row3_center_aisle * 0.25;
        const double left_bp  = p.row1_left_aisle * 0.5;
        const double right_bp = (p.row1_left_aisle + p.row1_shelf_width
                               + (W - p.row1_mini_shelf)) * 0.5;
        const double spine    = W * 0.5;

        // 纵向连接通道：cx, 连接的上/下走廊编号。各通道紧贴对应走廊的近端车道。
        struct Conn { double cx; int up_corr, lo_corr; };
        const std::vector<Conn> conns = {
            {left_bp  - row1_off, 1, 2}, {left_bp  + row1_off, 1, 2},   // 左侧绕行
            {right_bp - row1_off, 1, 2}, {right_bp + row1_off, 1, 2},   // 右侧绕行
            {spine,               2, 3},                                 // 中央脊（行2缝隙）
            {spine - row3_off,    3, 4}, {spine + row3_off,    3, 4},   // 行3双车道
        };

        for (const auto& c : conns) {
            const double y_hi = cy[c.up_corr] - h_off;  // 接上走廊的近端(下)车道
            const double y_lo = cy[c.lo_corr] + h_off;  // 接下走廊的近端(上)车道
            // 拐弯切线长按连接段长自适应（两端拐弯不互相穿插，中间留直线）
            const double mt = (y_hi - y_lo) * 0.45;
            const double t  = fittedTangent90(tp, mt);
            if (t <= 0.0) continue;  // 空间放不下曲率连续拐弯
            // 纵向中心线（两端各留出 t 给拐弯）
            addLineStrip(arr_, frame_, "roads_v", newId(),
                         {clippedPt(p, c.cx, y_lo + t, z),
                          clippedPt(p, c.cx, y_hi - t, z)}, lw, Color::kLane);
            // 共性拐弯：上端通道在下方(channel_dir=-1)，下端在上方(+1)；
            //           走廊左右来车(din_sign=±1)各画一条（靠墙越界者自动跳过）
            for (int din : {+1, -1}) {
                addClothoidTurn(arr_, p, frame_, "roads_arc", newId(),
                                c.cx, y_hi, din, -1, tp, mt, z, lw, Color::kLane);
                addClothoidTurn(arr_, p, frame_, "roads_arc", newId(),
                                c.cx, y_lo, din, +1, tp, mt, z, lw, Color::kLane);
            }
        }

        // ── 场地进出场口（共性：所有车进出场地都走中央脊顶/底段）──────────────
        // 顶部出场口：corridor1 上侧车道 → 顶部开口；底部：corridor4 下侧车道 → 底部开口
        const double top_corner_y = cy[1] + h_off;   // ≈4.070
        const double bot_corner_y = cy[4] - h_off;   // ≈0.430
        const double mt_port = 0.30;
        const double t_port  = fittedTangent90(tp, mt_port);
        if (t_port > 0.0) {
            addLineStrip(arr_, frame_, "roads_v", newId(),
                         {clippedPt(p, spine, top_corner_y + t_port, z),
                          clippedPt(p, spine, H, z)}, lw, Color::kLane);
            addLineStrip(arr_, frame_, "roads_v", newId(),
                         {clippedPt(p, spine, 0.0, z),
                          clippedPt(p, spine, bot_corner_y - t_port, z)}, lw, Color::kLane);
            for (int din : {+1, -1}) {
                addClothoidTurn(arr_, p, frame_, "roads_arc", newId(),
                                spine, top_corner_y, din, +1, tp, mt_port, z, lw, Color::kLane);
                addClothoidTurn(arr_, p, frame_, "roads_arc", newId(),
                                spine, bot_corner_y, din, -1, tp, mt_port, z, lw, Color::kLane);
            }
        }
    }
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "map_node");
    MapNode node;
    ros::spin();
    return 0;
}
