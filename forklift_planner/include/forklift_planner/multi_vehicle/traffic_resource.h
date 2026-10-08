#pragma once

// 草履虫工程落地版 §11 的资源-请求模型核心词汇表。
// 这是把"两两 pairwise 让行"重构为"对资源统一仲裁"的基础数据结构:
//   - 资源(TrafficResource):窄道/路口/货位口/方向车道/等待点/临时冲突区
//   - 请求(ResourceRequest):每辆车每周期对它将用到的资源发出的占用请求
//   - 统一优先级(PriorityKey):任意数量车辆冲突时选出唯一 winner(§11.2)
//   - 令牌表(ResourceTokenTable):谁持有资源、防抖保持、超时释放(§11.10)
// 后续 Phase 2 的固定同步阶段流水线据此仲裁。

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace forklift_planner {
namespace multi_vehicle {

// 资源类型(汇报版§4.1 / 工程落地版§4、§5)。
enum class ResourceType {
    SLOT_TASK,      // 货位任务绑定:同一货位同一时刻只分配给一辆车
    SLOT_BODY,      // 货位内部占用:停留期间持有
    SLOT_DOCK,      // 货位口操作区:一次只允许一辆车进出
    INTERSECTION,   // 路口:会碰撞的车不可同时进入
    NARROW,         // 窄道/单车道缺口:一次只允许一个方向
    LANE,           // 方向车道:按行驶方向拆分,同向可排队
    WAIT_POINT,     // 合法等待点:有容量
    CONFLICT_ZONE,  // 临时冲突区:固定路径预测出的时空重叠段(§8.6)
};

// 请求类型,自带优先序 CLEAR > EXIT > PASS > ENTER > DWELL > WAIT(§11.1)。
// 数值越大越优先(方便比较)。
enum class RequestType {
    WAIT = 0,    // 普通等待(最低)
    DWELL = 1,   // 作业停留,不抢占道路
    ENTER = 2,   // 进入资源
    PASS = 3,    // 主路连续通过(优先于支路汇入)
    EXIT = 4,    // 离开资源(优先于进入)
    CLEAR = 5,   // 清障(最高,破死锁/让出阻塞位置)
};

inline int requestTypeRank(RequestType t) { return static_cast<int>(t); }

// 轴对齐矩形(资源几何)。本图所有走廊/车道/缺口/货位区都可用 AABB 表达。
struct Box {
    double x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0;
    bool contains(double x, double y) const {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

// 一个资源对象(静态几何 + 容量 + 方向 + 出入口/上下游等待点)。§12.1 资源表。
struct TrafficResource {
    int id = -1;
    ResourceType type = ResourceType::CONFLICT_ZONE;
    Box box;                                  // 资源占用的矩形区域(世界坐标)
    int capacity = 1;                         // 同时容纳车辆数(互斥资源=1)
    bool directional = false;                 // 是否方向车道(同向可排队,对向互斥)
    double lane_heading = 0.0;                 // directional=true 时的行驶朝向(rad)
    double min_headway_distance = 0.0;        // 同向排队最小车距(容量>1 时)
    int slot_id = -1;                         // SLOT_* 资源关联的货位
    std::vector<int> upstream_wait_points;    // 上游等待点资源 id
    std::vector<int> downstream_wait_points;  // 下游等待点资源 id(出口检查用)
    std::string name;                         // 调试用
};

// 一辆车在某资源上的弧长占用区间(由其固定路径与资源几何相交算出)。
struct ResourceSpan {
    int resource_id = -1;
    double s_enter = 0.0;   // 车辆参考点进入该资源的弧长
    double s_exit = 0.0;    // 车辆参考点完全驶出(含车身后伸)的弧长
};

// 一辆车对一个资源的占用请求(§11 的 ResourceRequest)。
struct ResourceRequest {
    int vehicle_id = -1;
    int resource_id = -1;
    RequestType type = RequestType::WAIT;
    double eta = 0.0;            // 预计到达该资源入口的时间(s)
    double wait_time = 0.0;      // 已等待时间
    bool loaded = false;
    bool task_replaceable = true;
    bool already_inside = false; // 车身已在资源内
    bool already_has_token = false;
    bool starving = false;       // wait_time > T_starvation
    bool non_replaceable_task = false;
    int task_count = 0;
    double s_enter = 0.0;
    double s_exit = 0.0;
    bool emergency_or_clear = false;  // 紧急/破死锁临时最高
};

// 统一优先级 key(§11.2)。最后一项必为 vehicle_id(零自由度 tie-break,§11.12)。
// betterThan(a,b)=true 表示 a 比 b 优先。字典序逐字段比较,每字段"大者优先"。
struct PriorityKey {
    // 按 §11.2 顺序排列;true/大 = 更优先。
    static bool betterThan(const ResourceRequest& a, const ResourceRequest& b) {
        // 1 紧急/清障
        if (a.emergency_or_clear != b.emergency_or_clear)
            return a.emergency_or_clear;
        // 2 请求类型序 CLEAR>EXIT>PASS>ENTER>DWELL>WAIT
        if (a.type != b.type) return requestTypeRank(a.type) > requestTypeRank(b.type);
        // 3 已在资源内
        if (a.already_inside != b.already_inside) return a.already_inside;
        // 4 已持令牌
        if (a.already_has_token != b.already_has_token) return a.already_has_token;
        // 5 饥饿保护(必须排在 load 前,否则空载永久被压,§11.2 注)
        if (a.starving != b.starving) return a.starving;
        // 6 等待更久
        if (a.wait_time != b.wait_time) return a.wait_time > b.wait_time;
        // 7 任务不可替代
        if (a.non_replaceable_task != b.non_replaceable_task)
            return a.non_replaceable_task;
        // 8 满载优先于空载
        if (a.loaded != b.loaded) return a.loaded;
        // 9 更早到冲突点(eta 小者优先)
        if (a.eta != b.eta) return a.eta < b.eta;
        // 10 任务数少者优先
        if (a.task_count != b.task_count) return a.task_count < b.task_count;
        // 11 id 小者(最终确定性 tie-break)
        return a.vehicle_id < b.vehicle_id;
    }
};

// 资源令牌表(§11.10 通行权防抖 + §11.13.1 超时)。记录每个资源当前持有者,
// 通行权一旦授予至少保持 hold_min,除紧急/故障外不允许每周期翻转 winner;
// 持有者长时间不刷新则超时释放。所有时间用仿真时间(秒)传入,保持确定性。
class ResourceTokenTable {
public:
    struct Token {
        int owner = -1;       // 持有者 vehicle_id
        double granted_t = 0.0;   // 授予时刻(sim time)
        double refreshed_t = 0.0; // 最近一次仍在用的刷新时刻
    };

    // 当前持有者;无人持有返回 -1。
    int holder(int resource_id) const {
        auto it = tokens_.find(resource_id);
        return it == tokens_.end() ? -1 : it->second.owner;
    }

    bool heldBy(int resource_id, int vehicle_id) const {
        return holder(resource_id) == vehicle_id;
    }

    // 是否可把资源授予 v:无人持有,或已是 v 持有,或当前持有者已超过最短保持期
    // 且不再刷新(留给上层在持有者不再请求时释放)。
    bool grantable(int resource_id, int vehicle_id) const {
        const int h = holder(resource_id);
        return h < 0 || h == vehicle_id;
    }

    void grant(int resource_id, int vehicle_id, double now) {
        Token& t = tokens_[resource_id];
        if (t.owner != vehicle_id) {
            t.owner = vehicle_id;
            t.granted_t = now;
        }
        t.refreshed_t = now;
    }

    // 持有时长是否已满足最短保持期(防抖:未满则不允许翻转给别人)。
    bool heldLongEnough(int resource_id, double now, double hold_min) const {
        auto it = tokens_.find(resource_id);
        if (it == tokens_.end()) return true;
        return now - it->second.granted_t >= hold_min;
    }

    void release(int resource_id) { tokens_.erase(resource_id); }

    // 超时清理:持有者超过 timeout 未刷新则释放(防止持令牌车异常后永久占用)。
    void expireStale(double now, double timeout) {
        for (auto it = tokens_.begin(); it != tokens_.end();) {
            if (now - it->second.refreshed_t > timeout) {
                it = tokens_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // 释放某车持有的所有令牌(任务结束/进入 DWELL/异常时)。
    void releaseAllOf(int vehicle_id) {
        for (auto it = tokens_.begin(); it != tokens_.end();) {
            if (it->second.owner == vehicle_id) {
                it = tokens_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    std::map<int, Token> tokens_;  // resource_id -> token
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
