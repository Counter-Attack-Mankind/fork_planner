#include "forklift_planner/multi_vehicle/traffic_resource_map.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace forklift_planner {
namespace multi_vehicle {

TrafficResourceMap::TrafficResourceMap(const MapParam& mp,
                                       const std::vector<Slot>& slots,
                                       const std::vector<RoadSegment>& roads)
    : mp_(mp) {
    buildSlotResources(slots);
    buildNarrowResources();
    buildCorridorLaneResources();
    buildIntersectionResources(roads);
    // 后续 Phase 1.3e 在此追加:等待点。
}

void TrafficResourceMap::buildIntersectionResources(
    const std::vector<RoadSegment>& roads) {
    // 路口 = 竖向行驶线穿过横向走廊带的交叉处(汇报版§8.2)。一辆走横向、一辆走
    // 竖向的车会在此处车身冲突,故按互斥资源(capacity=1)建。几何由地图实际的
    // 竖向 road_segments × 走廊 y 带数据驱动求交,不硬编码坐标。
    struct Corr { double y_lo, y_hi; const char* name; };
    const Corr corrs[] = {
        {mp_.y1(), mp_.y2(), "C4"}, {mp_.y3(), mp_.y4(), "C3"},
        {mp_.y5(), mp_.y6(), "C2"}, {mp_.y7(), mp_.y8(), "C1"},
    };
    const double hw = mp_.vehicle_width;  // 路口盒沿竖线两侧的半宽(覆盖转弯车身)
    for (const RoadSegment& seg : roads) {
        if (seg.is_horizontal) continue;          // 只看竖向连接线
        const double sx = seg.x0;                  // 竖向线 x
        const double sy_lo = std::min(seg.y0, seg.y1);
        const double sy_hi = std::max(seg.y0, seg.y1);
        for (const Corr& c : corrs) {
            // 竖向段是否覆盖该走廊带(有重叠才有路口)。
            if (sy_hi < c.y_lo - 1e-6 || sy_lo > c.y_hi + 1e-6) continue;
            TrafficResource r;
            r.type = ResourceType::INTERSECTION;
            r.capacity = 1;
            r.box = Box{sx - hw, sx + hw, c.y_lo, c.y_hi};
            r.name = std::string("XING_") + c.name + "_x" +
                     std::to_string(static_cast<int>(sx * 1000));
            addResource(std::move(r));
        }
    }
}

void TrafficResourceMap::buildCorridorLaneResources() {
    // 四条横向走廊,每条按行驶方向拆成上/下两条半幅车道(汇报版§4.1/§8.5)。
    // 对向车天然落在不同半幅 → 不同资源,自然分离;同向车共用一条 → 排队跟车
    // (capacity>1,headway 由 §8.4 跟车规则控制)。半幅切分比硬编码 ±0.14 车道
    // 中心更稳:车在哪半幅就属哪条车道,不依赖路径生成器的精确车道偏移。
    const double W = mp_.field_width;
    struct Corr { double y_lo, y_hi; const char* name; };
    const Corr corrs[] = {
        {mp_.y1(), mp_.y2(), "C4"},  // 走廊4(最下)
        {mp_.y3(), mp_.y4(), "C3"},
        {mp_.y5(), mp_.y6(), "C2"},
        {mp_.y7(), mp_.y8(), "C1"},  // 走廊1(最上)
    };
    const double headway = mp_.vehicle_length + mp_.vehicle_width;  // 跟车最小净距参考
    for (const Corr& c : corrs) {
        const double cy = 0.5 * (c.y_lo + c.y_hi);
        // 下半幅 / 上半幅各一条方向车道。
        const std::pair<double, double> halves[] = {
            {c.y_lo, cy}, {cy, c.y_hi}};
        const char* tag[] = {"lo", "hi"};
        for (int h = 0; h < 2; ++h) {
            TrafficResource r;
            r.type = ResourceType::LANE;
            r.directional = true;
            r.capacity = 4;  // 同向排队容量(后续可按走廊长度精调)
            r.min_headway_distance = headway;
            r.box = Box{0.0, W, halves[h].first, halves[h].second};
            r.name = std::string("LANE_") + c.name + "_" + tag[h];
            addResource(std::move(r));
        }
    }
}

void TrafficResourceMap::buildNarrowResources() {
    // 本图中央竖向连接线 x = field_width/2,被货架打断,只在三处留单车道缺口
    // (汇报版§4.1):顶部中央缺口、行2瓶颈、底部中央缺口。缺口宽 = row2_gap。
    // 这三处一次只允许一辆车(任意方向),按互斥资源 capacity=1 处理。
    const double cx = mp_.field_width * 0.5;
    const double half = mp_.row2_gap * 0.5;
    struct Band { double y_min, y_max; const char* name; };
    const Band bands[] = {
        {0.0, mp_.y1(), "NARROW_bottom_gap"},          // 底部货架带
        {mp_.y4(), mp_.y5(), "NARROW_row2_bottleneck"},// 行2货架带(瓶颈)
        {mp_.y8(), mp_.field_height, "NARROW_top_gap"},// 顶部货架带
    };
    for (const Band& b : bands) {
        TrafficResource r;
        r.type = ResourceType::NARROW;
        r.capacity = 1;  // 单车道:对向/同向都互斥,一次一辆
        r.box = Box{cx - half, cx + half, b.y_min, b.y_max};
        r.name = b.name;
        addResource(std::move(r));
    }
}

int TrafficResourceMap::addResource(TrafficResource r) {
    r.id = static_cast<int>(resources_.size());
    resources_.push_back(std::move(r));
    return resources_.back().id;
}

void TrafficResourceMap::buildSlotResources(const std::vector<Slot>& slots) {
    // 每个货位拆成(工程落地版§4):
    //   SLOT_BODY_k —— 货位内部矩形(车身停在里面)
    //   SLOT_DOCK_k —— 货位口操作区(货位中心↔pre_dock 之间的进出通道)
    // SLOT_TASK_k 是逻辑任务绑定(无几何),在仲裁层用,不建几何资源。
    const double hw = mp_.vehicle_width * 0.5;
    const double hl = mp_.vehicle_length * 0.5;
    for (const Slot& s : slots) {
        // 货位内部:以货位中心为心的车身大小矩形。
        {
            TrafficResource r;
            r.type = ResourceType::SLOT_BODY;
            r.capacity = 1;
            r.slot_id = s.id;
            r.box = Box{s.dock_x() - hw, s.dock_x() + hw,
                        s.dock_y() - hl, s.dock_y() + hl};
            r.name = "SLOT_BODY_" + std::to_string(s.id);
            addResource(std::move(r));
        }
        // 货位口:货位中心与 pre_dock 之间的矩形通道(含两端),宽=车宽。
        {
            const double x0 = std::min(s.dock_x(), s.pre_dock_x);
            const double x1 = std::max(s.dock_x(), s.pre_dock_x);
            const double y0 = std::min(s.dock_y(), s.pre_dock_y);
            const double y1 = std::max(s.dock_y(), s.pre_dock_y);
            // 沿停靠朝向是细长通道;横向按车宽扩。dock 多为南北向(沿 Y),
            // 故 X 方向按车宽扩、Y 方向取 [center, pre_dock];反之亦然。这里用
            // 统一做法:两端点连成的矩形,再向四周按 hw 扩,覆盖口部车身扫掠。
            TrafficResource r;
            r.type = ResourceType::SLOT_DOCK;
            r.capacity = 1;
            r.slot_id = s.id;
            r.box = Box{x0 - hw, x1 + hw, y0 - hw, y1 + hw};
            r.name = "SLOT_DOCK_" + std::to_string(s.id);
            addResource(std::move(r));
        }
    }
}

std::vector<ResourceSpan> TrafficResourceMap::spansForPath(
    const PathTrack& track) const {
    std::vector<ResourceSpan> spans;
    if (track.empty()) return spans;

    const double len = track.length();
    constexpr double kStep = 0.02;  // 采样步长

    // 对每个资源,沿弧长找参考点落在资源矩形内的首末弧长。允许同一资源被路径
    // 多次进出(分段),这里取最外包络[首次进入, 最后驶出]——对单调通过足够;
    // 若将来出现往返,可扩展为多区间。
    const size_t nres = resources_.size();
    std::vector<double> first(nres, -1.0), last(nres, -1.0);

    for (double s = 0.0; s <= len + 1e-9; s += kStep) {
        const RoughWp p = track.poseAtS(std::min(s, len));
        for (size_t i = 0; i < nres; ++i) {
            if (resources_[i].box.contains(p.x, p.y)) {
                if (first[i] < 0.0) first[i] = s;
                last[i] = s;
            }
        }
    }

    for (size_t i = 0; i < nres; ++i) {
        if (first[i] >= 0.0) {
            ResourceSpan span;
            span.resource_id = resources_[i].id;
            span.s_enter = first[i];
            span.s_exit = std::min(last[i] + kStep, len);
            spans.push_back(span);
        }
    }
    return spans;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
