#pragma once

// 资源地图(草履虫工程落地版§10 TrafficResourceMap + §12.1 资源表)。
// 按本图实际几何(MapParam + 货位)构建静态交通资源:货位口/货位内部、窄道、
// 方向车道、路口、等待点。并提供:给定一条固定路径 → 它经过哪些资源、各自的
// 弧长占用区间(ResourceSpan),供 Phase 2 的资源仲裁/原子段授权使用。
//
// 当前进度:1.3a 货位资源(SLOT_BODY/SLOT_DOCK)+ spansForPath。
// 后续:1.3b 窄道、1.3c 方向车道、1.3d 路口、1.3e 等待点。

#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"
#include "forklift_planner/multi_vehicle/path_track.h"
#include "forklift_planner/multi_vehicle/traffic_resource.h"

namespace forklift_planner {
namespace multi_vehicle {

class TrafficResourceMap {
public:
    TrafficResourceMap(const MapParam& mp, const std::vector<Slot>& slots,
                       const std::vector<RoadSegment>& roads);

    const std::vector<TrafficResource>& resources() const { return resources_; }
    const TrafficResource* byId(int id) const {
        return (id >= 0 && id < static_cast<int>(resources_.size()))
                   ? &resources_[static_cast<size_t>(id)]
                   : nullptr;
    }

    // 给定一辆车的固定路径,返回它沿途经过的每个资源的弧长占用区间。
    // s_enter/s_exit 为参考点(后轴)进入/驶出资源矩形的弧长。
    std::vector<ResourceSpan> spansForPath(const PathTrack& track) const;

private:
    int addResource(TrafficResource r);  // 分配 id 并存入,返回 id
    void buildSlotResources(const std::vector<Slot>& slots);
    void buildNarrowResources();  // 单车道窄道:顶/底中央缺口、行2瓶颈(§4.1)
    void buildCorridorLaneResources();  // 四走廊各拆上/下半幅方向车道(§4.1/§8.5)
    void buildIntersectionResources(const std::vector<RoadSegment>& roads);  // §8.2 路口

    const MapParam& mp_;
    std::vector<TrafficResource> resources_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
