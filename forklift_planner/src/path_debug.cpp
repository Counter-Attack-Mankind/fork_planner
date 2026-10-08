// [DIAG] Standalone path-generator probe (no ROS runtime needed). Builds the
// map + planner from default params (which match the yaml) and reports, for a
// given source slot, why each outbound path is rejected -- mirroring the
// validatePath precedence (ARC > KINK > out-of-bounds). With two args it dumps
// the full waypoint list of one src->tgt path, flagging out-of-bounds samples.
//
//   rosrun forklift_planner path_debug 21          # scan all targets from 21
//   rosrun forklift_planner path_debug 21 16        # dump path 21 -> 16
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"

using forklift_planner::multi_vehicle::footprintInsideField;
using forklift_planner::multi_vehicle::bodyCenterPose;

// Replica of task_allocator's runtime heading-continuity pass, so this probe
// shows the EXACT rendered body trajectory (generate + makeHeadingContinuous).
static double dbgNorm(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}
static void dbgMakeHeadingContinuous(RoughPath& path) {
    const int n = static_cast<int>(path.size());
    if (n < 2) return;
    auto nextMove = [&](int i) -> int {
        for (int j = i + 1; j < n; ++j)
            if (std::hypot(path[j].x - path[i].x, path[j].y - path[i].y) > 1e-6) return j;
        return -1;
    };
    const double kCusp = 2.62;
    double prev_m = 0.0; bool have_prev = false, rev = false, seeded = false;
    for (int i = 0; i < n; ++i) {
        const int j = nextMove(i);
        if (j < 0) { if (i > 0) { path[i].theta = path[i-1].theta; path[i].type = path[i-1].type; } continue; }
        const double m = std::atan2(path[j].y - path[i].y, path[j].x - path[i].x);
        if (!seeded) { rev = std::abs(dbgNorm(path[i].theta - m)) > M_PI * 0.5; seeded = true; }
        else if (have_prev && std::abs(dbgNorm(m - prev_m)) > kCusp) rev = !rev;
        path[i].theta = dbgNorm(m + (rev ? M_PI : 0.0));
        path[i].type = rev ? WpType::REVERSE : WpType::FORWARD;
        prev_m = m; have_prev = true;
    }
}

static void dbgDockBodyToCenter(RoughPath& path, const Slot& src, const Slot& tgt,
                                double d) {
    const size_t n = path.size();
    if (n < 2) return;
    auto place = [&](size_t end, size_t nbr, double cx, double cy) {
        const double nx = cx - d * std::cos(path[end].theta);
        const double ny = cy - d * std::sin(path[end].theta);
        const double ax = path[end].x - path[nbr].x, ay = path[end].y - path[nbr].y;
        const double mx = nx - path[nbr].x, my = ny - path[nbr].y;
        if (ax * mx + ay * my > 0.0) { path[end].x = nx; path[end].y = ny; }
    };
    place(0, 1, src.dock_x(), src.dock_y());
    place(n - 1, n - 2, tgt.dock_x(), tgt.dock_y());
}

static double kinkMaxAngle(const RoughPath& wp, size_t* at) {
    double worst = 0.0;
    *at = 0;
    for (size_t i = 1; i + 1 < wp.size(); ++i) {
        const double d1x = wp[i].x - wp[i - 1].x, d1y = wp[i].y - wp[i - 1].y;
        const double d2x = wp[i + 1].x - wp[i].x, d2y = wp[i + 1].y - wp[i].y;
        const double n1 = std::hypot(d1x, d1y), n2 = std::hypot(d2x, d2y);
        if (n1 < 1e-4 || n2 < 1e-4) continue;
        double c = (d1x * d2x + d1y * d2y) / (n1 * n2);
        c = std::max(-1.0, std::min(1.0, c));
        const double a = std::acos(c);
        if (a > worst) { worst = a; *at = i; }
    }
    return worst;
}

int main(int argc, char** argv) {
    MapParam mp;       // defaults match map_param.yaml
    PlannerParam pp;   // defaults match planner_param.yaml
    ForkliftMap map(mp);
    PathGenerator gen(mp, pp);
    const auto& slots = map.slots();
    const int n = static_cast<int>(slots.size());

    const int src = (argc > 1) ? std::atoi(argv[1]) : 21;
    const int probe_tgt = (argc > 2) ? std::atoi(argv[2]) : -1;
    auto label = [&](int id) {
        const Slot& s = slots[id];
        char b[96];
        std::snprintf(b, sizeof b, "slot %d r%d c%d(x=%.3f y=%.3f th=%.0f)",
                      s.id, s.row_id, s.col, s.dock_x(), s.dock_y(),
                      s.dock_theta * 180.0 / M_PI);
        return std::string(b);
    };

    if (probe_tgt >= 0) {
        PathGenerationInfo info;
        // 后轴中心轨迹（生成器内已把停靠点沿鼻向后移 d）。footprint/碰撞从后轴向前
        // 偏移 d 还原车身中心。
        const std::string opt = (argc > 4) ? std::string(argv[4]) : "";
        const bool want_cont = opt.find("cont") != std::string::npos;
        RoughPath path = gen.generate(slots[src], slots[probe_tgt], &info);
        if (want_cont) {
            dbgMakeHeadingContinuous(path);
            dbgDockBodyToCenter(path, slots[src], slots[probe_tgt],
                                mp.rear_axle_to_center);
        }
        std::printf("== %s -> %s : wpts=%zu arc=%d (REAR-AXLE path; d=%.4f) ==\n",
                    label(src).c_str(), label(probe_tgt).c_str(), path.size(),
                    info.used_arc_fallback ? 1 : 0, mp.rear_axle_to_center);
        for (size_t i = 0; i < path.size(); ++i) {
            const bool inb = footprintInsideField(path[i], mp, 0.0);
            const RoughWp body = bodyCenterPose(path[i], mp);
            std::printf("  wp%3zu rear(%.4f,%.4f) body(%.4f,%.4f) th=%9.4f %s %s\n",
                        i, path[i].x, path[i].y, body.x, body.y,
                        path[i].theta * 180.0 / M_PI,
                        path[i].type == WpType::REVERSE ? "REV" : "FWD",
                        inb ? "" : "<<<< OUT OF BOUNDS");
        }
        // 停位精度：终点车身中心 = 后轴 + d·朝向，应落在目标库位中心。
        const RoughWp bc = bodyCenterPose(path.back(), mp);
        const Slot& tg = slots[probe_tgt];
        std::printf("DOCK CHECK: body_center=(%.4f,%.4f) slot_center=(%.4f,%.4f) "
                    "err=%.4f m\n", bc.x, bc.y, tg.dock_x(), tg.dock_y(),
                    std::hypot(bc.x - tg.dock_x(), bc.y - tg.dock_y()));
        return 0;
    }

    int ok = 0, arc = 0, kinkn = 0, oobn = 0, empty = 0;
    for (int t = 0; t < n; ++t) {
        if (t == src) continue;
        PathGenerationInfo info;
        RoughPath path = gen.generate(slots[src], slots[t], &info);
        if (path.size() < 2) { ++empty; continue; }
        size_t kat = 0;
        const double ka = kinkMaxAngle(path, &kat);
        const bool has_kink = (ka > 0.61 && ka < 2.53);
        int oob_i = -1;
        for (size_t i = 0; i < path.size(); ++i) {
            if (!footprintInsideField(path[i], mp, 0.0)) { oob_i = static_cast<int>(i); break; }
        }
        const char* cls;
        if (info.used_arc_fallback) { cls = "ARC"; ++arc; }
        else if (has_kink) { cls = "KINK"; ++kinkn; }
        else if (oob_i >= 0) { cls = "OOB"; ++oobn; }
        else { cls = "ok"; ++ok; }
        if (std::string(cls) != "ok") {
            std::printf("%-4s -> %s : arc=%d kink=%.0fdeg@wp%zu oob@wp%d/%zu\n",
                        cls, label(t).c_str(), info.used_arc_fallback ? 1 : 0,
                        ka * 180.0 / M_PI, kat, oob_i, path.size());
        }
    }
    std::printf("\nFROM %s : ok=%d arc=%d kink=%d oob=%d empty=%d (of %d)\n",
                label(src).c_str(), ok, arc, kinkn, oobn, empty, n - 1);
    return 0;
}
