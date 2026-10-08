#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <ros/time.h>

#include "forklift_map/forklift_map.h"
#include "forklift_planner/multi_vehicle/conflict_zone_closure.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/path_track.h"
#include "forklift_planner/path_generator.h"

namespace fs = std::filesystem;
using forklift_planner::multi_vehicle::OBB;
using forklift_planner::multi_vehicle::PathTrack;
using forklift_planner::multi_vehicle::makeBody;
using forklift_planner::multi_vehicle::overlaps;
using forklift_planner::multi_vehicle::insertConflictZoneWithClosure;

namespace {

struct Zone {
    double self_enter = 0.0;
    double self_exit = 0.0;
    double other_enter = 0.0;
    double other_exit = 0.0;
    int lineage = -1;
};

struct Args {
    fs::path out_dir;
    double step = 0.025;
    double merge_gap = -1.0;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string value(argv[i]);
        if (value == "--out" && i + 1 < argc) {
            args.out_dir = argv[++i];
        } else if (value == "--step" && i + 1 < argc) {
            args.step = std::stod(argv[++i]);
        } else if (value == "--merge-gap" && i + 1 < argc) {
            args.merge_gap = std::stod(argv[++i]);
        } else {
            throw std::runtime_error("unknown/incomplete argument: " + value);
        }
    }
    if (args.out_dir.empty()) {
        throw std::runtime_error("--out is required");
    }
    if (args.step <= 0.0) throw std::runtime_error("--step must be positive");
    if (args.merge_gap < 0.0) args.merge_gap = args.step * 2.25;
    return args;
}

Slot makeA1() {
    Slot a1;
    a1.id = 101;
    a1.row_id = 0;
    a1.col = -1;
    a1.cx = 1.250;
    a1.cy = 4.375;
    a1.pre_dock_x = 1.250;
    a1.pre_dock_y = 4.100;
    a1.dock_theta = 1.5707963267948966;
    return a1;
}

const char* typeName(WpType type) {
    return type == WpType::REVERSE ? "REVERSE" : "FORWARD";
}

double intervalGap(double a0, double a1, double b0, double b1) {
    if (a1 < b0) return b0 - a1;
    if (b1 < a0) return a0 - b1;
    return 0.0;
}

void writePath(const fs::path& path, const PathTrack& track) {
    std::ofstream out(path);
    out << "s,x,y,theta,wp_type\n" << std::setprecision(10);
    constexpr double kPoseStep = 0.00625;
    for (double s = 0.0; s <= track.length() + 1e-9; s += kPoseStep) {
        const double sc = std::min(s, track.length());
        const RoughWp pose = track.poseAtS(sc);
        out << sc << ',' << pose.x << ',' << pose.y << ',' << pose.theta << ','
            << typeName(track.typeAtS(sc)) << '\n';
    }
}

void writeZoneSummary(const fs::path& path, const std::vector<Zone>& zones,
                      double step, double merge_gap,
                      double self_length, double other_length) {
    std::ofstream out(path);
    out << "step,merge_gap,self_length,other_length,zone_count\n"
        << std::setprecision(10) << step << ',' << merge_gap << ','
        << self_length << ',' << other_length << ',' << zones.size() << "\n\n";
    out << "raw_index,self_enter,self_exit,other_enter,other_exit,lineage\n";
    for (size_t i = 0; i < zones.size(); ++i) {
        const Zone& z = zones[i];
        out << i << ',' << z.self_enter << ',' << z.self_exit << ','
            << z.other_enter << ',' << z.other_exit << ',' << z.lineage << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        // Path generation emits ROS diagnostics even though this probe has no
        // node or ROS graph dependency.
        ros::Time::init();
        const Args args = parseArgs(argc, argv);
        fs::create_directories(args.out_dir);

        // Values are the checked-in map/planner YAML baseline used by EXP-001.
        MapParam mp;
        mp.max_steer_angle = 0.40;
        mp.rear_axle_to_center = mp.vehicle_length * 0.5 - mp.rear_hang;
        PlannerParam pp;
        pp.turn_model = "clothoid";
        pp.terminal_docking_mode = "auto";
        ForkliftMap map(mp);
        const Slot a1 = makeA1();
        const Slot& b21 = map.slots().at(21);
        const Slot& b31 = map.slots().at(31);

        PathGenerator a1_to_b(mp, pp, PathGeneratorRouteMode::A1_TO_B);
        PathGenerator b_to_a1(mp, pp, PathGeneratorRouteMode::B_TO_A1);
        PathGenerationInfo info_v0;
        PathGenerationInfo info_v1;
        const RoughPath path_v0 = a1_to_b.generate(a1, b21, &info_v0);
        const RoughPath path_v1 = b_to_a1.generate(b31, a1, &info_v1);
        if (path_v0.empty() || path_v1.empty()) {
            throw std::runtime_error("path generation returned an empty path");
        }

        PathTrack v0;
        PathTrack v1;
        v0.set(path_v0);
        v1.set(path_v1);
        writePath(args.out_dir / "v0_path.csv", v0);
        writePath(args.out_dir / "v1_path.csv", v1);

        std::ofstream points(args.out_dir / "overlap_points.csv");
        std::ofstream row_splits(args.out_dir / "row_splits.csv");
        std::ofstream merge_trace(args.out_dir / "merge_trace.csv");
        std::ofstream bridge_rows(args.out_dir / "bridge_rows.csv");
        points << "s_v0,s_v1,overlap,v0_x,v0_y,v0_theta,v0_type,"
                  "v1_x,v1_y,v1_theta,v1_type,center_distance\n";
        row_splits << "s_v0,previous_other_exit,new_other_enter,other_gap,"
                      "merge_gap\n";
        merge_trace << "s_v0,row_other_enter,row_other_exit,candidate_raw,"
                       "candidate_self_enter,candidate_self_exit,"
                       "candidate_other_enter,candidate_other_exit,"
                       "self_touch,other_touch,self_gap,other_gap,merge_gap,"
                       "outcome\n";
        bridge_rows << "s_v0,row_other_enter,row_other_exit,touch_count,"
                       "touch_raw_indices,merge_gap\n";
        points << std::setprecision(10);
        row_splits << std::setprecision(10);
        merge_trace << std::setprecision(10);
        bridge_rows << std::setprecision(10);

        constexpr double kConflictMargin = 0.04;
        const double cm = kConflictMargin * 0.5;
        std::vector<Zone> zones;
        size_t overlap_count = 0;

        for (double ss = 0.0; ss <= v0.length() + 1e-9; ss += args.step) {
            const double s0 = std::min(ss, v0.length());
            const RoughWp p0 = v0.poseAtS(s0);
            const OBB obb0 = makeBody(p0, mp, cm);
            std::vector<double> row;

            for (double so = 0.0; so <= v1.length() + 1e-9; so += args.step) {
                const double s1 = std::min(so, v1.length());
                const RoughWp p1 = v1.poseAtS(s1);
                const OBB obb1 = makeBody(p1, mp, cm);
                if (!overlaps(obb0, obb1)) continue;
                row.push_back(s1);
                ++overlap_count;
                points << s0 << ',' << s1 << ",1," << p0.x << ',' << p0.y
                       << ',' << p0.theta << ',' << typeName(v0.typeAtS(s0))
                       << ',' << p1.x << ',' << p1.y << ',' << p1.theta << ','
                       << typeName(v1.typeAtS(s1)) << ','
                       << std::hypot(p0.x - p1.x, p0.y - p1.y) << '\n';
            }

            if (row.empty()) continue;
            std::vector<Zone> row_zones;
            for (double s1 : row) {
                if (row_zones.empty() ||
                    s1 > row_zones.back().other_exit + args.merge_gap) {
                    if (!row_zones.empty()) {
                        row_splits << s0 << ',' << row_zones.back().other_exit
                                   << ',' << s1 << ','
                                   << (s1 - row_zones.back().other_exit) << ','
                                   << args.merge_gap << '\n';
                    }
                    Zone z;
                    z.self_enter = z.self_exit = s0;
                    z.other_enter = z.other_exit = s1;
                    row_zones.push_back(z);
                } else {
                    row_zones.back().other_exit = s1;
                }
            }

            for (const Zone& row_zone : row_zones) {
                std::vector<size_t> touching;
                for (size_t zi = 0; zi < zones.size(); ++zi) {
                    const Zone& z = zones[zi];
                    const bool self_touch =
                        row_zone.self_enter <= z.self_exit + args.merge_gap;
                    const bool other_touch =
                        row_zone.other_enter <= z.other_exit + args.merge_gap &&
                        row_zone.other_exit + args.merge_gap >= z.other_enter;
                    if (self_touch && other_touch) touching.push_back(zi);
                }
                if (touching.size() >= 2) {
                    bridge_rows << s0 << ',' << row_zone.other_enter << ','
                                << row_zone.other_exit << ',' << touching.size()
                                << ',';
                    for (size_t i = 0; i < touching.size(); ++i) {
                        if (i != 0) bridge_rows << '|';
                        bridge_rows << touching[i];
                    }
                    bridge_rows << ',' << args.merge_gap << '\n';
                }
                for (size_t zi = 0; zi < zones.size(); ++zi) {
                    const Zone& z = zones[zi];
                    const bool self_touch =
                        row_zone.self_enter <= z.self_exit + args.merge_gap;
                    const bool other_touch =
                        row_zone.other_enter <= z.other_exit + args.merge_gap &&
                        row_zone.other_exit + args.merge_gap >= z.other_enter;
                    const double self_gap = std::max(
                        0.0, row_zone.self_enter - z.self_exit);
                    const double other_gap = intervalGap(
                        row_zone.other_enter, row_zone.other_exit,
                        z.other_enter, z.other_exit);
                    merge_trace << s0 << ',' << row_zone.other_enter << ','
                                << row_zone.other_exit << ',' << zi << ','
                                << z.self_enter << ',' << z.self_exit << ','
                                << z.other_enter << ',' << z.other_exit << ','
                                << (self_touch ? 1 : 0) << ','
                                << (other_touch ? 1 : 0) << ',' << self_gap
                                << ',' << other_gap << ',' << args.merge_gap
                                << ',' << ((self_touch && other_touch)
                                               ? "TOUCH"
                                               : "REJECT")
                                << '\n';
                }
                const auto touches = [&](const Zone& lhs, const Zone& rhs) {
                    return lhs.self_enter <= rhs.self_exit + args.merge_gap &&
                           lhs.self_exit + args.merge_gap >= rhs.self_enter &&
                           lhs.other_enter <= rhs.other_exit + args.merge_gap &&
                           lhs.other_exit + args.merge_gap >= rhs.other_enter;
                };
                const auto merge = [](Zone& destination, const Zone& source) {
                    destination.self_enter =
                        std::min(destination.self_enter, source.self_enter);
                    destination.self_exit =
                        std::max(destination.self_exit, source.self_exit);
                    destination.other_enter =
                        std::min(destination.other_enter, source.other_enter);
                    destination.other_exit =
                        std::max(destination.other_exit, source.other_exit);
                    if (destination.lineage < 0 ||
                        (source.lineage >= 0 &&
                         source.lineage < destination.lineage)) {
                        destination.lineage = source.lineage;
                    }
                };
                insertConflictZoneWithClosure(row_zone, zones, touches, merge);
            }
        }

        writeZoneSummary(args.out_dir / "zones.csv", zones, args.step,
                         args.merge_gap, v0.length(), v1.length());
        std::ofstream metadata(args.out_dir / "metadata.txt");
        metadata << std::setprecision(10)
                 << "scenario=V0_A1_to_B21_gen142__V1_B31_to_A1_gen137\n"
                 << "source_identity=EXP-001 task and path generation evidence\n"
                 << "v0_length=" << v0.length() << "\n"
                 << "v1_length=" << v1.length() << "\n"
                 << "v0_waypoints=" << path_v0.size() << "\n"
                 << "v1_waypoints=" << path_v1.size() << "\n"
                 << "v0_arc_fallback=" << info_v0.used_arc_fallback << "\n"
                 << "v1_arc_fallback=" << info_v1.used_arc_fallback << "\n"
                 << "step=" << args.step << "\n"
                 << "merge_gap=" << args.merge_gap << "\n"
                 << "conflict_margin=" << kConflictMargin << "\n"
                 << "overlap_points=" << overlap_count << "\n"
                 << "zone_count=" << zones.size() << "\n";

        std::cout << "out=" << args.out_dir << " overlap_points="
                  << overlap_count << " zones=" << zones.size() << '\n';
        for (size_t i = 0; i < zones.size(); ++i) {
            const Zone& z = zones[i];
            std::cout << "raw" << i << " V0[" << z.self_enter << ','
                      << z.self_exit << "] V1[" << z.other_enter << ','
                      << z.other_exit << "] lineage=" << z.lineage << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "conflict_zone_geometry_diag: " << error.what() << '\n';
        return 2;
    }
}
