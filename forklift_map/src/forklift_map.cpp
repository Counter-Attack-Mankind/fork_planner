#include "forklift_map/forklift_map.h"
#include <cmath>
#include <algorithm>

static constexpr double kPi = M_PI;

ForkliftMap::ForkliftMap(const MapParam& p) : p_(p) {
    build();
}

std::vector<Slot> ForkliftMap::free_slots() const {
    std::vector<Slot> out;
    for (auto& s : slots_)
        if (!s.occupied) out.push_back(s);
    return out;
}

// ─── private helpers ──────────────────────────────────────────────────────────

std::vector<double> ForkliftMap::col_centers(double shelf_x_min,
                                              double shelf_w,
                                              int n_cols) const {
    double gap = (shelf_w - n_cols * p_.vehicle_width) / (n_cols + 1);
    double first = shelf_x_min + gap + p_.vehicle_width * 0.5;
    double spacing = p_.vehicle_width + gap;
    std::vector<double> xs(n_cols);
    for (int i = 0; i < n_cols; ++i) xs[i] = first + i * spacing;
    return xs;
}

// ─── build ────────────────────────────────────────────────────────────────────

void ForkliftMap::build() {
    build_shelves();
    build_slots();
    build_roads();
}

void ForkliftMap::build_shelves() {
    const double W  = p_.field_width;
    const double sl = p_.vehicle_length;  // slot depth (Y)

    // ── 顶货架 (单排，轻紫) ──────────────────────────────────────────────────
    double tb_gap = W - 2.0 * p_.tb_shelf_width;  // = 0.25m
    shelf_blocks_.push_back({"顶货架左", 0.0, p_.y8(),
                              p_.tb_shelf_width, p_.bottom_shelf_depth,
                              false, true});
    shelf_blocks_.push_back({"顶货架右", p_.tb_shelf_width + tb_gap, p_.y8(),
                              p_.tb_shelf_width, p_.bottom_shelf_depth,
                              false, true});

    // ── 行1 (中央岛，双排，深紫) ─────────────────────────────────────────────
    shelf_blocks_.push_back({"行1主货架", p_.row1_left_aisle, p_.y6(),
                              p_.row1_shelf_width, p_.shelf_row_depth,
                              true, false});
    double row1_mini_x = W - p_.row1_mini_shelf;  // = 2.300
    shelf_blocks_.push_back({"行1小货架", row1_mini_x, p_.y6(),
                              p_.row1_mini_shelf, p_.shelf_row_depth,
                              true, false});

    // ── 行2 (瓶颈，双排，深紫) ───────────────────────────────────────────────
    double row2_right_x = p_.row2_left_width + p_.row2_gap;  // = 1.375
    shelf_blocks_.push_back({"行2左货架", 0.0, p_.y4(),
                              p_.row2_left_width, p_.shelf_row_depth,
                              true, false});
    shelf_blocks_.push_back({"行2右货架", row2_right_x, p_.y4(),
                              W - row2_right_x, p_.shelf_row_depth,
                              true, false});

    // ── 行3 (双车道，双排，深紫) ─────────────────────────────────────────────
    double row3_right_x = p_.row3_left_shelf + p_.row3_center_aisle;  // = 1.500
    shelf_blocks_.push_back({"行3左货架", 0.0, p_.y2(),
                              p_.row3_left_shelf, p_.shelf_row_depth,
                              true, false});
    shelf_blocks_.push_back({"行3右货架", row3_right_x, p_.y2(),
                              W - row3_right_x, p_.shelf_row_depth,
                              true, false});

    // ── 底货架 (单排，轻紫) ──────────────────────────────────────────────────
    shelf_blocks_.push_back({"底货架左", 0.0, 0.0,
                              p_.tb_shelf_width, p_.bottom_shelf_depth,
                              false, true});
    shelf_blocks_.push_back({"底货架右", p_.tb_shelf_width + tb_gap, 0.0,
                              p_.tb_shelf_width, p_.bottom_shelf_depth,
                              false, true});
}

void ForkliftMap::build_slots() {
    const double W   = p_.field_width;
    const double sl  = p_.vehicle_length;
    const double pre = p_.pre_dock_clearance;

    int global_id = 0;

    // ── 顶货架 row_id=0, heading=90° (north, entering from below) ──────────
    {
        const double shelf_bot = p_.y8(), shelf_top = p_.field_height;
        const double dock_y    = (shelf_bot + shelf_top) * 0.5;   // single-row: center
        const double pre_y     = shelf_bot - pre;
        const double theta     = kPi * 0.5;  // 90°: pointing north
        const double tb_gap    = W - 2.0 * p_.tb_shelf_width;
        for (int side = 0; side < 2; ++side) {
            double x0 = (side == 0) ? 0.0 : p_.tb_shelf_width + tb_gap;
            auto xs = col_centers(x0, p_.tb_shelf_width, 5);
            for (int c = 0; c < 5; ++c) {
                Slot s;
                s.id = global_id++; s.row_id = 0; s.col = side * 5 + c;
                s.cx = xs[c]; s.cy = dock_y;
                s.dock_theta = theta;
                s.pre_dock_x = xs[c]; s.pre_dock_y = pre_y;
                slots_.push_back(s);
            }
        }
    }

    // ── 行1 上排 row_id=1, heading=270° (south, entering from above) ────────
    // ── 行1 下排 row_id=2, heading=90°  (north, entering from below) ────────
    {
        const double shelf_bot = p_.y6(), shelf_top = p_.y7();
        // upper row: slot faces north (top of shelf, entered from 転弱区1)
        // lower row: slot faces south (bottom, entered from 転弱区2)
        double dock_y_upper = shelf_top - sl * 0.5;   // 3.506m
        double dock_y_lower = shelf_bot + sl * 0.5;   // 3.236m
        double pre_y_upper  = shelf_top + pre;         // 3.761m
        double pre_y_lower  = shelf_bot - pre;         // 2.980m

        // Main shelf: x=[0.580, 1.720], 5 cols. The right-edge mini shelf
        // (x=[2.300, 2.500]) is intentionally left as a solid obstacle with NO
        // parking slots: a 0.191 m-wide vehicle cannot maneuver out of a slot
        // only ~0.10 m from the right wall (any rotation clips the wall), so the
        // former slot20/21 there were permanent traps. The "行1小货架" block
        // still exists in shelf_blocks_, so it remains a rectangular obstacle.
        std::vector<double> all_xs =
            col_centers(p_.row1_left_aisle, p_.row1_shelf_width, 5);

        for (int c = 0; c < (int)all_xs.size(); ++c) {
            // upper
            {
                Slot s; s.id = global_id++; s.row_id = 1; s.col = c;
                s.cx = all_xs[c]; s.cy = dock_y_upper;
                s.dock_theta = -kPi * 0.5;  // 270°
                s.pre_dock_x = all_xs[c]; s.pre_dock_y = pre_y_upper;
                slots_.push_back(s);
            }
            // lower
            {
                Slot s; s.id = global_id++; s.row_id = 2; s.col = c;
                s.cx = all_xs[c]; s.cy = dock_y_lower;
                s.dock_theta = kPi * 0.5;  // 90°
                s.pre_dock_x = all_xs[c]; s.pre_dock_y = pre_y_lower;
                slots_.push_back(s);
            }
        }
    }

    // ── 行2 上排 row_id=3, heading=270° ─────────────────────────────────────
    // ── 行2 下排 row_id=4, heading=90°  ─────────────────────────────────────
    {
        const double shelf_bot = p_.y4(), shelf_top = p_.y5();
        double dock_y_upper = shelf_top - sl * 0.5;   // 2.385m
        double dock_y_lower = shelf_bot + sl * 0.5;   // 2.115m
        double pre_y_upper  = shelf_top + pre;         // 2.641m
        double pre_y_lower  = shelf_bot - pre;         // 1.859m

        double row2_right_x = p_.row2_left_width + p_.row2_gap;
        double row2_right_w = W - row2_right_x;  // 1.125m

        for (int side = 0; side < 2; ++side) {
            double x0 = (side == 0) ? 0.0 : row2_right_x;
            double sw_shelf = (side == 0) ? p_.row2_left_width : row2_right_w;
            auto xs = col_centers(x0, sw_shelf, 5);
            for (int c = 0; c < 5; ++c) {
                // upper
                {
                    Slot s; s.id = global_id++; s.row_id = 3; s.col = side * 5 + c;
                    s.cx = xs[c]; s.cy = dock_y_upper;
                    s.dock_theta = -kPi * 0.5;
                    s.pre_dock_x = xs[c]; s.pre_dock_y = pre_y_upper;
                    slots_.push_back(s);
                }
                // lower
                {
                    Slot s; s.id = global_id++; s.row_id = 4; s.col = side * 5 + c;
                    s.cx = xs[c]; s.cy = dock_y_lower;
                    s.dock_theta = kPi * 0.5;
                    s.pre_dock_x = xs[c]; s.pre_dock_y = pre_y_lower;
                    slots_.push_back(s);
                }
            }
        }
    }

    // ── 行3 上排 row_id=5, heading=270° ─────────────────────────────────────
    // ── 行3 下排 row_id=6, heading=90°  ─────────────────────────────────────
    {
        const double shelf_bot = p_.y2(), shelf_top = p_.y3();
        double dock_y_upper = shelf_top - sl * 0.5;   // 1.265m
        double dock_y_lower = shelf_bot + sl * 0.5;   // 0.994m
        double pre_y_upper  = shelf_top + pre;         // 1.521m
        double pre_y_lower  = shelf_bot - pre;         // 0.739m

        double row3_right_x = p_.row3_left_shelf + p_.row3_center_aisle;  // 1.500m
        double row3_right_w = W - row3_right_x;  // 1.000m

        for (int side = 0; side < 2; ++side) {
            double x0 = (side == 0) ? 0.0 : row3_right_x;
            double sw_shelf = (side == 0) ? p_.row3_left_shelf : row3_right_w;
            auto xs = col_centers(x0, sw_shelf, 4);
            for (int c = 0; c < 4; ++c) {
                // upper
                {
                    Slot s; s.id = global_id++; s.row_id = 5; s.col = side * 4 + c;
                    s.cx = xs[c]; s.cy = dock_y_upper;
                    s.dock_theta = -kPi * 0.5;
                    s.pre_dock_x = xs[c]; s.pre_dock_y = pre_y_upper;
                    slots_.push_back(s);
                }
                // lower
                {
                    Slot s; s.id = global_id++; s.row_id = 6; s.col = side * 4 + c;
                    s.cx = xs[c]; s.cy = dock_y_lower;
                    s.dock_theta = kPi * 0.5;
                    s.pre_dock_x = xs[c]; s.pre_dock_y = pre_y_lower;
                    slots_.push_back(s);
                }
            }
        }
    }

    // ── 底货架 row_id=7, heading=270° (south, entering from above) ──────────
    {
        const double shelf_bot = 0.0, shelf_top = p_.bottom_shelf_depth;
        const double dock_y    = (shelf_bot + shelf_top) * 0.5;   // 0.125m
        const double pre_y     = shelf_top + pre;                  // 0.400m
        const double theta     = -kPi * 0.5;  // 270°: pointing south
        const double tb_gap    = W - 2.0 * p_.tb_shelf_width;
        for (int side = 0; side < 2; ++side) {
            double x0 = (side == 0) ? 0.0 : p_.tb_shelf_width + tb_gap;
            auto xs = col_centers(x0, p_.tb_shelf_width, 5);
            for (int c = 0; c < 5; ++c) {
                Slot s;
                s.id = global_id++; s.row_id = 7; s.col = side * 5 + c;
                s.cx = xs[c]; s.cy = dock_y;
                s.dock_theta = theta;
                s.pre_dock_x = xs[c]; s.pre_dock_y = pre_y;
                slots_.push_back(s);
            }
        }
    }
    // Expected total: 10+12+20+16+10 = 68
}

void ForkliftMap::build_roads() {
    const double W = p_.field_width;

    // ── Horizontal corridor roads (4 corridors, full width) ──────────────────
    for (double cy : {p_.corridor1_cy(), p_.corridor2_cy(),
                      p_.corridor3_cy(), p_.corridor4_cy()}) {
        road_segs_.push_back({0.0, cy, W, cy, true});
    }

    // ── Vertical spine at x=1.25m (center of 0.25m gap) ─────────────────────
    double cx = p_.field_width * 0.5;

    // Segment above 行1 (through 顶货架 gap → down to 転弱区1 bottom)
    road_segs_.push_back({cx, p_.field_height, cx, p_.y7(), false});

    // Below 行1 through 行2 bottleneck into 転弱区3
    road_segs_.push_back({cx, p_.y6(), cx, p_.y3(), false});

    // Through 行3 center aisle down into 転弱区4 and bottom exit
    road_segs_.push_back({cx, p_.y3(), cx, 0.0, false});

    // ── 行1 bypass: left aisle x=0.29m ───────────────────────────────────────
    double left_bypass_cx = p_.row1_left_aisle * 0.5;          // 0.290m
    road_segs_.push_back({left_bypass_cx, p_.y6(), left_bypass_cx, p_.y7(), false});

    // ── 行1 bypass: right aisle x=2.01m ──────────────────────────────────────
    // right aisle = [row1_shelf_right, mini_shelf_left]
    double rs_right   = p_.row1_left_aisle + p_.row1_shelf_width;  // 1.720
    double mini_left  = W - p_.row1_mini_shelf;                     // 2.300
    double right_bypass_cx = (rs_right + mini_left) * 0.5;         // 2.010m
    road_segs_.push_back({right_bypass_cx, p_.y6(), right_bypass_cx, p_.y7(), false});
}
