#pragma once
#include <string>
#include <cmath>

// Single parking slot (货位)
struct Slot {
    int id     = -1;   // global 0~67
    int row_id = -1;   // 0=顶货架 1=行1上 2=行1下 3=行2上 4=行2下 5=行3上 6=行3下 7=底货架
    int col    = -1;   // column index within this row (0-based)

    double cx = 0.0, cy = 0.0;          // slot center (world frame)
    double dock_theta = 0.0;             // heading when parked (rad):  +π/2=north  -π/2=south

    double pre_dock_x = 0.0;            // pre-dock position center
    double pre_dock_y = 0.0;            // (theta same as dock_theta)

    bool occupied = false;

    double dock_x() const { return cx; }
    double dock_y() const { return cy; }
};

// Rectangular shelf block (货架矩形障碍物)
struct ShelfBlock {
    std::string name;
    double x = 0.0, y = 0.0;   // bottom-left corner (world frame)
    double w = 0.0, h = 0.0;   // width (X), height (Y)
    bool is_double_row   = true;
    bool is_light_purple = false;  // true=单排(顶/底), false=双排(行1/2/3)

    double cx() const { return x + w * 0.5; }
    double cy() const { return y + h * 0.5; }
    double x_max() const { return x + w; }
    double y_max() const { return y + h; }
};

// Road center-line segment (道路中心线段)
struct RoadSegment {
    double x0 = 0, y0 = 0;
    double x1 = 0, y1 = 0;
    bool is_horizontal = true;   // true=E-W, false=N-S
};
