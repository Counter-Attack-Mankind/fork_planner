#pragma once
#include <vector>
#include <cmath>
#include "forklift_map/map_param.h"
#include "forklift_map/map_types.h"

class ForkliftMap {
public:
    explicit ForkliftMap(const MapParam& p);

    // Accessors (read-only views for planning / visualization)
    const std::vector<Slot>&       slots()        const { return slots_; }
    const std::vector<ShelfBlock>& shelf_blocks() const { return shelf_blocks_; }
    const std::vector<RoadSegment>& road_segments() const { return road_segs_; }
    const MapParam&                param()         const { return p_; }

    // Mutable slot access (planning updates occupancy)
    Slot& slot(int id) { return slots_.at(id); }
    void  set_occupied(int id, bool occ) { slots_.at(id).occupied = occ; }

    std::vector<Slot> free_slots() const;

private:
    void build();
    void build_shelves();
    void build_slots();
    void build_roads();

    // Helper: generate slot columns for a shelf X-range
    // Slots uniformly distributed: gap = (shelf_w - n*slot_w) / (n+1)
    std::vector<double> col_centers(double shelf_x_min, double shelf_w, int n_cols) const;

    MapParam p_;
    std::vector<Slot>        slots_;
    std::vector<ShelfBlock>  shelf_blocks_;
    std::vector<RoadSegment> road_segs_;
};
