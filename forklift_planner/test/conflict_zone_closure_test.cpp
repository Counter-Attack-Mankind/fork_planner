#include "forklift_planner/multi_vehicle/conflict_zone_closure.h"

#include <cmath>
#include <iostream>
#include <vector>

using forklift_planner::multi_vehicle::insertConflictZoneWithClosure;

namespace {

struct IntervalZone {
    double self_enter;
    double self_exit;
    double other_enter;
    double other_exit;
};

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

int fail(const char* message) {
    std::cerr << "conflict_zone_closure_test: " << message << '\n';
    return 1;
}

bool touches(const IntervalZone& lhs, const IntervalZone& rhs) {
    constexpr double gap = 0.0;
    return lhs.self_enter <= rhs.self_exit + gap &&
           lhs.self_exit + gap >= rhs.self_enter &&
           lhs.other_enter <= rhs.other_exit + gap &&
           lhs.other_exit + gap >= rhs.other_enter;
}

void merge(IntervalZone& destination, const IntervalZone& source) {
    destination.self_enter = std::min(destination.self_enter, source.self_enter);
    destination.self_exit = std::max(destination.self_exit, source.self_exit);
    destination.other_enter =
        std::min(destination.other_enter, source.other_enter);
    destination.other_exit =
        std::max(destination.other_exit, source.other_exit);
}

}  // namespace

int main() {
    // A and B are initially separate.  The bridge touches both at once.
    std::vector<IntervalZone> simultaneous{{0.0, 1.0, 0.0, 1.0},
                                           {0.0, 1.0, 2.0, 3.0}};
    insertConflictZoneWithClosure({0.5, 1.5, 0.5, 2.5}, simultaneous,
                                  touches, merge);
    if (simultaneous.size() != 1) {
        return fail("simultaneous bridge did not produce one component");
    }
    const IntervalZone& united = simultaneous.front();
    if (!near(united.self_enter, 0.0) || !near(united.self_exit, 1.5) ||
        !near(united.other_enter, 0.0) || !near(united.other_exit, 3.0)) {
        return fail("simultaneous bridge interval union is wrong");
    }

    // A is rejected first.  After candidate absorbs B, it expands far enough
    // to touch A, which requires another pass over the existing components.
    std::vector<IntervalZone> rescan{{0.0, 1.0, 0.0, 1.1},
                                    {0.0, 1.0, 1.0, 2.0}};
    insertConflictZoneWithClosure({0.5, 1.5, 1.9, 2.1}, rescan,
                                  touches, merge);
    if (rescan.size() != 1 || !near(rescan.front().other_enter, 0.0) ||
        !near(rescan.front().other_exit, 2.1)) {
        return fail("fixed-point rescan did not close transitive component");
    }

    std::cout << "conflict_zone_closure_test: PASS\n";
    return 0;
}
