#include "forklift_planner/path_generator.h"

RoughPath PathGenerator::generate(const Slot& src, const Slot& tgt,
                                  PathGenerationInfo* info) const {
    switch (route_mode_) {
        case PathGeneratorRouteMode::A1_TO_B:
            return generateRouteA1ToB(src, tgt, info);
        case PathGeneratorRouteMode::B_TO_A1:
            return generateRouteBToA1(src, tgt, info);
        case PathGeneratorRouteMode::A2_TO_B:
            return generateRouteA2ToB(src, tgt, info);
        case PathGeneratorRouteMode::B_TO_A2:
            return generateRouteBToA2(src, tgt, info);
        case PathGeneratorRouteMode::AUTO:
            return generateRouteCommon(src, tgt, info);
    }
    return generateRouteCommon(src, tgt, info);
}
