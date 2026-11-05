#include "forklift_nlp_recovery/common/kinematic_model.h"

namespace forklift_nlp_recovery {

casadi::MX bicycleDerivative(const casadi::MX& state,
                             const casadi::MX& control, double wheel_base) {
    return casadi::MX::vertcat({state(3) * cos(state(2)),
                               state(3) * sin(state(2)),
                               state(3) * tan(state(4)) / wheel_base,
                               control(0), control(1)});
}

casadi::MX integrateBicycleEuler(const casadi::MX& state,
                                 const casadi::MX& control,
                                 const casadi::MX& duration, double wheel_base) {
    return state + duration * bicycleDerivative(state, control, wheel_base);
}

casadi::MX symbolicBodyCenter(const casadi::MX& state,
                              double rear_axle_to_center) {
    return casadi::MX::vertcat({
        state(0) + rear_axle_to_center * cos(state(2)),
        state(1) + rear_axle_to_center * sin(state(2))});
}

}  // namespace forklift_nlp_recovery
