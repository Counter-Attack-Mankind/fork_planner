#include "forklift_nlp_recovery/common/kinematic_model.h"

namespace forklift_nlp_recovery {

casadi::MX bicycleDerivative(const casadi::MX& state,
                             const casadi::MX& control, double wheel_base) {
    return casadi::MX::vertcat({state(3) * cos(state(2)),
                               state(3) * sin(state(2)),
                               state(3) * tan(state(4)) / wheel_base,
                               control(0), control(1)});
}

casadi::MX integrateBicycleRk4(const casadi::MX& state,
                               const casadi::MX& control,
                               const casadi::MX& duration, double wheel_base) {
    const casadi::MX k1 = bicycleDerivative(state, control, wheel_base);
    const casadi::MX k2 = bicycleDerivative(state + duration * 0.5 * k1,
                                             control, wheel_base);
    const casadi::MX k3 = bicycleDerivative(state + duration * 0.5 * k2,
                                             control, wheel_base);
    const casadi::MX k4 = bicycleDerivative(state + duration * k3,
                                             control, wheel_base);
    return state + duration * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
}

casadi::MX symbolicBodyCenter(const casadi::MX& state,
                              double rear_axle_to_center) {
    return casadi::MX::vertcat({
        state(0) + rear_axle_to_center * cos(state(2)),
        state(1) + rear_axle_to_center * sin(state(2))});
}

}  // namespace forklift_nlp_recovery
