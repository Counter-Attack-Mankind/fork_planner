#pragma once

#include <casadi/casadi.hpp>

namespace forklift_nlp_recovery {

casadi::MX bicycleDerivative(const casadi::MX& state,
                             const casadi::MX& control, double wheel_base);
casadi::MX integrateBicycleEuler(const casadi::MX& state,
                                 const casadi::MX& control,
                                 const casadi::MX& duration, double wheel_base);
casadi::MX symbolicBodyCenter(const casadi::MX& state,
                              double rear_axle_to_center);

}  // namespace forklift_nlp_recovery
