#include "forklift_nlp_recovery/cooperative/dynamic_obca.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace forklift_nlp_recovery {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Vector2 { double x = 0.0; double y = 0.0; };
struct DualGuess { std::array<double, 4> lambda{}; std::array<double, 4> mu{}; };

double dot(const Vector2& a, const Vector2& b) { return a.x * b.x + a.y * b.y; }

Pose2d center(const Pose2d& rear, double offset) {
    return {rear.x + offset * std::cos(rear.yaw),
            rear.y + offset * std::sin(rear.yaw), rear.yaw};
}

double radius(double half_length, double half_width, const Vector2& longitudinal,
              const Vector2& lateral, const Vector2& axis) {
    return half_length * std::abs(dot(longitudinal, axis)) +
           half_width * std::abs(dot(lateral, axis));
}

std::array<double, 4> coefficients(const Vector2& axis,
                                   const Vector2& longitudinal,
                                   const Vector2& lateral) {
    const double x = dot(axis, longitudinal);
    const double y = dot(axis, lateral);
    return {std::max(0.0, x), std::max(0.0, -x),
            std::max(0.0, y), std::max(0.0, -y)};
}

DualGuess initialDual(const Pose2d& first_rear, const Pose2d& second_rear,
                      const VehicleParameters& first,
                      const VehicleParameters& second) {
    const Pose2d first_center = center(first_rear, first.rear_axle_to_center);
    const Pose2d second_center = center(second_rear, second.rear_axle_to_center);
    const Vector2 first_x{std::cos(first_rear.yaw), std::sin(first_rear.yaw)};
    const Vector2 first_y{-first_x.y, first_x.x};
    const Vector2 second_x{std::cos(second_rear.yaw), std::sin(second_rear.yaw)};
    const Vector2 second_y{-second_x.y, second_x.x};
    const Vector2 delta{first_center.x - second_center.x,
                        first_center.y - second_center.y};
    const std::array<Vector2, 8> axes{{first_x, {-first_x.x, -first_x.y},
        first_y, {-first_y.x, -first_y.y}, second_x, {-second_x.x, -second_x.y},
        second_y, {-second_y.x, -second_y.y}}};
    Vector2 normal = axes.front();
    double best = -kInfinity;
    for (const Vector2& axis : axes) {
        const double separation = dot(delta, axis) -
            radius(0.5 * first.length, 0.5 * first.width, first_x, first_y, axis) -
            radius(0.5 * second.length, 0.5 * second.width, second_x, second_y, axis);
        if (separation > best) { best = separation; normal = axis; }
    }
    DualGuess guess;
    guess.lambda = coefficients(normal, second_x, second_y);
    const Vector2 local{dot(normal, first_x), dot(normal, first_y)};
    guess.mu = coefficients({-local.x, -local.y}, {1.0, 0.0}, {0.0, 1.0});
    return guess;
}

casadi::MX symbolicCenter(const casadi::MX& state, double offset) {
    return casadi::MX::vertcat({state(0) + offset * cos(state(2)),
                               state(1) + offset * sin(state(2))});
}

}  // namespace

void addDynamicObca(
    const casadi::MX& first_state, const casadi::MX& second_state,
    const Pose2d& initial_first_rear, const Pose2d& initial_second_rear,
    const VehicleParameters& first, const VehicleParameters& second,
    int first_id, int second_id, size_t node, const std::string& sample_name,
    NlpConstraintCategory category, std::vector<casadi::MX>& variables,
    std::vector<double>& initial_values, std::vector<double>& lower_bounds,
    std::vector<double>& upper_bounds,
    std::vector<NlpConstraintMetadata>& variable_metadata,
    std::vector<casadi::MX>& constraint_expressions,
    std::vector<double>& constraint_lower, std::vector<double>& constraint_upper,
    std::vector<NlpConstraintMetadata>& constraint_metadata) {
    const std::string suffix = "_V" + std::to_string(first_id) + "_V" +
        std::to_string(second_id) + "_N" + std::to_string(node) + "_" + sample_name;
    const casadi::MX lambda = casadi::MX::sym("obca_lambda" + suffix, 4);
    const casadi::MX mu = casadi::MX::sym("obca_mu" + suffix, 4);
    variables.push_back(lambda);
    variables.push_back(mu);
    const DualGuess guess = initialDual(initial_first_rear, initial_second_rear,
                                        first, second);
    for (size_t i = 0; i < 4; ++i) {
        initial_values.push_back(guess.lambda[i]);
        lower_bounds.push_back(0.0);
        upper_bounds.push_back(kInfinity);
        variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
            first_id, second_id, node, sample_name + "_lambda_" + std::to_string(i)});
    }
    for (size_t i = 0; i < 4; ++i) {
        initial_values.push_back(guess.mu[i]);
        lower_bounds.push_back(0.0);
        upper_bounds.push_back(kInfinity);
        variable_metadata.push_back({NlpConstraintCategory::VARIABLE_BOUND,
            first_id, second_id, node, sample_name + "_mu_" + std::to_string(i)});
    }

    const casadi::MX first_center = symbolicCenter(first_state, first.rear_axle_to_center);
    const casadi::MX second_center = symbolicCenter(second_state, second.rear_axle_to_center);
    const casadi::MX second_cos = cos(second_state(2));
    const casadi::MX second_sin = sin(second_state(2));
    const casadi::MX normal_x = second_cos * (lambda(0) - lambda(1)) -
                                second_sin * (lambda(2) - lambda(3));
    const casadi::MX normal_y = second_sin * (lambda(0) - lambda(1)) +
                                second_cos * (lambda(2) - lambda(3));
    const casadi::MX first_cos = cos(first_state(2));
    const casadi::MX first_sin = sin(first_state(2));
    const casadi::MX local_x = first_cos * normal_x + first_sin * normal_y;
    const casadi::MX local_y = -first_sin * normal_x + first_cos * normal_y;
    const auto add = [&](const casadi::MX& expression, double lower, double upper,
                         const std::string& component) {
        constraint_expressions.push_back(expression);
        constraint_lower.push_back(lower);
        constraint_upper.push_back(upper);
        constraint_metadata.push_back({category, first_id, second_id, node,
                                       sample_name + "_" + component});
    };
    add(mu(0) - mu(1) + local_x, 0.0, 0.0, "stationarity_x");
    add(mu(2) - mu(3) + local_y, 0.0, 0.0, "stationarity_y");
    add(normal_x * normal_x + normal_y * normal_y, 1.0, 1.0, "normalization");

    const casadi::MX delta_x = first_center(0) - second_center(0);
    const casadi::MX delta_y = first_center(1) - second_center(1);
    const casadi::MX separation = normal_x * delta_x + normal_y * delta_y -
        0.5 * second.length * (lambda(0) + lambda(1)) -
        0.5 * second.width * (lambda(2) + lambda(3)) -
        0.5 * first.length * (mu(0) + mu(1)) -
        0.5 * first.width * (mu(2) + mu(3));
    add(separation, 0.0, kInfinity, "separation");
}

}  // namespace forklift_nlp_recovery
