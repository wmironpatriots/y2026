// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
//  FRC 2026 Look Up Table (LUT) Generator
//  This program generates a LUT with inputs of r (distance from hub)
//  and outputs of v_0 (initial velocity), pitch, & yaw for a fuel
//  gamepiece to land in the hub while minimizing initial velocity
//
//  To customize the output table, user may configure the following
//  top constants: 
//      - The maximum distance from the hub to calculate for (R)
//      - The amount of steps within the max distance to calculate points for (dR)
//
//  All units are meters, mps, mpsps, & kg.
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include <cmath>
#include <numbers>
#include <print>

#include <Eigen/Core>
#include <sleipnir/optimization/problem.hpp>

// ? ~~~~~~~~~~~~~ PARAMETERS ~~~~~~~~~~~~~

/* The range from 0.0 to calculate for */
constexpr double R = 182.11 * 0.0254;

/* The step size for points */
constexpr double dR = R / 10;

// * ~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~

/* An overapproximation circle  */
constexpr double kTargetEffectiveOpeningRadius = 47.0 / 2.0 * 0.0254;
constexpr double kTargetEffectiveOpeningHeight = 72.0 * 0.0254;

// * PROJECTILE CHARACTERISTICS
/* The mass of fuel in kilograms */
constexpr double kFuelMass = 10.0;

/* The Density of a fuel */
constexpr double kFuelRho = 128.17; // TODO

// * FORCES
/** Vector of acceleration caused by gravity */
constexpr auto kGravitationalAcceleration = Eigen::Vector3d{
    {0.0}, 
    {0.0}, 
    {9.806}};

slp::VariableMatrix<double> cross(const slp::VariableMatrix<double>& a,
                                  const slp::VariableMatrix<double>& b) {
  return slp::VariableMatrix<double>({{a[1, 0] * b[2, 0] - a[2, 0] * b[1, 0]},
                                      {a[2, 0] * b[0, 0] - a[0, 0] * b[2, 0]},
                                      {a[0, 0] * b[1, 0] - a[1, 0] * b[0, 0]}});
}

slp::VariableMatrix<double> function(const slp::VariableMatrix<double>& x) {
    using namespace slp::slicing;

    // r' = r'
    // z' = z'
    //
    // [r", z"] = [0, -g] - (F_D(v)/m v_hat - F_L(v)/m (omega * v)

    // rho is the fluid density in kg/(m^3)
    // v is the linear velocity in meters/sec
    // omega is the angular velocity in rads/sec
    // A is the cross-sectional area of a circle in m^2
    // m is the projectile mass in kg
    auto v = x[slp::Slice{2, 4}, _];
    slp::Variable v2 = v.T() * v;
    auto v_norm = sqrt(v2);
    auto v_hat = v / v_norm;
    constexpr Eigen::Vector3d omega{{0.0}, {-2.0}, {0.0}};
    constexpr double r = 0.15;
    constexpr double A = std::numbers::pi * r * r;
    constexpr double m = 0.283;

    // Per https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation:
    //   F_D(v) = ½ρ|v|²C_D A
    //   C_D is the drag coefficient (dimensionless)
    constexpr double C_D = 0.5;
    auto F_D = 0.5 * kFuelRho * v2 * C_D * A;

    // Magnus force:
    //   F_L(v) = ½ρ|v|C_L A
    //   C_L is the lift coefficient (dimensionless)
    constexpr double C_L = 0.5;
    auto F_L = 0.5 * kFuelRho * v_norm * C_L * A;

    return slp::block<double>(
        {{v}, {-kGravitationalAcceleration - F_D / m * v_hat - F_L / m * cross(v, omega)}});
}

using Vector2d = Eigen::Vector2d;

int main() {
    constexpr Vector2d robot_wrt_hub{{0.0}, {0.0}};
    constexpr double max_initial_velocity = 10.0;

    // 20 in up 6 in forward
    Vector2d shooter_wrt_robot{6.0, 20.0};
    Vector2d shooter_wrt_field = robot_wrt_hub + shooter_wrt_robot;

    return 0;

    // 24.6 height 8.3 backwards
}
