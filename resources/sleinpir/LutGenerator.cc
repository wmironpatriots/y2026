// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include <cmath>
#include <numbers>
#include <print>

#include <Eigen/Core>
#include <sleipnir/autodiff/variable_matrix.hpp>
#include <sleipnir/optimization/problem.hpp>

/*
    FRC 2026 Look Up Table (LUT) Generator

    This program generates a LUT with inputs of r (distance from hub)
    and outputs of v_0 (initial velocity), pitch, & yaw for a fuel
    gamepiece to land in the hub while minimizing initial velocity

    To customize the output table, user may configure the following
    top constants: 
        - The maximum distance from the hub to calculate for (R)
        - The amount of steps within the max distance to calculate points for (dR)

    All units are meters, mps, mpsps, & kg.
*/

// ? PARAMETERS
constexpr double R = 182.11 * 0.0254;
constexpr double dR = R / 10;

// * TARGET CHARACTERISTICS
constexpr double kTargetEffectiveOpeningRadius = 47.0 / 2.0 * 0.0254;
constexpr double kTargetEffectiveOpeningHeight = 72.0 * 0.0254;

// * PROJECTILE CHARACTERISTICS
/* The radius of fuel in meters */
constexpr double kFuelRadius = 5.91 / 2 * 0.0254;
/* The mass of fuel in meters */
constexpr double kFuelMass = 10.0;

/* 
    The Density of a fuel 
    V = (4.0 / 3.0) * pi * radius^2
    rho = m / V
*/
// constexpr double kFuelRho = kFuelMass / ((4.0 / 3.0) * M_PI * kFuelRadius * kFuelRadius * kFuelRadius);
constexpr double kFuelRho = 1.1; // TODO

// * FORCES
/** Vector of acceleration caused by gravity */
constexpr auto kGravitationalAcceleration = Eigen::Vector3d{
    {0.0}, 
    {0.0}, 
    {9.806}};
