// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include <cmath>
#include <numbers>
#include <print>

#include <Eigen/Core>
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
*/

using Eigen::Vector3d;