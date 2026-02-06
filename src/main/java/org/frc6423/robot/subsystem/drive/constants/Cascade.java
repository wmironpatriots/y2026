// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import org.frc6423.robot.Constants.Matrix;

/** {@link RebuiltL1} but for usage on cascade (just has modified gear ratios tbh) */
public class Cascade extends RebuiltL1 {
  @Override
  public LinearVelocity getMaxLinearVelocity() {
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    return FeetPerSecond.of(17.1);
  }

  @Override
  public LinearAcceleration getMaxLinearAcceleration() {
    // TODO derive in choreo; values copied from L2 ratio
    return MetersPerSecondPerSecond.of(12.624);
  }

  @Override
  public AngularAcceleration getAngularAcceleration() {
    // TODO derive in choreo; values copied from L2 ratio
    return RadiansPerSecondPerSecond.of(50.022);
  }

  @Override
  public double getDriveRotorToMechRatio() {
    return (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    // TODO derive offsets
    return new ModuleConfig[] {
      new ModuleConfig(
          "FR",
          getCANBus(),
          5,
          6,
          11,
          getPivotServoConfig(Matrix.kDriveFrEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(0.01)),
          getWheelRadius()),
      new ModuleConfig(
          "FL",
          getCANBus(),
          3,
          4,
          10,
          getPivotServoConfig(Matrix.kDriveFlEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(-0.16)),
          getWheelRadius()),
      new ModuleConfig(
          "BL",
          getCANBus(),
          1,
          2,
          9,
          getPivotServoConfig(Matrix.kDriveBlEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(0.36)),
          getWheelRadius()),
      new ModuleConfig(
          "BR",
          getCANBus(),
          7,
          8,
          12,
          getPivotServoConfig(Matrix.kDriveBrEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(-0.26)),
          getWheelRadius())
    };
  }
}
