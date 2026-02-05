// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.constants;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/** Represents a configuration for a swerve drivetrain */
public abstract class DriveConstants {
  /**
   * @return {@link Distance} representing the width between the centers of swerve module wheels
   *     along side
   */
  public abstract Distance getTrackWidth();

  /**
   * @return {@link Distance} representing the width between the center of chassis and a swerve
   *     module wheel center
   */
  public Distance getTrackRadius() {
    var width = getTrackWidth();
    return Inches.of(Math.hypot(width.in(Inches), width.in(Inches)));
  }

  /**
   * @return {@link Distance} representing the thickness of the bumpers
   */
  public abstract Distance getBumperThickness();

  /**
   * @return {@link LinearVelocity} representing the maximum possible velocity of drivetrain
   */
  public abstract LinearVelocity getMaxLinearVelocity();

  /**
   * @return {@link LinearAcceleration} representing the maximum possible acceleration of drivetrain
   *     <p><strong> NOTE </strong> - You can calculate this with choreo
   */
  public abstract LinearAcceleration getMaxLinearAcceleration();

  /**
   * @return {@link AngularVelocity} representing the maximum possible velocity of drivetrain
   */
  public AngularVelocity getMaxAngularVelocity() {
    return RadiansPerSecond.of(
        getMaxLinearVelocity().in(FeetPerSecond) / getTrackRadius().in(Inches));
  }

  /**
   * @return {@link AngularAcceleration} representing the maximum possible acceleration drivetrain
   *     <p><strong> NOTE </strong> - You can calculate this with choreo
   */
  public abstract AngularAcceleration getAngularAcceleration();

  /**
   * @return {@link Mass} representing the mass of the entire robot
   */
  public abstract Mass getMass();

  /**
   * @return {@link MomentOfInertia} representing the Rotational Inertia of drivetrain
   */
  public abstract MomentOfInertia getRotationalInertia();

  /**
   * @return gear ratio between pivot servo rotor to pivot encoder
   */
  public abstract double getPivotRotorToSensorRatio();

  /**
   * @return gear ratio between pivot encoder to mechanism output
   */
  public abstract double getPivotSensorToMechanismRatio();

  /**
   * @return {@link Distance} representing the radius of the swerve module wheel
   */
  public abstract Distance getWheelRadius();

  /**
   * @return {@link Translation2d} array representing the displacement of modules wrt to center of
   *     chassis (FR, FL, BL, BR)
   */
  public Translation2d[] getModuleDisplacements() {
    var coord = getTrackWidth().div(2);
    return new Translation2d[] {
      new Translation2d(coord, coord),
      new Translation2d(coord.times(-1), coord),
      new Translation2d(coord, coord).times(-1),
      new Translation2d(coord, coord.times(-1))
    };
  }

  /**
   * @return {@link TalonFXConfiguration} representing servo config for swerve module pivot
   */
  public abstract TalonFXConfiguration getPivotServoConfig(int CANcoderId);

  /**
   * @return {@link TalonFXConfiguration} representing servo config for swerve module drive
   */
  public abstract TalonFXConfiguration getDriveServoConfig();

  /**
   * @return {@link CANcoderConfiguration} representing CANcoder config for swerve module encoder
   */
  public abstract CANcoderConfiguration getCANcoderConfig();

  /**
   * @return {@link ModuleConfig} array representing the configs of swerve modules (FR, FL, BL, BR)
   */
  public abstract ModuleConfig[] getModuleConfigs();

  /**
   * Represents a configuration for a {@link SwerveModule}
   *
   * @param name {@link String} nickname for module
   * @param canBus {@link CANBus} representing the CANbus loop hardware is attached to
   * @param pivotDeviceId the CAN device ID of the pivot servo
   * @param driveDeviceId the CAN device ID of the drive servo
   * @param cancoderId the CAN device ID of the encoder
   * @param cancoderOffset {@link Angle} representing the angular position offset of encoder
   */
  public static record ModuleConfig(
      String name,
      CANBus canBus,
      int pivotDeviceId,
      int driveDeviceId,
      int cancoderId,
      Angle cancoderOffset) {}
}
