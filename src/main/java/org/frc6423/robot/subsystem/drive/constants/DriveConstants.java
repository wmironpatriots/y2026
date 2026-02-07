// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
        getMaxLinearVelocity().in(InchesPerSecond) / getTrackRadius().in(Inches));
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
   * @return true if pivot positive direction should be inverted
   */
  public abstract boolean getPivotInverted();

  /**
   * @return gear ratio between drive servo rotor to mechanism
   */
  public abstract double getDriveRotorToMechRatio();

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
   * @return {@link SwerveDriveKinematics} representing the kinematics solver for the drivetrain
   */
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(getModuleDisplacements());
  }

  /**
   * @return {@link CANBus} representing CANbus devices are on
   */
  public CANBus getCANBus() {
    return CANBus.roboRIO();
  }

  /**
   * @return {@link Pigeon2Configuration} represent pigeon2 config for gyro
   */
  public abstract Pigeon2Configuration getPigeon2Config();

  /**
   * @return {@link TalonFXConfiguration} representing servo config for swerve module pivot
   */
  public abstract TalonFXConfiguration getPivotServoConfig(int CANcoderId);

  /**
   * @return {@link TalonFXConfiguration} representing servo config for swerve module drive
   */
  public abstract TalonFXConfiguration getDriveServoConfig();

  /**
   * @return the torque constant of the drive motor through its gearbox
   */
  public double getDriveGearboxKt() {
    return getDriveRotorToMechRatio() / getDriveMotorKt();
  }

  /**
   * @return the torque constant of the drive motor
   */
  public abstract double getDriveMotorKt();

  /**
   * @param angularOffset {@link Angle} representing the angular position offset of encoder
   * @return {@link CANcoderConfiguration} representing CANcoder config for swerve module encoder
   */
  public abstract CANcoderConfiguration getCANcoderConfig(Angle angularOffset);

  /**
   * @return {@link GyroConfig} representing config of gyro
   */
  public abstract GyroConfig getGyroConfig();

  /**
   * @return {@link ModuleConfig} array representing the configs of swerve modules (FR, FL, BL, BR)
   */
  public abstract ModuleConfig[] getModuleConfigs();

  /**
   * Represents a configuration for a {@link GyroIO}
   *
   * @param canBus {@link CANBus} representing the CANbus loop hardware is in
   * @param deviceId the CAN device ID of gyro
   * @param config {@link Pigeon2Configuration} representing the pigeon2 config for gyro
   */
  public static record GyroConfig(CANBus canBus, int deviceId, Pigeon2Configuration config) {}

  /**
   * Represents a configuration for a {@link SwerveModuleIO}
   *
   * @param name {@link String} nickname for module
   * @param canBus {@link CANBus} representing the CANbus loop hardware is attached to
   * @param pivotDeviceId the CAN device ID of the pivot servo
   * @param driveDeviceId the CAN device ID of the drive servo
   * @param cancoderId the CAN device ID of the encoder
   * @param pivotConfig {@link TalonFXConfiguration} representing servo config for pivot
   * @param driveConfig {@link TalonFXConfiguration} representing servo config for drive
   * @param cancoderConfig {@link CANcoderConfiguration} representing cancoder config for encoder
   */
  public static record ModuleConfig(
      String name,
      CANBus canBus,
      int pivotDeviceId,
      int driveDeviceId,
      int cancoderId,
      TalonFXConfiguration pivotConfig,
      TalonFXConfiguration driveConfig,
      CANcoderConfiguration cancoderConfig) {}
}
