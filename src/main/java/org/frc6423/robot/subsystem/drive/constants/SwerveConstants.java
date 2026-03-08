// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Configuration for a swerve drivetrain */
public abstract class SwerveConstants {
  /**
   * Get distance from wheel center to wheel center
   *
   * @return {@link Double}
   */
  public abstract double getTrackWidthMeters();

  /**
   * Get distance from wheel center to chassis center
   *
   * @return {@link Double}
   */
  public double getTrackRadiusMeters() {
    var width = getTrackWidthMeters();
    return Math.hypot(width, width);
  }

  /**
   * Get distance between frame perimeter to bumper perimeter
   *
   * @return {@link Double}
   */
  public abstract double getBumperThicknessInches();

  /**
   * Get maximum linear velocity of drivetrain
   *
   * @return {@link Double}
   */
  public abstract double getMaxLinearVelocityMetersPerSecond();

  /**
   * Get maximum linear acceleration of drivetrain
   *
   * @return {@link Double}
   */
  public abstract double getMaxLinearAccelerationMetersPerSecondPerSecond();

  /**
   * Get maximum magnitude of {@link #getMaxLinearVelocity()} before toggling foc
   *
   * @return {@link Double}
   */
  public abstract double getFocAutoToggleMagnitude();

  /**
   * Get maximum angular rate
   *
   * @return {@link Double}
   */
  public double getMaxAngularVelocityRadsPerSec() {
    return getMaxLinearVelocityMetersPerSecond() / getTrackRadiusMeters();
  }

  /**
   * Get maximum rate of angualar rate
   *
   * @return {@link Double}
   */
  public abstract double getAngularAccelerationRadsPerSecPerSec();

  /**
   * Get mass of drivetrain
   *
   * @return {@link Double}
   */
  public abstract double getMassKg();

  /**
   * Get rotational inertia of drivetrain when spinning around its center
   *
   * @return {@link Double}
   */
  public abstract double getRotationalInertiaKgSquaredMeters();

  /**
   * Gear ratio between pivot servo rotor and abs encoder
   *
   * @return {@link Double}
   */
  public abstract double getPivotRotorToSensorRatio();

  /**
   * Get ratio between abs encoder and pivot mechanism
   *
   * @return {@link Double}
   */
  public abstract double getPivotSensorToMechanismRatio();

  /**
   * Check if pivot motor should be inverted
   *
   * @return {@link Double}
   */
  public abstract boolean getPivotInverted();

  /**
   * Get ratio between drive servo rotor and drive mechanism
   *
   * @return {@link Double}
   */
  public abstract double getDriveRotorToMechRatio();

  /**
   * Get displacements of wheel centers from center of chassis
   *
   * <p>FR, BR, FL, BL
   *
   * @return {@link Array} of {@link Translation2d}
   */
  public Translation2d[] getModuleDisplacements() {
    var coord = getTrackWidthMeters() / 2;
    return new Translation2d[] {
      new Translation2d(coord, coord),
      new Translation2d(coord, -coord),
      new Translation2d(-coord, coord),
      new Translation2d(-coord, -coord),
    };
  }

  /**
   * Get {@link SwerveDriveKinematics} for drivetrain arrangement
   *
   * <p>FR, BR, FL, BL
   *
   * @return {@link SwerveDriveKinematics}
   */
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(getModuleDisplacements());
  }

  /**
   * Get CAN bus devices are connected to
   *
   * @return {@link CANBus}
   */
  public CANBus getCANBus() {
    return CANBus.roboRIO();
  }

  /**
   * Get hardware configuration of gyro
   *
   * @return {@link Pigeon2}
   */
  public abstract Pigeon2Configuration getPigeon2Config();

  /**
   * Get hardware configuration of pivot servo
   *
   * @param CANcoderId {@link Integer} CAN Device ID of abs encoder of pivot mechanism
   * @return {@link TalonFXConfiguration}
   */
  public abstract TalonFXConfiguration getPivotServoConfig(int CANcoderId);

  /**
   * Get hardware configuration of drive servo
   *
   * @return {@link TalonFXConfiguration}
   */
  public abstract TalonFXConfiguration getDriveServoConfig();

  /**
   * Get Torque gain of drive servo through gearbox
   *
   * @return {@link Double}
   */
  public double getDriveGearboxKt() {
    return getDriveRotorToMechRatio() / getDriveMotorKt();
  }

  /**
   * Get Torque gain of drive servo
   *
   * @return {@link Double}
   */
  public abstract double getDriveMotorKt();

  /**
   * Get radius of wheels
   *
   * @return {@link Double}
   */
  public abstract double getWheelRadiusMeters();

  /**
   * Get hardware configuration of pivot abs encoder
   *
   * @param angularOffsetRevs {@link Double} ABS offset
   * @return {@link CANcoderConfiguration}
   */
  public abstract CANcoderConfiguration getCANcoderConfig(double angularOffsetRevs);

  /**
   * Get configuration for gyro
   *
   * @return {@link GyroConfig}
   */
  public abstract GyroConfig getGyroConfig();

  /**
   * Get configuration for front right module
   *
   * @return {@link ModuleConfig}
   */
  public abstract ModuleConfig getFrontRightModuleConfig();

  /**
   * Get configuration for back right module
   *
   * @return {@link ModuleConfig}
   */
  public abstract ModuleConfig getBackRightModuleConfig();

  /**
   * Get configuration for front left module
   *
   * @return {@link ModuleConfig}
   */
  public abstract ModuleConfig getFrontLeftModuleConfig();

  /**
   * Get configuration for back left module
   *
   * @return {@link ModuleConfig}
   */
  public abstract ModuleConfig getBackLeftModuleConfig();

  public static record GyroConfig(CANBus canBus, int deviceId, Pigeon2Configuration config) {}

  public static record ModuleConfig(
      String name,
      CANBus canBus,
      int pivotDeviceId,
      int driveDeviceId,
      int cancoderId,
      TalonFXConfiguration pivotConfig,
      TalonFXConfiguration driveConfig,
      CANcoderConfiguration cancoderConfig,
      double wheelRadiusMeters) {}
}
