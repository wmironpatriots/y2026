// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * This is a globally accessible class for storing immutable values.
 *
 * <p>All values in this class are public, static, and final
 *
 * <p>To utilize values in this class, you should statically import the entire class or its
 * subclasses
 */
public final class Constants {
  /** The matrix contains the CAN identification information for all devices */
  public static final class Matrix {
    public static final CANBus kDriveCanBus = new CANBus("DRIVE");

    public static final int kDriveBrPivotId = 1;
    public static final int kDriveBrEncoderId = 2;
    public static final int kDriveBrDriveId = 3;
    public static final int kDriveFrPivotId = 4;
    public static final int kDriveFrEncoderId = 5;
    public static final int kDriveFrDriveId = 6;
    public static final int kDriveFlPivotId = 7;
    public static final int kDriveFlEncoderId = 8;
    public static final int kDriveFlDriveId = 9;
    public static final int kDriveBlPivotId = 10;
    public static final int kDriveBlEncoderId = 11;
    public static final int kDriveBlDriveId = 12;

    public static final CANBus kSubsystemCanBus = new CANBus("SOUP");

    public static final int kIntakePivotId = 1;
    public static final int kIntakeEncoderId = 2;
    public static final int kIntakeRollerId = 3;
    public static final int kIndexerId = 4;
    public static final int kFeederId = 5;
    public static final int kHoodId = 6;
    public static final int kFlywheelLeftId = 7;
    public static final int kFlywheelRightId = 8;
  }

  /**
   * Represents constants for the {@link Drive} Subsystem
   *
   * <p>TODO make this an abstract class for hotswap between cascade & 2026
   */
  public static final class DriveConstants {
    // * CHASSIS
    /**
     * {@link Distance} representing the width between the centers of swerve module wheels along
     * sides
     */
    public static final Distance kTrackDistance = Inches.of(20.76);

    /**
     * {@link Distance} representing the width between the center of chassis and a swerve module
     * wheel center
     */
    public static final Distance kTrackRadius =
        Inches.of(
            Math.hypot(kTrackDistance.baseUnitMagnitude(), kTrackDistance.baseUnitMagnitude())
                / 2.0);

    /** {@link Distance} representing the thickness of the bumpers */
    public static final Distance kBumperThickness = Inches.of(2.0); // TODO

    /**
     * {@link LinearVelocity} representing the maximum possible velocity of drivetrain /w FOC
     * enabled
     *
     * @see https://www.swervedrivespecialties.com/products/mk5i-swerve-module
     */
    public static final LinearVelocity kMaxLinearVelocityFoc = FeetPerSecond.of(16.8);

    /**
     * {@link LinearVelocity} representing the maximum possible velocity of drivetrain /w FOC
     * disabled
     *
     * @see https://www.swervedrivespecialties.com/products/mk5i-swerve-module
     */
    public static final LinearVelocity kMaxLinearVelocity = FeetPerSecond.of(17.4);

    /**
     * {@link LinearAcceleration} representing the maximum possible acceleration of drivetrain
     * (calculated using choreo)
     */
    public static final LinearAcceleration kMaxLinearAcceleration =
        MetersPerSecondPerSecond.of(12.624);

    /** {@link AngularVelocity} representing the maximum possible velocity of drivetrain */
    public static final AngularVelocity kMaxAngularVelocity =
        RadiansPerSecond.of(kMaxLinearVelocity.div(kTrackRadius.in(Meters)).baseUnitMagnitude());

    /**
     * {@link AngularAcceleration} representing the maximum possible acceleration drivetrain
     * (calculated using choreo)
     */
    public static final AngularAcceleration kMaxAngularAcceleration =
        RadiansPerSecondPerSecond.of(50.022);

    /** {@link Mass} representing the mass of the entire robot */
    public static final Mass kMass = Pounds.of(105.9);

    /** {@link MomentOfInertia} representing the Rotational Inertia of drivetrain */
    public static final MomentOfInertia kRotationalInertia =
        KilogramSquareMeters.of(17452.55455 * 0.0002926397);

    // * MODULES
    /** Represents the gear ratio between rotor of pivot servo to encoder/mechanism */
    public static final double kPivotRotorToSensorRatio = (26.0 / 1.0);

    /** When true, pivot servo should be inverted */
    public static final boolean kPivotInverted = false;

    /** Represents the gear ratio between relative encoder of drive servo to mechanism */
    public static final double kDriveSensorToMechRatio =
        (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0);

    /** {@link Distance} representing radius of swerve module wheels */
    public static final Distance kWheelRadius = Inches.of(2);

    /**
     * Calculate module displacement WRT chassis center
     *
     * @return {@link Translation2d} array representing calculated displacements (FR, FL, BL, BR)
     */
    public static final Translation2d[] getModuleDisplacements() {
      var coord = kTrackDistance.div(2);
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
    public static final TalonFXConfiguration getPivotConfig() {
      var config = new TalonFXConfiguration();
      // ...
      return config;
    }

    /**
     * @return {@link TalonFXConfiguration} representing servo config for swerve module drive
     */
    public static final TalonFXConfiguration getDriveConfig() {
      var config = new TalonFXConfiguration();
      // ...
      return config;
    }

    /**
     * @return {@link CANcoderConfiguration} representing encoder config for swerve module CANcoder
     */
    public static final CANcoderConfiguration getEncoderConfig() {
      var config = new CANcoderConfiguration();
      // ...
      return config;
    }
  }
}
