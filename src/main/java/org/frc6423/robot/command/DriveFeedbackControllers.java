// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;

/** Wrapper for WPIlib {@link PIDController} for x, y, angular positions (meters, meters, rads) */
public class DriveFeedbackControllers {
  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link String} NT directory to store tunables */
  public static String kTunablesPrefix = "/Drive Feedback Controllers";

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  /** {@link TunableNumber} Gain driving translational position error to 0 */
  public static final TunableNumber kTranslationalKp =
      new TunableNumber(kTunablesPrefix + "/Translational kP");

  /** {@link TunableNumber} Gain driving the derivative of translational position error to 0 */
  public static final TunableNumber kTranslationalKd =
      new TunableNumber(kTunablesPrefix + "/Translational kD");

  /** {@link TunableNumber} Max acceptable translational position error (centimeters) */
  public static final TunableNumber kTranslationalToleranceCm =
      new TunableNumber(kTunablesPrefix + "/Translational Tolernace (centimeters)");

  /** {@link TunableNumber} Gain driving angular position error to 0 */
  public static final TunableNumber kAngularKp = new TunableNumber(kTunablesPrefix + "/Angular kP");

  /** {@link TunableNumber} Gain driving the derivative of angular position error to 0 */
  public static final TunableNumber kAngularKd = new TunableNumber(kTunablesPrefix + "/Angular kD");

  /** {@link TunableNumber} Max acceptable angular position error in (degrees) */
  public static final TunableNumber kAngularToleranceDeg =
      new TunableNumber(kTunablesPrefix + "/Angular Tolerance (degrees)");

  static {
    if (Robot.isReal()) {
      kTranslationalKp.initDefault(8.0);
      kTranslationalKd.initDefault(0.0);
      kTranslationalToleranceCm.initDefault(1.0);

      kAngularKp.initDefault(11.5);
      kAngularKd.initDefault(0.05);
      kAngularToleranceDeg.initDefault(2.0);
    } else {
      kTranslationalKp.initDefault(8.0);
      kTranslationalKd.initDefault(0.0);
      kTranslationalToleranceCm.initDefault(1.0);

      kAngularKp.initDefault(11.5);
      kAngularKd.initDefault(0.05);
      kAngularToleranceDeg.initDefault(2.0);
    }
  }

  // * ~~~~~~~~ WPILIB ~~~~~~~~

  /**
   * {@link PIDController} Feedback controller for managing translational position error in the x
   * direction (meters)
   */
  public static final PIDController kTranslationalXController;

  /**
   * {@link PIDController} Feedback controller for managing translational position error in the y
   * direction (meters)
   */
  public static final PIDController kTranslationalYController;

  /** {@link PIDController} Feedback controller for managing angular position error (radians) */
  public static final PIDController kAngularController;

  static {
    kTranslationalXController =
        new PIDController(kTranslationalKp.get(), 0.0, kTranslationalKd.get());
    kTranslationalXController.setTolerance(kTranslationalToleranceCm.get() / 100.0);

    kTranslationalYController =
        new PIDController(kTranslationalKp.get(), 0.0, kTranslationalKd.get());
    kTranslationalYController.setTolerance(kTranslationalToleranceCm.get() / 100.0);

    kAngularController = new PIDController(kAngularKp.get(), 0.0, kAngularKd.get());
    kAngularController.setTolerance(Units.degreesToRadians(kAngularToleranceDeg.get()));
    kAngularController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Update feedback gains from tunables
   *
   * @param drive {@link Drive} cooked way of doing this but I need a hashcode i think
   */
  private static void updateFeedbackGains(DriveSubsystem drive) {
    // Update tunables if needed
    if (kTranslationalKp.hasChanged(drive.hashCode())
        || kTranslationalKd.hasChanged(drive.hashCode())
        || kTranslationalToleranceCm.hasChanged(drive.hashCode())) {
      kTranslationalXController.setPID(kTranslationalKp.get(), 0.0, kTranslationalKd.get());
      kTranslationalXController.setTolerance(kTranslationalToleranceCm.get() / 100.0);
      kTranslationalYController.setPID(kTranslationalKp.get(), 0.0, kTranslationalKd.get());
      kTranslationalYController.setTolerance(kTranslationalToleranceCm.get() / 100.0);
      reset();
    }

    if (kAngularKp.hasChanged(drive.hashCode()) || kAngularKd.hasChanged(drive.hashCode())) {
      kAngularController.setPID(kAngularKp.get(), 0.0, kAngularKd.get());
      reset();
    }
  }

  private DriveFeedbackControllers() {}

  /**
   * Calculate angular feedback output
   *
   * @param drive {@link DriveSubsystem} Drive subsystem to calculate for (used for hashcode)
   * @param measurement {@link Double} Current estimated angular position of robot
   * @param setpoint {@link Double} Desired angular position of robot
   * @return {@link Double}
   */
  public static double calculateAngular(DriveSubsystem drive, double measurement, double setpoint) {
    updateFeedbackGains(drive);

    return kAngularController.calculate(measurement, setpoint);
  }

  /**
   * Calculate translational feedback output in the x direction
   *
   * @param drive {@link DriveSubsystem} Drive subsystem to calculate for (used for hashcode)
   * @param measurement {@link Double} Current estimated x position of robot
   * @param setpoint {@link Double} Desired x position of robot
   * @return {@link Double}
   */
  public static double calculateTranslationalX(
      DriveSubsystem drive, double measurement, double setpoint) {
    updateFeedbackGains(drive);

    return kTranslationalXController.calculate(measurement, setpoint);
  }

  /**
   * Calculate translational feedback output in the y direction
   *
   * @param drive {@link DriveSubsystem} Drive subsystem to calculate for (used for hashcode)
   * @param measurement {@link Double} Current estimated y position of robot
   * @param setpoint {@link Double} Desired y position of robot
   * @return {@link Double}
   */
  public static double calculateTranslationalY(
      DriveSubsystem drive, double measurement, double setpoint) {
    updateFeedbackGains(drive);

    return kTranslationalYController.calculate(measurement, setpoint);
  }

  /** Reset previous error and integral terms from controllers */
  public static void reset() {
    kAngularController.reset();
    kTranslationalXController.reset();
    kTranslationalYController.reset();
  }
}
