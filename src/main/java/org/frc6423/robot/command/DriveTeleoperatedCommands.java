// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;

/** Static {@link Command}s for driving drivetrain with teleoperated controls */
public class DriveTeleoperatedCommands {
  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link Double} Joystick deadband to apply to drive inputs */
  public static double kJoystickDeadband = 0.02;

  /** {@link String} NT directory to store tunables */
  public static String kTunablesPrefix = "/Drive Teleoperated Controllers";

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

      kAngularKp.initDefault(4.0);
      kAngularKd.initDefault(0.0);
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

  public static void updateTunables(DriveSubsystem drive) {
    // Update tunables if needed
    if (kTranslationalKp.hasChanged(drive.hashCode())
        || kTranslationalKd.hasChanged(drive.hashCode())
        || kTranslationalToleranceCm.hasChanged(drive.hashCode())) {

      kTranslationalXController.setPID(kTranslationalKp.get(), 0.0, kTranslationalKd.get());
      kTranslationalXController.setTolerance(kTranslationalToleranceCm.get() / 100.0);
      kTranslationalXController.reset();

      kTranslationalYController.setPID(kTranslationalKp.get(), 0.0, kTranslationalKd.get());
      kTranslationalYController.setTolerance(kTranslationalToleranceCm.get() / 100.0);
      kTranslationalYController.reset();
    }

    if (kAngularKp.hasChanged(drive.hashCode()) || kAngularKd.hasChanged(drive.hashCode())) {

      kAngularController.setPID(kAngularKp.get(), 0.0, kAngularKd.get());
      kAngularController.reset();
    }
  }

  private DriveTeleoperatedCommands() {}

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Set joystick deadband to apply to teleoperated inputs
   *
   * @param value {@link Double} Deadband magnitude to apply to joystick inputs (between 0-1)
   */
  public static void setTeleoperatedJoystickDeadband(double value) {
    kJoystickDeadband = value;
  }

  // * ~~~~~~~~ MODIFIERS ~~~~~~~~

  /**
   * Convert requested linear speed magnitudes in the x, y directions to a linear velocity vector
   *
   * @param vx {@link Double} Requested magnitude of max speed in x direction
   * @param vy {@link Double} Requested magnitude of max speed in y direction
   * @return {@link Translation2d}
   */
  private static Translation2d getLinearVelocityFromJoysticks(double vx, double vy) {
    // Calculate the request magnitude and the direction of it
    var linearNorm = MathUtil.applyDeadband(Math.hypot(vx, vy), kJoystickDeadband);
    var linearDirection = new Rotation2d(vx, vy);

    // Square magnitude
    linearNorm = linearNorm * linearNorm;

    // Calculate linear vel vector
    var linearVel =
        new Pose2d(Translation2d.kZero, linearDirection)
            .transformBy(new Transform2d(linearNorm, 0.0, Rotation2d.kZero))
            .getTranslation()
            .times(Flags.kDriveConstants.getMaxLinearVelocityMetersPerSecond());

    return linearVel;
  }

  /**
   * Convert requested angular rate magnitude into a drive omega
   *
   * @param omega {@link Double} Requested magnitude of max angular rate
   * @return {@link Double}
   */
  private static double getOmegaFromJoystick(double omega) {
    var deadband =
        MathUtil.applyDeadband(omega, kJoystickDeadband)
            * (Flags.kDriveConstants.getMaxAngularVelocityRadsPerSec() / 4);

    return deadband * deadband * Math.signum(omega) * -1;
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Calculate a {@link ChassisSpeeds} from teleoperated requested speeds (vx, vy, omega) and
   * request a {@link DriveSubsystem} to run it
   *
   * @param drive {@link DriveSubsystem} Drivetrain to run desired setpoint
   * @param vxSupplier {@link DoubleSupplier} Requested magnitude of max velocity in the x direction
   * @param vySupplier {@link DoubleSupplier} Requested magnitude of max velocity in the y direction
   * @param omegaSupplier {@link DoubleSupplier} Requested nagitude of max angular rate
   * @return {@link Command}
   */
  public static Command runTeleoperatedDrive(
      DriveSubsystem drive,
      DoubleSupplier vxSupplier,
      DoubleSupplier vySupplier,
      DoubleSupplier omegaSupplier) {
    return drive.run(
        () -> {
          // Calculate linear vel vector
          var linear =
              getLinearVelocityFromJoysticks(vxSupplier.getAsDouble(), vySupplier.getAsDouble());

          // Calculate omega
          var omega = getOmegaFromJoystick(omegaSupplier.getAsDouble());

          // Run setpoint
          drive.setChassisSpeedsSetpoint(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linear.getX(),
                  linear.getY(),
                  omega,
                  drive.getRotation2d().plus(Flags.getAllianceRotation())));
        });
  }

  /**
   * Calculate a {@link ChassisSpeeds} from teleoperated requested linear speeds (x, y) & a target
   * to face
   *
   * @param drive {@link DriveSubystem} Drivetrain to run desired setpoint
   * @param vxSupplier {@link DoubleSupplier} Requested magnitude of max velocity in the x direction
   * @param vySupplier {@link DoubleSupplier} Requested magnitude of max velocity in the y direction
   * @param targetSupplier {@link Supplier} of {@link Translation2d} Field coordinates of target to
   *     face
   * @param flipped {@link Boolean} Whether the opposite side of drivetrain should be facing target
   * @return {@link Command}
   */
  public static Command runTeleoperatedDriveWhileFacing(
      DriveSubsystem drive,
      DoubleSupplier vxSupplier,
      DoubleSupplier vySupplier,
      Supplier<Translation2d> targetSupplier,
      boolean flip) {
    return runTeleoperatedDriveWithAngularAssist(
        drive,
        vxSupplier,
        vySupplier,
        () ->
            targetSupplier
                .get()
                .minus(drive.getPose2d().getTranslation())
                .getAngle()
                .plus((flip) ? Rotation2d.k180deg : Rotation2d.kZero));
  }

  /**
   * Calculate a {@link ChassisSpeeds} from teleoperated requested linear speeds (x, y) and
   * caculates an omega using feedback to reach a specified angle
   *
   * @param drive {@link DriveSubsystem} Drivetrain to run desired setpoint
   * @param vxSupplier {@link DoubleSupplier} Requested magnitude of max velocity in the x direction
   * @param vySupplier {@link DoubleSupplier} Requested magnitude of max velocity in the y direction
   * @param angleSupplier {@link Supplier} of {@link Rotation2d} Requested angle of drivetrain
   * @return {@link Command}
   */
  public static Command runTeleoperatedDriveWithAngularAssist(
      DriveSubsystem drive,
      DoubleSupplier vxSupplier,
      DoubleSupplier vySupplier,
      Supplier<Rotation2d> angleSupplier) {
    return drive.run(
        () -> {
          updateTunables(drive);

          var linear =
              getLinearVelocityFromJoysticks(vxSupplier.getAsDouble(), vySupplier.getAsDouble());
          var omega =
              kAngularController.calculate(
                  drive.getRotation2d().getRadians(), angleSupplier.get().getRadians());

          // Run setpoint
          drive.setChassisSpeedsSetpoint(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linear.getX(),
                  linear.getY(),
                  omega,
                  drive.getRotation2d().plus(Flags.getAllianceRotation())));
        });
  }
}
