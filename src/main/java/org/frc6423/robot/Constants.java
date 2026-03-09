// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frc6423.lib.util.GeometryUtil;
import org.frc6423.robot.subsystem.drive.constants.RebuiltL2;
import org.frc6423.robot.subsystem.drive.constants.SwerveConstants;

/**
 * This is a globally accessible class for storing immutable values.
 *
 * <p>All values in this class are public, static, and final
 *
 * <p>To utilize values in this class, you should statically import the entire class or its
 * subclasses
 */
public final class Constants {
  /** Runtime flags determining the robots initialization */
  public static final class Flags {
    /** {@link SwerveConstants} Drivetrain characterization */
    public static final SwerveConstants kDriveConstants = new RebuiltL2();

    /** When true, subsystems will not be initialized */
    public static final boolean kSubsystemDisabled = false;

    /** When true, drive will not be initialized */
    public static final boolean kDriveDisabled = false;

    /** When true, tunables/characterization commands will appear on dashboard */
    public static final boolean kTuningModeEnabled = true;

    /** {@link Importance} Minimum Epilogue importance to be logged */
    public static final Importance kLoggingLevel = Importance.DEBUG;

    public static final Alliance kRobotAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    public static Rotation2d getAllianceRotation() {
      return Rotation2d.fromRotations(kRobotAlliance == Alliance.Blue ? 0.0 : 0.5);
    }

    public static Pose2d getRobotAlliancePose2d(Pose2d pose) {
      return GeometryUtil.allianceFlipPose2d(Rebuilt.kMidPose, pose);
    }
  }

  /** The matrix contains the CAN identification information for all devices */
  public static final class Matrix {
    public static final CANBus kDriveCanBus = new CANBus("DRIVE");

    public static final int kDriveFrPivotId = 1;
    public static final int kDriveFrEncoderId = 9;
    public static final int kDriveFrDriveId = 2;

    public static final int kDriveBrPivotId = 7;
    public static final int kDriveBrEncoderId = 12;
    public static final int kDriveBrDriveId = 8;

    public static final int kDriveFlPivotId = 3;
    public static final int kDriveFlEncoderId = 10;
    public static final int kDriveFlDriveId = 4;

    public static final int kDriveBlPivotId = 5;
    public static final int kDriveBlEncoderId = 11;
    public static final int kDriveBlDriveId = 6;

    public static final int kDriveGyroId = 13;

    public static final CANBus kSubsystemCanBus = new CANBus("SOUP");

    public static final int kIntakePivotId = 14;
    public static final int kIntakeEncoderId = 15;
    public static final int kIntakeRollerId = 16;

    public static final int kIndexerId = 17;

    public static final int kFeederId = 18;

    public static final int kHoodId = 19;
    public static final int kHoodEncoderId = 20;

    public static final int kFlywheelLeftId = 21;
    public static final int kFlywheelRightId = 22;

    public static final int kIntakeBeamBreakDio = 0;
    public static final int kFeederBeamBreakDio = 1;
  }
}
