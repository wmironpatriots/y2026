// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import com.ctre.phoenix6.CANBus;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.RebuiltL1;
import org.frc6423.robot.subsystem.drive.constants.RebuiltL2;

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
    /** {@link RobotType} representing the robot chassis being used */
    public static RobotType kRobotType = RobotType.Y2026_L2;

    /** When true, subsystems will not be initialized */
    public static boolean kSubsystemDisabled = false;

    /** When true, drive will not be initialized */
    public static boolean kDriveDisabled = false;
  }

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
  }

  /** Represents the type of robot codebase is running on */
  public static enum RobotType {
    /** {@link RobotType} representing the 2026 competition robot chassis /w L1 Ratio */
    Y2026_L1(new RebuiltL1()),
    /** {@link RobotType} representing the 2026 competition robot chassis /w L2 Ratio */
    Y2026_L2(new RebuiltL2()),
    /** {@link RobotType} representing the 2026 competition robot chassis /w L3 Ratio */
    Y2026_L3(new RebuiltL2());

    public final DriveConstants mDriveConstants;

    private RobotType(DriveConstants drivetrainConstants) {
      this.mDriveConstants = drivetrainConstants;
    }
  }
}
