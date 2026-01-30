// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import com.ctre.phoenix6.CANBus;

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
}
