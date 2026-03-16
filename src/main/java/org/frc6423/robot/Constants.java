// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frc6423.robot.subsystem.drive.constant.RebuiltL2;
import org.frc6423.robot.subsystem.drive.constant.SwerveConstants;

public final class Constants {
  public static final class Flags {
    public static final boolean kInitializeTunables = true;

    public static final boolean kInitializeSimulatedFuelPools = true;

    public static final SwerveConstants kDrivetrainContants = new RebuiltL2();

    public static final Importance kLowestLoggingLevel = Importance.DEBUG;

    public static Alliance getRobotAlliance() {
      return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static Rotation2d getAllianceRotation() {
      return Rotation2d.fromRotations(getRobotAlliance() == Alliance.Blue ? 0.0 : 0.5);
    }
  }

  public static final class Field {
    public static final Distance kFieldLength = Inches.of(651.22);

    public static final Distance kFieldWidth = Inches.of(317.69);

    public static final Distance kAllianceZoneLength = Inches.of(182.11);

    public static final Pose2d kMidPose =
        new Pose2d(kFieldLength.div(2), kFieldWidth.div(2), Rotation2d.kZero);

    public static Pose2d kBlueAllianceZonePose2d =
        new Pose2d(kAllianceZoneLength.div(2), kFieldWidth.div(2), Rotation2d.kZero);

    public static Pose2d kBlueAllianceHubPose2d =
        new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);

    public static Pose2d getHubPose2d() {
      return getRobotAlliancePose2d(kBlueAllianceHubPose2d);
    }

    public static Rectangle2d getAllianceZone() {
      return new Rectangle2d(
          getRobotAlliancePose2d(kBlueAllianceZonePose2d), kAllianceZoneLength, kFieldWidth);
    }

    public static Pose2d getRobotAlliancePose2d(Pose2d pose) {
      return (Flags.getRobotAlliance() == DriverStation.Alliance.Blue)
          ? pose
          : new Pose2d(
              pose.getTranslation().rotateAround(kMidPose.getTranslation(), Rotation2d.k180deg),
              pose.getRotation().plus(Rotation2d.k180deg));
    }
  }

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
