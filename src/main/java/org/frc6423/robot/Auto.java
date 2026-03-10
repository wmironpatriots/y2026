// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.drive.Drive;

/** A manager class for building autonomous routines */
public class Auto {
  private final Drive mDrive;
  private final AutoFactory mFactory;

  public Auto(Drive drive) {
    this.mDrive = drive;

    // Init Auto Factory
    mFactory =
        new AutoFactory(
            () -> RobotState.getInstance().getEstimatedPosition(),
            (pose) -> RobotState.getInstance().resetPose(pose),
            drive.getChoreoSwerveSampleConsumer(),
            true,
            drive);
  }

  // Enum for key field nodes (starting poses)
  public static enum Node {
    S1(new Pose2d(4.50, 0.65, Rotation2d.fromRadians(1 / 4 * Math.PI))),
    S2(new Pose2d(3.55, 4.00, Rotation2d.fromRadians(Math.PI))),
    S3(new Pose2d(3.6513702869415283, 7.2504496574401855, Rotation2d.fromRadians(Math.PI))),
    S4(new Pose2d(3.55, 0.70, Rotation2d.fromRadians(Math.PI))),
    N2(new Pose2d(8.00, 2.9, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    N3(new Pose2d(7.90, 4.00, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    N4(new Pose2d(7.90, 5.20, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    N5(new Pose2d(7.90, 6.00, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    A1(new Pose2d(0.65, 5.95, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    F(new Pose2d(2.33, 4.00, Rotation2d.fromRadians(Math.PI)));
    private final Pose2d mPose2d;

    private Node(Pose2d pose) {
      mPose2d = pose;
    }

    public Pose2d getPose2d() {
      return mPose2d;
    }
  }

  // Starting Depot Edges
  public Command driveS4toA1() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.S4.getPose2d()))
        .andThen(mFactory.trajectoryCmd("S4_A1"));
  }

  public Command driveS2toA1() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.S2.getPose2d()))
        .andThen(mFactory.trajectoryCmd("S2_A1"));
  }

  public Command driveS3toA1() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.S3.getPose2d()))
        .andThen(mFactory.trajectoryCmd("S3_A1"));
  }

  // Starting Firing edge
  public Command driveA1toF() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.A1.getPose2d()))
        .andThen(mFactory.trajectoryCmd("A1_F"));
  }

  // Starting S1 to Neutral Zone edges
  public Command driveS1toN2() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.S1.getPose2d()))
        .andThen(mFactory.trajectoryCmd("S1_N2"));
  }

  public Command driveS1toN3() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.S1.getPose2d()))
        .andThen(mFactory.trajectoryCmd("S1_N3"));
  }

  public Command driveS1toN4() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.S1.getPose2d()))
        .andThen(mFactory.trajectoryCmd("S1_N4"));
  }

  public Command driveS1toN5() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.S1.getPose2d()))
        .andThen(mFactory.trajectoryCmd("S1_N5"));
  }

  // Starting Neutral Zone reverse edges
  public Command driveN2toS1() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.N2.getPose2d()))
        .andThen(mFactory.trajectoryCmd("N2_S1"));
  }

  public Command driveN3toS1() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.N3.getPose2d()))
        .andThen(mFactory.trajectoryCmd("N3_S1"));
  }

  public Command driveN4toS1() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.N4.getPose2d()))
        .andThen(mFactory.trajectoryCmd("N4_S1"));
  }

  public Command driveN5toS1() {
    return mDrive
        .runOnce(() -> mDrive.resetPosition(Node.N5.getPose2d()))
        .andThen(mFactory.trajectoryCmd("N5_S1"));
  }
}
