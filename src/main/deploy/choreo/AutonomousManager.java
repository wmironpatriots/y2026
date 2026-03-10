// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.autonomous;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import java.util.function.Consumer;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.subsystem.drive.Drive;

public abstract class AutonomousManager {
  private final AutoFactory mFactory;
  private final RobotState mRobotState = RobotState.getInstance();

  private final Consumer<SwerveSample> mSampleConsumer;

  public AutonomousManager(Drive drive) {
    mSampleConsumer = drive.getChoreoConsumer();

    mFactory =
        new AutoFactory(
            () -> mRobotState.getPose3d().toPose2d(),
            mRobotState::resetPose,
            mSampleConsumer::accept,
            true,
            drive);
  }
}
