// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;

public class AutoBuilder {
  private final DriveSubsystem mDrive;
  private final AutoFactory mFactory;

  public AutoBuilder(DriveSubsystem drive) {
    mDrive = drive;

    mFactory =
        new AutoFactory(mDrive::getPose2d, mDrive::reset, mDrive::runSwerveSample, true, drive);
  }

  public enum Path {
    NEUTRAL_SHOT_1("NeutralShot1");

    private final String mName;

    private Path(String name) {
      mName = name;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      return routine.trajectory(mName);
    }
  }

  public Command getSelectedAuton() {
    return Commands.none();
  }
}
