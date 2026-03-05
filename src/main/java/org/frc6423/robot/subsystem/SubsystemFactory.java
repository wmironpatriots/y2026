// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem;

import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.Drive;
import org.frc6423.robot.subsystem.drive.component.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOTalonFx;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOTalonFxSim;

// TODO move rest of subystems here
/** A class containing factory methods for building subsystems */
public class SubsystemFactory {
  /**
   * Create new {@link Drive} subsystem
   *
   * @return {@link Drive}
   */
  public static Drive createDriveSubsystem() {
    return (Robot.isReal())
        ? new Drive(
            new GyroIOPigeon2(
                Flags.kRobotType.mDriveConstants.getGyroConfig().deviceId(),
                Flags.kRobotType.mDriveConstants.getGyroConfig().canBus(),
                Flags.kRobotType.mDriveConstants.getGyroConfig().config()),
            new SwerveModuleIOTalonFx(
                "Front Right",
                Flags.kRobotType.mDriveConstants.getFrontRightModuleConfig(),
                Flags.kRobotType.mDriveConstants),
            new SwerveModuleIOTalonFx(
                "Front Left",
                Flags.kRobotType.mDriveConstants.getFrontLeftModuleConfig(),
                Flags.kRobotType.mDriveConstants),
            new SwerveModuleIOTalonFx(
                "Back Left",
                Flags.kRobotType.mDriveConstants.getBackLeftModuleConfig(),
                Flags.kRobotType.mDriveConstants),
            new SwerveModuleIOTalonFx(
                "Back Right",
                Flags.kRobotType.mDriveConstants.getBackRightModuleConfig(),
                Flags.kRobotType.mDriveConstants))
        : new Drive(
            new GyroIOPigeon2(
                Flags.kRobotType.mDriveConstants.getGyroConfig().deviceId(),
                Flags.kRobotType.mDriveConstants.getGyroConfig().canBus(),
                Flags.kRobotType.mDriveConstants.getGyroConfig().config()),
            new SwerveModuleIOTalonFxSim(
                "Front Right",
                Flags.kRobotType.mDriveConstants.getFrontRightModuleConfig(),
                Flags.kRobotType.mDriveConstants),
            new SwerveModuleIOTalonFxSim(
                "Front Left",
                Flags.kRobotType.mDriveConstants.getFrontLeftModuleConfig(),
                Flags.kRobotType.mDriveConstants),
            new SwerveModuleIOTalonFxSim(
                "Back Left",
                Flags.kRobotType.mDriveConstants.getBackLeftModuleConfig(),
                Flags.kRobotType.mDriveConstants),
            new SwerveModuleIOTalonFxSim(
                "Back Right",
                Flags.kRobotType.mDriveConstants.getBackRightModuleConfig(),
                Flags.kRobotType.mDriveConstants));
  }
}
