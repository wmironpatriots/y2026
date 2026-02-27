// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;

// TODO
public class Drive extends SubsystemBase {
  public final DriveConstants mConstants = Flags.kRobotType.mDriveConstants;

  // * HARDWARE MEMBERS
  @Logged private final SwerveModuleIO mFrModule;
  @Logged private final SwerveModuleIO mFlModule;
  @Logged private final SwerveModuleIO mBlModule;
  @Logged private final SwerveModuleIO mBrModule;

  private final SwerveModuleIO[] mModules;
  private final GyroIO mGyro;

  // * SYSID MEMBERS
  private final SysIdRoutine mPivotCharacterization,
      mDriveLinearCharacterization,
      mDriveAngularCharacterization;

  /**
   * Create new {@link Drive}
   *
   * @param robotState {@link RobotState} RobotState instance
   * @param gyro {@link GyroIO} Gyro for odometry
   * @param frontRightModule {@link SwerveModuleIO} Front Right Swerve Module
   * @param frontLeftModule {@link SwerveModuleIO} Front Left Swerve Module
   * @param backLeftModule {@link SwerveModuleIO} Back Left Swerve Module
   * @param backRightModule {@link SwerveModuleIO} Back Right Swerve Module
   */
  public Drive(
      RobotState robotState,
      GyroIO gyro,
      SwerveModuleIO frontRightModule,
      SwerveModuleIO frontLeftModule,
      SwerveModuleIO backLeftModule,
      SwerveModuleIO backRightModule) {
    // Init hardware
    mFrModule = frontRightModule;
    mFlModule = frontLeftModule;
    mBlModule = backLeftModule;
    mBrModule = backRightModule;

    mModules =
        new SwerveModuleIO[] {
          mFrModule, mFlModule, mBlModule, mBrModule,
        };
    mGyro = gyro;

    // Init SysId
    mPivotCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {
                  for (int i = 0; i < mModules.length; i++) {
                    mModules[i].setPivotCurrent(Amps.of(Voltage.in(Volts)));
                  }
                },
                null,
                this,
                "SwervePivotSysId"));

    mDriveLinearCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {}, // TODO
                null,
                this,
                "SwerveLinearSysId"));

    mDriveAngularCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {}, // TODO
                null,
                this,
                "SwerveAngularSysId"));

    // Publish SysId commands if in tuning mode
    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData(runPivotCharacterizationSequence());
      SmartDashboard.putData(runLinearDriveCharacterization());
      SmartDashboard.putData(runAngularDriveCharacterization());
    }
  }

  @Override
  public void periodic() {
    // Update Hardware
    for (var module : mModules) {
      module.periodic();
      ;
    }

    // Send odometry measurements
    updateRobotState();
  }

  /** Update {@link RobotState} with odometry measurement */
  private void updateRobotState() {}

  // * COMMANDS
  public Command runPivotCharacterizationSequence() {
    return Commands.sequence(
            mPivotCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mPivotCharacterization.quasistatic(Direction.kReverse),
            Commands.waitSeconds(1.5),
            mPivotCharacterization.dynamic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mPivotCharacterization.dynamic(Direction.kReverse))
        .withName("Drive Pivot Characterization");
  }

  public Command runLinearDriveCharacterization() {
    return Commands.sequence(
            mDriveLinearCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveLinearCharacterization.dynamic(Direction.kReverse),
            Commands.waitSeconds(1.5),
            mDriveLinearCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveLinearCharacterization.dynamic(Direction.kReverse))
        .withName("Drive Linear Characterization");
  }

  public Command runAngularDriveCharacterization() {
    return Commands.sequence(
            mDriveAngularCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveAngularCharacterization.dynamic(Direction.kReverse),
            Commands.waitSeconds(1.5),
            mDriveAngularCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveAngularCharacterization.dynamic(Direction.kReverse))
        .withName("Drive Linear Characterization");
  }
}
