// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.frc6423.lib.util.Tracer;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.RobotState.OdometryMeasurement;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOComp;
import org.frc6423.robot.subsystem.drive.constants.SwerveConstants;

public class Drive extends SubsystemBase {
  public static Drive create() {
    return new Drive(
        new GyroIOPigeon2(kConstants.getGyroConfig()),
        new SwerveModuleIOComp(kConstants.getFrontRightModuleConfig()),
        new SwerveModuleIOComp(kConstants.getBackRightModuleConfig()),
        new SwerveModuleIOComp(kConstants.getFrontLeftModuleConfig()),
        new SwerveModuleIOComp(kConstants.getBackLeftModuleConfig()));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  public static final SwerveConstants kConstants = Flags.kDriveConstants;

  public static Lock kLock = new ReentrantLock();

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private final GyroIO mGyro;

  @Logged(name = "Front Right")
  private final SwerveModuleIO mFrModule;

  @Logged(name = "Back Right")
  private final SwerveModuleIO mBrModule;

  @Logged(name = "Front Left")
  private final SwerveModuleIO mFlModule;

  @Logged(name = "Back Left")
  private final SwerveModuleIO mBlModule;

  private final SwerveModuleIO[] mModules;

  public Drive(
      GyroIO gyro,
      SwerveModuleIO frontRight,
      SwerveModuleIO backRight,
      SwerveModuleIO frontLeft,
      SwerveModuleIO backLeft) {
    mGyro = gyro;

    mFrModule = frontRight;
    mBrModule = backRight;
    mFlModule = frontLeft;
    mBlModule = backLeft;

    mModules = new SwerveModuleIO[] {mFrModule, mBrModule, mFlModule, mBlModule};
  }

  @Override
  public void periodic() {
    Tracer.traceFunc(
        "Odometry Update",
        () -> {
          // Prevent odo thread from updating while processing data
          kLock.lock();

          try {

            // Get samples & their timestamps
            var sampleTimestamps =
                Robot.isReal()
                    ? mGyro.getYawTimestampsSec()
                    : new double[] {Timer.getFPGATimestamp()};

            var gyroSamples = mGyro.getYawRotationsRads();
            var moduleSamples =
                new SwerveModulePosition[][] {
                  mModules[0].getWheelPositions(),
                  mModules[1].getWheelPositions(),
                  mModules[2].getWheelPositions(),
                  mModules[3].getWheelPositions(),
                };

            var sampleSize = sampleTimestamps.length;

            // Give all data to {@link RobotState}
            for (int i = 0; i < sampleSize; i++) {
              SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];

              for (int j = 0; j < 4; j++) {
                wheelPositions[j] = moduleSamples[j][i];
              }

              RobotState.getInstance()
                  .addOdometryMeasurement(
                      new OdometryMeasurement(
                          sampleTimestamps[i],
                          wheelPositions,
                          (Robot.isReal())
                              ? Optional.of(Rotation2d.fromRadians(gyroSamples[i]))
                              : Optional.empty()));
            }

          } catch (RuntimeException e) {
            e.printStackTrace();
          } finally {
            kLock.unlock();
          }
        });

    // todo
    RobotState.getInstance().setChassisSpeeds(getChassisSpeeds());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kConstants.getKinematics().toChassisSpeeds(getWheelStates());
  }

  public SwerveModuleState[] getWheelStates() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getWheelState)
        .toArray(SwerveModuleState[]::new);
  }
}
