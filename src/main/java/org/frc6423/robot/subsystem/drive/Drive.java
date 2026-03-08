// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.frc6423.lib.util.Tracer;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.RobotState.OdometryMeasurement;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;

public class Drive extends SubsystemBase {

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  public static Lock kLock = new ReentrantLock();

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged(name = "Gyro")
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
          Tracer.traceFunc(
              "Proccess Hardware Signals",
              () -> {
                // Prevent odo thread from updating while processing data
                kLock.lock();

                // Update hardware
                mGyro.periodic();
                for (var module : mModules) {
                  module.periodic();
                }
                kLock.unlock();

                double[] sampleTimestamps =
                    Robot.isReal()
                        ? mGyro.getYawTimestampsSec()
                        : new double[] {Timer.getFPGATimestamp()};

                int sampleCount = sampleTimestamps.length;
                for (int i = 0; i < sampleCount; i++) {
                  SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];

                  for (int j = 0; j < 4; j++) {
                    // TODO wheelPositions[j] = mModules[j].getOdometryPositions()[i];
                  }

                  RobotState.getInstance()
                      .addOdometryMeasurement(
                          new OdometryMeasurement(
                              sampleTimestamps[i],
                              wheelPositions,
                              Optional.ofNullable(mGyro.getY)));
                }
              });
        });
  }
}
