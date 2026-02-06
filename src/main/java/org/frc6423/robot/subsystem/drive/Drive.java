// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.frc6423.robot.Robot;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.RobotState.OdometryMeasurement;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.gyro.GyroIO;
import org.frc6423.robot.subsystem.drive.gyro.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIOTalonFx;

public class Drive extends SubsystemBase {
  private final DriveConstants mConstants;

  private final RobotState mRobotState;
  private final SwerveDriveKinematics mKinematics;

  private final SwerveModuleIO[] mModules;
  private final GyroIO gyro;

  public Drive(DriveConstants constants) {
    mConstants = constants;

    mRobotState = RobotState.createInstance(constants);
    mKinematics = constants.getKinematics();

    var configs = constants.getModuleConfigs();
    mModules = new SwerveModuleIO[configs.length];
    gyro = new GyroIOPigeon2(mConstants.getGyroConfig());

    for (int i = 0; i < configs.length; i++) {
      mModules[i] =
          Robot.isReal()
              ? new SwerveModuleIOTalonFx(configs[i].name(), configs[i], constants)
              : new SwerveModuleIOTalonFx(configs[i].name(), configs[i], constants);
    }
  }

  @Override
  public void periodic() {
    // TODO high freq odometry
    mRobotState.addOdometryMeasurement(
        new OdometryMeasurement(
            Timer.getTimestamp(),
            getSwerveModulePositions(),
            Robot.isReal() ? Optional.of(getRotation3d()) : Optional.empty()));

    for (var module : mModules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /**
   * @return {@link Rotation2d} representing measured yaw rotation
   */
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  /**
   * @return {@link Rotation3d} representing the measured orientation of the robot in 3D space
   */
  public Rotation3d getRotation3d() {
    return gyro.getRotation3d();
  }

  /**
   * @return {@link SwerveModulePosition} representing the displacement vectors of swerve modules
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    var poses = new SwerveModulePosition[mModules.length];
    for (int i = 0; i < poses.length; i++) {
      poses[i] = mModules[i].getSwerveModulePosition();
    }

    return poses;
  }

  // TODO
  public void stop() {}
}
