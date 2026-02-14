// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.function.Consumer;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.gyro.GyroIO;
import org.frc6423.robot.subsystem.drive.gyro.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIOTalonFx;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIOTalonFxSim;

/** {@link SubsystemBase} extension representing swerve drivetrain */
public class Drive extends SubsystemBase {
  private final RobotState mRobotState;

  private final DriveConstants mConstants;
  private final SwerveDriveKinematics mKinematics;

  @Logged private final SwerveModuleIO mFrModule;
  @Logged private final SwerveModuleIO mFlModule;
  @Logged private final SwerveModuleIO mBlModule;
  @Logged private final SwerveModuleIO mBrModule;
  private final SwerveModuleIO[] mModules;
  @Logged private final GyroIO mGyro;

  private SwerveModuleState[] mSetpointStates;

  private boolean mAutoFocToggle = true;

  /**
   * Create new {@link Drive}
   *
   * @param robotState {@link RobotState} instance to send odometry samples to
   */
  public Drive(RobotState robotState) {
    mRobotState = robotState;

    mConstants = Flags.kDriveConstants;
    mKinematics = mConstants.getKinematics();

    mGyro = new GyroIOPigeon2(mConstants.getGyroConfig());

    if (Robot.isReal()) {
      mFrModule = new SwerveModuleIOTalonFx(mConstants.getFrontRightModuleConfig(), mConstants);
      mFlModule = new SwerveModuleIOTalonFx(mConstants.getFrontLeftModuleConfig(), mConstants);
      mBlModule = new SwerveModuleIOTalonFx(mConstants.getBackLeftModuleConfig(), mConstants);
      mBrModule = new SwerveModuleIOTalonFx(mConstants.getBackRightModuleConfig(), mConstants);
    } else {
      mFrModule = new SwerveModuleIOTalonFxSim(mConstants.getFrontRightModuleConfig(), mConstants);
      mFlModule = new SwerveModuleIOTalonFxSim(mConstants.getFrontLeftModuleConfig(), mConstants);
      mBlModule = new SwerveModuleIOTalonFxSim(mConstants.getBackLeftModuleConfig(), mConstants);
      mBrModule = new SwerveModuleIOTalonFxSim(mConstants.getBackRightModuleConfig(), mConstants);
    }

    mModules = new SwerveModuleIO[] {mFrModule, mFlModule, mBlModule, mBrModule};
  }

  @Override
  public void periodic() {
    // TODO odometry thread

    // Update Swerve Module Signals
    for (var module : mModules) {
      module.periodic();
    }

    // Stop dt when disabled
    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /**
   * @return {@link Rotation2d} representing estimated chassis yaw orientation
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return mRobotState.getRotation2d();
  }

  /**
   * @return {@link Rotation3d} representing estimated chassis orientation in 3D space (yaw, pitch,
   *     roll)
   */
  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public Rotation3d getRotation3d() {
    return mRobotState.getRotation3d();
  }

  /**
   * @return {@link Pose3d} representing estimated chassis position in 2D space (x, y)
   */
  @Logged(name = "Pose2d", importance = Importance.INFO)
  public Pose2d getPose2d() {
    return mRobotState.getPose2d();
  }

  /**
   * @return {@link Pose3d} representing estimated chassis position in 3D space (x, y, z)
   */
  @Logged(name = "Pose3d", importance = Importance.INFO)
  public Pose3d getPose3d() {
    return mRobotState.getPose3d();
  }

  /**
   * @return {@link LinearVelocity} representing estimated chassis speed
   */
  @Logged(name = "Linear Velocity", importance = Importance.INFO)
  public LinearVelocity getLinearVelocity() {
    var speeds = getRobotRelativeChassisSpeeds();
    return MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
  }

  /**
   * @return {@link ChassisSpeeds} representing estimated chassis velocity components relative to
   *     field
   */
  @Logged(name = "Chassis Speeds WRT Field", importance = Importance.INFO)
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getRobotRelativeChassisSpeeds(), mGyro.getRotation2d());
  }

  /**
   * @return {@link ChassisSpeeds} representing estimated chassis velocity components
   */
  @Logged(name = "Chassis Speeds", importance = Importance.INFO)
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return mKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * @return {@link SwerveModulePosition} {@link Array} representing the field position of each
   *     swerve module WRT to its initial position
   */
  @Logged(name = "Module Poses", importance = Importance.INFO)
  public SwerveModulePosition[] getSwerveModulePositions() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getSwerveModulePosition)
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * @return {@link SwerveModuleState} {@link Array} representing the velocity components of each
   *     swerve module
   */
  @Logged(name = "Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSwerveModuleStates() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getSwerveModuleState)
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * @return {@link SwerveModuleState} {@link Array} representing the desired velocity components of
   *     each swerve module
   */
  @Logged(name = "Setpoint Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSetpointSwerveModuleStates() {
    return null; // Arrays.stream(mModules).map(SwerveModuleIO::desiredState).toArray(SwerveModuleState[]::new);
  }

  /**
   * @return true if FOC is enabled
   */
  @Logged(name = "FOC enabled", importance = Importance.INFO)
  public boolean isFocEnabled() {
    return mAutoFocToggle
        && getLinearVelocity()
            .gt(mConstants.getMaxLinearVelocity().times(mConstants.getFocAutoToggleMagnitude()));
  }

  /**
   * Disable Auto Field-Oriented Control (FOC) Toggle
   *
   * <p>Auto FOC Toggle is a system that automatically enables FOC while drivetrain is accelerating
   * and then disables it once a certain magnitude of the maximum possible drivetrain speed is
   * reached. This allows the drivetrain to utilize the high acceleration offered by FOC without the
   * cost of maximum speed
   *
   * <p>When Auto FOC Toggle is disabled, drivetrain will default to utilizing FOC control.
   *
   * <p><strong> WARNING </strong> ~ FOC is required when controlling wheel torques; You must
   * disable auto toggle if you wish to.
   */
  public void disableAutoFocToggle() {
    mAutoFocToggle = false;
  }

  /**
   * Enable Auto Field-Oriented Control (FOC) Toggle
   *
   * <p>Auto FOC Toggle is a system that automatically enables FOC while drivetrain is accelerating
   * and then disables it once a certain magnitude of the maximum possible drivetrain speed is
   * reached. This allows the drivetrain to utilize the high acceleration offered by FOC without the
   * cost of maximum speed
   *
   * <p>When Auto FOC Toggle is disabled, drivetrain will default to utilizing FOC control.
   *
   * <p><strong> WARNING </strong> ~ FOC is required when controlling wheel torques; You must
   * disable auto toggle if you wish to.
   */
  public void enableAutoFocToggle() {
    mAutoFocToggle = true;
  }

  protected void setChassisSpeedsSetpoint(ChassisSpeeds speeds, boolean openLoopEnabled) {}

  /**
   * @return {@link Consumer} for running {@link SwerveSample} setpoints
   */
  public Consumer<SwerveSample> getSwerveSampleConsumer() {
    return (sample) -> {};
  }

  /** Stop drivetrain completely with the wheel forming an X */
  public void xStop() {}

  /** Stop drivetrain completely */
  public void stop() {
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].stop();
    }
  }
}
