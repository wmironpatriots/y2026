// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.frc6423.robot.Robot;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.RobotState.OdometryMeasurement;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.gyro.GyroIO;
import org.frc6423.robot.subsystem.drive.gyro.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIOTalonFx;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIOTalonFxSim;

/** {@link SubsystemBase} extension representing swerve drivetrain */
public class Drive extends SubsystemBase {
  private final DriveConstants mConstants;

  private final RobotState mRobotState;
  private final SwerveDriveKinematics mKinematics;

  @Logged private final SwerveModuleIO mFrModule;
  @Logged private final SwerveModuleIO mFlModule;
  @Logged private final SwerveModuleIO mBlModule;
  @Logged private final SwerveModuleIO mBrModule;
  private final SwerveModuleIO[] mModules;
  @Logged private final GyroIO mGyro;

  private SwerveModuleState[] mSetpointStates;

  private Rotation2d mSimRotation2d = Rotation2d.kZero;

  private boolean mIsFocAutoToggleEnabled = true;

  /**
   * Create new {@link Drive}
   *
   * @param constants {@link DriveConstants} representing subsystem configuration
   */
  public Drive(DriveConstants constants) {
    mConstants = constants;

    mRobotState = RobotState.getInstance();
    mKinematics = constants.getKinematics();

    mGyro = new GyroIOPigeon2(mConstants.getGyroConfig());

    if (Robot.isReal()) {
      mFrModule = new SwerveModuleIOTalonFx(mConstants.getFrontRightModuleConfig(), constants);
      mFlModule = new SwerveModuleIOTalonFx(mConstants.getFrontLeftModuleConfig(), constants);
      mBlModule = new SwerveModuleIOTalonFx(mConstants.getBackLeftModuleConfig(), constants);
      mBrModule = new SwerveModuleIOTalonFx(mConstants.getBackRightModuleConfig(), constants);
    } else {
      mFrModule = new SwerveModuleIOTalonFxSim(mConstants.getFrontRightModuleConfig(), constants);
      mFlModule = new SwerveModuleIOTalonFxSim(mConstants.getFrontLeftModuleConfig(), constants);
      mBlModule = new SwerveModuleIOTalonFxSim(mConstants.getBackLeftModuleConfig(), constants);
      mBrModule = new SwerveModuleIOTalonFxSim(mConstants.getBackRightModuleConfig(), constants);
    }

    mModules = new SwerveModuleIO[] {mFrModule, mFlModule, mBlModule, mBrModule};
  }

  @Override
  public void periodic() {
    // Update odometry /w measurements
    mRobotState.addOdometryMeasurement(
        new OdometryMeasurement(
            Timer.getTimestamp(),
            getSwerveModulePositions(),
            Robot.isReal() ? Optional.of(getRotation3d()) : Optional.empty()));

    // Update Swerve Module Signals
    for (var module : mModules) {
      module.periodic();
    }

    // Stop dt when disabled
    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  @Override
  public void simulationPeriodic() {
    // Rotate simulated gyro by omega, but only if omega exists
    mSimRotation2d =
        mSimRotation2d.rotateBy(
            Rotation2d.fromRadians(
                !Double.isNaN(getRobotRelativeChassisSpeeds().omegaRadiansPerSecond)
                    ? getRobotRelativeChassisSpeeds().omegaRadiansPerSecond * 0.02
                    : 0.0));
  }

  /**
   * @return {@link Rotation2d} representing measured yaw rotation
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Robot.isReal() ? mGyro.getRotation2d() : mSimRotation2d;
  }

  /**
   * @return {@link Rotation3d} representing the measured orientation of the robot in 3D space
   */
  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public Rotation3d getRotation3d() {
    return Robot.isReal() ? mGyro.getRotation3d() : new Rotation3d(mSimRotation2d);
  }

  /**
   * @return {@link Pose3d} representing field position WRT to alliance origin in 2D space
   */
  @Logged(name = "Pose2d", importance = Importance.INFO)
  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  /**
   * @return {@link Pose3d} representing field position WRT to alliance origin in 3D space
   */
  @Logged(name = "Pose3d", importance = Importance.INFO)
  public Pose3d getPose3d() {
    return mRobotState.getPose3d();
  }

  public LinearVelocity getLinearVelocity() {
    var speeds = getRobotRelativeChassisSpeeds();
    return MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getRotation2d());
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return mKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    // Find SwerveModuleIO equivalent of position getter method. This implementation is for ModuleIO
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getSwerveModulePosition)
        .toArray(SwerveModulePosition[]::new);
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getSwerveModuleState)
        .toArray(SwerveModuleState[]::new);
  }

  public SwerveModuleState[] getSetpointSwerveModuleStates() {
    return null; // Arrays.stream(mModules).map(SwerveModuleIO::desiredState).toArray(SwerveModuleState[]::new);
  }

  /**
   * @return true if FOC is enabled
   */
  public boolean isFocEnabled() {
    return mIsFocAutoToggleEnabled
        && getLinearVelocity()
            .gt(mConstants.getMaxLinearVelocity().times(mConstants.getFocAutoToggleMagnitude()));
  }

  public Command driveTeleop(
      DoubleSupplier xSpeedMag,
      DoubleSupplier ySpeedMag,
      DoubleSupplier omegaSpeedMag,
      BooleanSupplier isSpeedReduced,
      double reducedSpeedMag) {
    return driveTeleop(xSpeedMag, ySpeedMag, omegaSpeedMag, isSpeedReduced, reducedSpeedMag, false);
  }

  public Command driveTeleop(
      DoubleSupplier xSpeedMag,
      DoubleSupplier ySpeedMag,
      DoubleSupplier omegaSpeedMag,
      BooleanSupplier isSpeedReduced,
      double reducedSpeedMag,
      boolean openLoopEnabled) {
    return Commands.none();
  }

  /**
   * Auto FOC toggle is a system that disables Field Oriented Control once servos reach a certain
   * magnitude of the maximum drivetrain speed. This allows the drivetrain to reach its maximum
   * possible speed
   *
   * <p><strong> WARNING </strong> ~ FOC is required to control wheel torques; You must disable auto
   * toggle if you wish to accurate control torques
   *
   * @param enabled when true, FOC will automatically toggle on/off to maximize velocity
   */
  protected void setFocAutoToggleStatus(boolean enabled) {
    mIsFocAutoToggleEnabled = enabled;
  }

  protected void setChassisSpeedsSetpoint(ChassisSpeeds speeds, boolean openLoopEnabled) {}

  /** Stop drivetrain completely with the wheel forming an X */
  public void xStop() {}

  /** Stop drivetrain completely */
  public void stop() {
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].stop();
    }
  }
}
