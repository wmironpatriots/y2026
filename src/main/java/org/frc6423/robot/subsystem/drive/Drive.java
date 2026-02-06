// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
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
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIO.ControlMode;
import org.frc6423.robot.subsystem.drive.module.SwerveModuleIOTalonFx;

/** {@link SubsystemBase} extension representing swerve drivetrain */
public class Drive extends SubsystemBase {
  private final DriveConstants mConstants;

  private final RobotState mRobotState;
  private final SwerveDriveKinematics mKinematics;

  private final SwerveModuleIO[] mModules;
  private final GyroIO mGyro;

  private SwerveModuleState[] mSetpointStates;

  private Rotation2d mSimRotation2d = Rotation2d.kZero;

  /**
   * Create new {@link Drive}
   *
   * @param constants {@link DriveConstants} representing subsystem configuration
   */
  public Drive(DriveConstants constants) {
    mConstants = constants;

    mRobotState = RobotState.createInstance(constants);
    mKinematics = constants.getKinematics();

    var configs = constants.getModuleConfigs();
    mModules = new SwerveModuleIO[configs.length];
    mGyro = new GyroIOPigeon2(mConstants.getGyroConfig());

    for (int i = 0; i < configs.length; i++) {
      mModules[i] =
          Robot.isReal()
              ? new SwerveModuleIOTalonFx(configs[i].name(), configs[i], constants)
              : new SwerveModuleIOTalonFx(configs[i].name(), configs[i], constants);
    }

    mSetpointStates = new SwerveModuleState[mModules.length];
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
   * @return {@link LinearVelocity} representing the linear velocity of drivetrain
   */
  @Logged(name = "LinearVelocity", importance = Importance.INFO)
  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(
        Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond));
  }

  /**
   * @return {@link AngularVelocity} representing the angular velocity of drivetrain
   */
  @Logged(name = "AngularVelocity", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return RadiansPerSecond.of(getChassisSpeeds().omegaRadiansPerSecond);
  }

  /**
   * @return {@link SwerveModulePosition} representing the displacement vectors of swerve modules
   */
  @Logged(name = "SwerveModulePositions", importance = Importance.INFO)
  public SwerveModulePosition[] getSwerveModulePositions() {
    var poses = new SwerveModulePosition[mModules.length];
    for (int i = 0; i < poses.length; i++) {
      poses[i] = mModules[i].getSwerveModulePosition();
    }

    return poses;
  }

  /**
   * @return {@link SwerveModulePosition} representing the velocity vectors of swerve modules
   */
  @Logged(name = "SwerveModuleStates", importance = Importance.INFO)
  public SwerveModuleState[] getSwerveModuleStates() {
    var states = new SwerveModuleState[mModules.length];
    for (int i = 0; i < states.length; i++) {
      states[i] = mModules[i].getSwerveModuleState();
    }

    return states;
  }

  /**
   * @return {@link SwerveModuleState} array representing the setpoint velocity vectors of swerve
   *     modules
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public SwerveModuleState[] getSetpointSwerveModuleStates() {
    return mSetpointStates;
  }

  /**
   * @return {@link ChassisSpeeds} representing the drive speeds
   */
  @Logged(name = "ChassisSpeeds", importance = Importance.INFO)
  public ChassisSpeeds getChassisSpeeds() {
    return mKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * Set a velocity setpoint to optimized and run
   *
   * @param speeds {@link ChassisSpeeds} representing velocity setpoint
   * @param openLoopEnabled when true, open-loop controlled will be utilized
   */
  public void setChassisSpeedsSetpoint(ChassisSpeeds speeds, boolean openLoopEnabled) {
    // Generate a time specific setpoint from continuous-time speeds
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert to module states and clamp velocity
    var states = mKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, mConstants.getMaxLinearVelocity());

    ControlMode mode;

    // Use FOC control until over 90% of max velocity
    if (getLinearVelocity().gt(mConstants.getMaxLinearVelocity().times(0.9))) {
      mode = openLoopEnabled ? ControlMode.OPEN_LOOP_VOLT_FOC : ControlMode.CLOSED_LOOP_TORQUE_FOC;
    } else {
      mode = openLoopEnabled ? ControlMode.OPEN_LOOP_VOLT : ControlMode.CLOSED_LOOP_VOLT;
    }

    for (int i = 0; i < mModules.length; i++) {
      mModules[i].setSetpoint(states[i], mode);
    }

    mSetpointStates = states;
  }

  /** Stop drivetrain completely */
  public void stop() {
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].stop();
    }

    mSetpointStates = new SwerveModuleState[mModules.length];
  }
}
