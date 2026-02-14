// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.lang.reflect.Array;
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

  private final PIDController mVelXController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController mVelYController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController mOmegaController = new PIDController(2.5, 0.0, 0.0);

  private final SysIdRoutine mWheelPivotCharacterization;

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

    mWheelPivotCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {
                  for (int i = 0; i < mModules.length; i++) {
                    mModules[i].setPivotTorqueCurrentFocSetpoint(Amps.of(Voltage.in(Volts)));
                  }
                },
                null,
                this,
                "SwervePivotSysId"));

    SmartDashboard.putData(
        "Swerve Pivot Characterization (Dynamic Forward)",
        mWheelPivotCharacterization.dynamic(Direction.kForward));
    SmartDashboard.putData(
        "Swerve Pivot Characterization (Dynamic Reverse)",
        mWheelPivotCharacterization.dynamic(Direction.kReverse));
    SmartDashboard.putData(
        "Swerve Pivot Characterization (Quasistatic Forward)",
        mWheelPivotCharacterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Swerve Pivot Characterization (Quasistatic Reverse)",
        mWheelPivotCharacterization.quasistatic(Direction.kReverse));
  }

  @Override
  public void periodic() {
    // Update Swerve Module Signals
    for (var module : mModules) {
      module.periodic();
    }

    // Stop dt when disabled
    if (DriverStation.isDisabled()) {
      xStop();
    }
  }

  /**
   * @return {@link Rotation2d} representing estimated chassis yaw orientation
   */
  private Rotation2d getEstimatedRotation2d() {
    return mRobotState.getRotation2d();
  }

  /**
   * @return {@link Pose3d} representing estimated chassis position in 2D space (x, y)
   */
  private Pose2d getEstimatedPose2d() {
    return mRobotState.getPose2d();
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
    var poses = new SwerveModulePosition[mModules.length];

    for (int i = 0; i < mModules.length; i++) {
      poses[i] = mModules[i].getSwerveModulePosition();
    }
    return poses;
  }

  /**
   * @return {@link SwerveModuleState} {@link Array} representing the velocity components of each
   *     swerve module
   */
  @Logged(name = "Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSwerveModuleStates() {
    var states = new SwerveModuleState[mModules.length];

    for (int i = 0; i < mModules.length; i++) {
      states[i] = mModules[i].getSwerveModuleState();
    }
    return states;
  }

  /**
   * @return {@link SwerveModuleState} {@link Array} representing the desired velocity components of
   *     each swerve module
   */
  @Logged(name = "Setpoint Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSetpointSwerveModuleStates() {
    return mSetpointStates;
  }

  /**
   * @return true if FOC is enabled
   */
  @Logged(name = "FOC enabled", importance = Importance.INFO)
  public boolean isFocEnabled() {
    return getLinearVelocity()
        .gt(mConstants.getMaxLinearVelocity().times(mConstants.getFocAutoToggleMagnitude()));
  }

  /**
   * Set a {@link ChassisSpeeds} setpoint
   *
   * @param speeds {@link ChassisSpeeds} representing the desired velocity components
   */
  protected void setChassisSpeedsSetpoint(ChassisSpeeds speeds) {
    // Generate a time-specific setpoint from continous-time speeds
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert to SwerveModuleState setpoints & clamp their velocities
    var states = mKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, mConstants.getMaxLinearVelocity());

    // Auto FOC Toggle calculations
    var focEnabled =
        !getLinearVelocity()
            .gt(mConstants.getMaxLinearVelocity().times(mConstants.getFocAutoToggleMagnitude()));

    // Send setpoints
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].setSetpoint(states[i], focEnabled);
    }

    // Log setpoints
    mSetpointStates = states;
  }

  /**
   * @return {@link Consumer} for running {@link SwerveSample} setpoints
   */
  public Consumer<SwerveSample> getSwerveSampleConsumer() {
    return (sample) -> {
      // Get sample velocities & feedback velocities
      var speeds = sample.getChassisSpeeds();
      var feedbackSpeeds =
          new ChassisSpeeds(
              mVelXController.calculate(getEstimatedPose2d().getX(), sample.x),
              mVelYController.calculate(getEstimatedPose2d().getY(), sample.y),
              mOmegaController.calculate(getEstimatedRotation2d().getRadians(), sample.heading));

      // Create full velocities & convert to states
      speeds = speeds.plus(feedbackSpeeds);
      var states = mKinematics.toSwerveModuleStates(speeds);

      // Get desired Module forces
      var xForces = sample.moduleForcesX();
      var yForces = sample.moduleForcesY();

      for (int i = 0; i < mModules.length; i++) {
        // Get desired angle of module
        var angle = states[i].angle;

        // Calculate desired force vector of module and account for chassis orientation
        var force =
            new Translation2d(xForces[i], yForces[i])
                .rotateBy(Rotation2d.fromRadians(sample.heading).unaryMinus())
                .toVector();
        var forceDirection = VecBuilder.fill(angle.getCos(), angle.getSin());

        // Convert desired force vector into wheel torque
        var torque =
            NewtonMeters.of(force.dot(forceDirection) * mConstants.getWheelRadius().in(Meters));

        // Send setpoint
        mModules[i].setSetpointWithWheelTorque(states[i], torque);
      }

      // Log setpoints
      mSetpointStates = states;
    };
  }

  /** Stop drivetrain completely with the wheel forming an X */
  public void xStop() {
    var states =
        new SwerveModuleState[] {
          new SwerveModuleState(InchesPerSecond.zero(), Rotation2d.fromDegrees(45)),
          new SwerveModuleState(InchesPerSecond.zero(), Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(InchesPerSecond.zero(), Rotation2d.fromDegrees(45)),
          new SwerveModuleState(InchesPerSecond.zero(), Rotation2d.fromDegrees(-45))
        };

    for (int i = 0; i < mModules.length; i++) {
      mModules[i].setSetpoint(states[i], false);
    }
  }

  /** Stop drivetrain completely */
  public void stop() {
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].stop();
    }
  }
}
