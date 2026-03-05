// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.frc6423.lib.util.Tracer;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.PositionEstimator.EncoderMeasurement;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.localization.Vision;

/** {@link SubsystemBase} The class controlling the swerve drivetrain subsystem */
public class Drive extends SubsystemBase {
  private static final DriveConstants mConstants = Flags.kRobotType.mDriveConstants;

  // * HARDWARE MEMBERS
  @Logged private final SwerveModuleIO mFrModule;
  @Logged private final SwerveModuleIO mFlModule;
  @Logged private final SwerveModuleIO mBlModule;
  @Logged private final SwerveModuleIO mBrModule;

  private final SwerveModuleIO[] mModules;
  private final GyroIO mGyro;

  private final Vision mVision = Vision.create();

  // * CONTROL MEMBERS
  private final PositionEstimator mPoseEstimator;

  private SwerveModuleState[] mSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private final PIDController mPositionXController, mPositionYController, mRotationController;

  /**
   * Create new {@link Drive}
   *
   * @param gyro {@link GyroIO} Gyro subsystem should use
   * @param frontRightModule {@link SwerveModuleIO} Front Right Swerve Module Hardware
   * @param frontLeftModule {@link SwerveModuleIO} Front Left Swerve Module Hardware
   * @param backLeftModule {@link SwerveModuleIO} Back Left Swerve Module Hardware
   * @param backRightModule {@link SwerveModuleIO} Back Right Swerve Module Hardwaree
   */
  public Drive(
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

    // Init controls
    mPoseEstimator =
        new PositionEstimator(
            mConstants.getKinematics(), VecBuilder.fill(0.6, 0.6, 0.6, 0.07), 0.0);

    mPositionXController = new PIDController(0.0, 0.0, 0.0);
    mPositionYController = new PIDController(0.0, 0.0, 0.0);
    mRotationController = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    Tracer.traceFunc(
        "Update Swerve Hardware",
        () -> {
          for (var module : mModules) {
            module.periodic();
          }

          mGyro.periodic();
        });

    Tracer.traceFunc(
        "Update Swerve Position Estimator",
        () -> {
          if (Robot.isReal())
            mPoseEstimator.addOdometryMeasurement(
                new EncoderMeasurement(
                    Timer.getFPGATimestamp(), getSwerveModulePositions(), Optional.empty()));
          else
            mPoseEstimator.addOdometryMeasurement(
                new EncoderMeasurement(
                    Timer.getFPGATimestamp(),
                    getSwerveModulePositions(),
                    Optional.of(mGyro.getRotation3d())));

          mPoseEstimator.update();
        });
  }

  // * COMMANDS
  public Command driveFromTeleoperatedInputs(
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier omegaSupplier) {
    return this.run(
        () ->
            setChassisSpeedsSetpoint(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    mConstants.getMaxLinearVelocity().times(xVelocitySupplier.getAsDouble()),
                    mConstants.getMaxLinearVelocity().times(yVelocitySupplier.getAsDouble()),
                    mConstants.getMaxAngularVelocity().times(omegaSupplier.getAsDouble()),
                    getRotation2d())));
  }

  public Command driveWhileFacing(
      DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, Pose2d pose2d) {
    return Commands.none();
  }

  public Command stop() {
    return this.run(
        () -> {
          for (var module : mModules) {
            module.stop();
          }
        });
  }

  public Command lock() {
    return Commands.none();
  }

  // * SETTERS
  /**
   * Reset drive position to a specified position
   *
   * @param pose {@link Pose2d} Position to reset to in 3-Dimensional Space
   */
  public void resetPosition(Pose2d pose) {
    mPoseEstimator.resetPose(pose);
  }

  private void setChassisSpeedsSetpoint(ChassisSpeeds speeds) {
    // Generate a time-specific setpoint from continous-time speeds
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert to SwerveModuleState setpoints & clamp their velocities
    var states = mConstants.getKinematics().toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, mConstants.getMaxLinearVelocity());

    // Auto FOC Toggle calculations
    var focEnabled =
        !getVelocity()
            .gt(mConstants.getMaxLinearVelocity().times(mConstants.getFocAutoToggleMagnitude()));

    // Send setpoints
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].setSetpointState(states[i], focEnabled);
    }

    // Log setpoints
    mSetpointStates = states;
  }

  // * GETTERS
  @Logged(name = "Is Colliding (bool)", importance = Importance.INFO)
  public boolean isColliding() {
    return mGyro.getAccelerationMetersPerSecondPerSecond().norm()
        > mConstants.getMaxLinearAcceleration().in(MetersPerSecondPerSecond);
  }

  @Logged(name = "Is Slipping (bool)", importance = Importance.INFO)
  public boolean isSlipping() {
    boolean is = false;
    var expectedStates = mConstants.getKinematics().toSwerveModuleStates(getChassisSpeeds());
    var actualState = getSwerveModuleStates();

    for (int i = 0; i < expectedStates.length; i++) {
      is =
          !MathUtil.isNear(
              expectedStates[i].speedMetersPerSecond, actualState[i].speedMetersPerSecond, 0.01);
    }

    return is;
  }

  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return getPose2d().getRotation();
  }

  @Logged(name = "Pose2d", importance = Importance.INFO)
  public Pose2d getPose2d() {
    return mPoseEstimator.getPose3d().toPose2d();
  }

  @Logged(name = "Linear Velocity (meters per second)", importance = Importance.INFO)
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(getVelocityMetersPerSecond().norm());
  }

  public Vector<N3> getVelocityWrtFieldMetersPerSecond() {
    return VecBuilder.fill(
        getChassisSpeedsWrtField().vxMetersPerSecond,
        getChassisSpeedsWrtField().vyMetersPerSecond,
        getChassisSpeedsWrtField().omegaRadiansPerSecond);
  }

  public Vector<N3> getSetpointVelocityMetersPerSecond() {
    return VecBuilder.fill(
        getSetpointChassisSpeeds().vxMetersPerSecond,
        getSetpointChassisSpeeds().vyMetersPerSecond,
        getSetpointChassisSpeeds().omegaRadiansPerSecond);
  }

  public Vector<N3> getVelocityMetersPerSecond() {
    return VecBuilder.fill(
        getChassisSpeeds().vxMetersPerSecond,
        getChassisSpeeds().vyMetersPerSecond,
        getChassisSpeeds().omegaRadiansPerSecond);
  }

  @Logged(name = "Chassis Speeds (wrt field)", importance = Importance.INFO)
  public ChassisSpeeds getChassisSpeedsWrtField() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
  }

  @Logged(name = "Setpoint Chassis Speeds", importance = Importance.INFO)
  public ChassisSpeeds getSetpointChassisSpeeds() {
    return mConstants.getKinematics().toChassisSpeeds(getSetpointSwerveModuleState());
  }

  @Logged(name = "Chassis Speeds", importance = Importance.INFO)
  public ChassisSpeeds getChassisSpeeds() {
    return mConstants.getKinematics().toChassisSpeeds(getSwerveModuleStates());
  }

  @Logged(name = "Swerve Module Positions", importance = Importance.INFO)
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] poses = new SwerveModulePosition[mModules.length];
    for (int i = 0; i < mModules.length; i++) {
      poses[i] = mModules[i].getSwerveModulePosition();
    }

    return poses;
  }

  @Logged(name = "Setpoint Swerve Module State", importance = Importance.INFO)
  public SwerveModuleState[] getSetpointSwerveModuleState() {
    return mSetpointStates;
  }

  @Logged(name = "Swerve Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[mModules.length];
    for (int i = 0; i < mModules.length; i++) {
      states[i] = mModules[i].getSwerveModuleState();
    }

    return states;
  }

  public Consumer<SwerveSample> getSwerveSampleConsumer() {
    return (sample) -> {
      // Get sample velocities & feedback velocities
      var speeds = sample.getChassisSpeeds();
      var feedbackSpeeds =
          new ChassisSpeeds(
              mPositionXController.calculate(getPose2d().getX(), sample.x),
              mPositionYController.calculate(getPose2d().getY(), sample.y),
              mRotationController.calculate(getRotation2d().getRadians(), sample.heading));

      // Create full velocities & convert to states
      speeds = speeds.plus(feedbackSpeeds);
      var states = mConstants.getKinematics().toSwerveModuleStates(speeds);

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
        mModules[i].setSetpointState(states[i], torque);
      }

      // Log setpoints
      mSetpointStates = states;
    };
  }
}
