// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.util.Tracer;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.GyroIONone;
import org.frc6423.robot.subsystem.drive.component.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOComp;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOSim;
import org.frc6423.robot.subsystem.drive.constant.SwerveConstants;

/** {@link SubsystemBase} Manager class for the swerve drivetrain */
public class DriveSubsystem extends SubsystemBase {
  /**
   * Static Factory for automatically configuring and creating a {@link DriveSubsystem}
   *
   * @return {@link DriveSubsystem}
   */
  public static DriveSubsystem create() {
    return (Robot.isReal())
        ? new DriveSubsystem(
            new GyroIOPigeon2(kConstants.getGyroConfig()),
            new SwerveModuleIOComp(kConstants.getFrontLeftModuleConfig()),
            new SwerveModuleIOComp(kConstants.getFrontRightModuleConfig()),
            new SwerveModuleIOComp(kConstants.getBackLeftModuleConfig()),
            new SwerveModuleIOComp(kConstants.getBackRightModuleConfig()))
        : new DriveSubsystem(
            new GyroIONone(),
            new SwerveModuleIOSim(kConstants.getFrontLeftModuleConfig()),
            new SwerveModuleIOSim(kConstants.getFrontRightModuleConfig()),
            new SwerveModuleIOSim(kConstants.getBackLeftModuleConfig()),
            new SwerveModuleIOSim(kConstants.getBackRightModuleConfig()));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link SwerveConstants} Physical/hardware constants to initialize subsystem with */
  public static final SwerveConstants kConstants = Flags.kDrivetrainContants;

  /** {@link Lock} Mutex for high frequency odometry */
  public static Lock kLock = new ReentrantLock();

  /** {@link String} The directory to store tunables in NT */
  public static final String kTunablesPrefix = "/Drive";

  // * ~~~~~~~~ TUNNABLES ~~~~~~~~

  private static final TunableNumber kTranslationalFeedbackKp =
      new TunableNumber(kTunablesPrefix + "/Trans kP");
  private static final TunableNumber kTranslationalFeedbackKi =
      new TunableNumber(kTunablesPrefix + "/Trans kI");
  private static final TunableNumber kTranslationalFeedbackKd =
      new TunableNumber(kTunablesPrefix + "/Trans kD");
  private static final TunableNumber kTranslationalFeedbackTolerance =
      new TunableNumber(kTunablesPrefix + "/tolerance (centimeters)");

  private static final TunableNumber kAngularFeedbackKp =
      new TunableNumber(kTunablesPrefix + "/Angular kP");
  private static final TunableNumber kAngularFeedbackKi =
      new TunableNumber(kTunablesPrefix + "/Angular kI");
  private static final TunableNumber kAngularFeedbackKd =
      new TunableNumber(kTunablesPrefix + "/Angular kd");
  private static final TunableNumber kAngularFeebackToleranceDeg =
      new TunableNumber(kTunablesPrefix + "/tolerance (degrees)");

  static {
    if (Robot.isReal()) {
      kTranslationalFeedbackKp.initDefault(10.0);
      kTranslationalFeedbackKi.initDefault(0.01);
      kTranslationalFeedbackKd.initDefault(0.02);
      kTranslationalFeedbackTolerance.initDefault(2.54);

      kAngularFeedbackKp.initDefault(5.0);
      kAngularFeedbackKi.initDefault(0.0);
      kAngularFeedbackKd.initDefault(0.08);
      kAngularFeebackToleranceDeg.initDefault(2.0);
    } else {
      kTranslationalFeedbackKp.initDefault(10.0);
      kTranslationalFeedbackKi.initDefault(0.01);
      kTranslationalFeedbackKd.initDefault(0.02);
      kTranslationalFeedbackTolerance.initDefault(2.54);

      kAngularFeedbackKp.initDefault(5.0);
      kAngularFeedbackKi.initDefault(0.0);
      kAngularFeedbackKd.initDefault(0.08);
      kAngularFeebackToleranceDeg.initDefault(2.0);
    }
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  /** {@link GyroIO} Hardware Interface for the gyro */
  private final GyroIO mGyro;

  /** {@link Rotation2d} Simulated robot rotation for sim */
  private Rotation2d mSimRotation = Rotation2d.kZero;

  /** {@link SwerveModuleIO} Hardware Interface for the front right swerve module */
  @Logged(name = "Front Right")
  private final SwerveModuleIO mFrontRightModule;

  /** {@link SwerveModuleIO} Hardware Interface for the back right swerve module */
  @Logged(name = "Back Right")
  private final SwerveModuleIO mBackRightModule;

  /** {@link SwerveModuleIO} Hardware Interface for the front left swerve module */
  @Logged(name = "Front Left")
  private final SwerveModuleIO mFrontLeftModule;

  /** {@link SwerveModuleIO} Hardware Interface for the back left swerve module */
  @Logged(name = "Back Left")
  private final SwerveModuleIO mBackLeftModule;

  /**
   * {@link Array} of {@link SwerveModuleIO} Array storing all {@link SwerveModuleIO} hardware for
   * easier manipulation
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   */
  private final SwerveModuleIO[] mModules;

  /**
   * {@link Array} of {@link SwerveModuleState} Array for storing setpoint {@link SwerveModuleIO}
   * states for logging
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   */
  private SwerveModuleState[] mSetpointWheelStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  /**
   * {@link SwerveDrivePoseEstimator} Odometry class for estimating the position of robot using
   * vision/drivetrain measurements
   */
  private final SwerveDrivePoseEstimator mPoseEstimator;

  /** {@link Field2d} Member for visualizing relevant positions on Elastic */
  private final Field2d mF2d;

  /**
   * {@link ProfiledPIDController} Feedback controller for translational assists in the x direction
   */
  private final ProfiledPIDController mTranslationalXController =
      new ProfiledPIDController(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get(),
          new Constraints(
              kConstants.getMaxLinearVelocityMetersPerSecond(),
              kConstants.getMaxLinearAccelerationMetersPerSecondPerSecond()));

  /**
   * {@link ProfiledPIDController} Feedback controller for translational assists in the y direction
   */
  private final ProfiledPIDController mTranslationalYController =
      new ProfiledPIDController(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get(),
          new Constraints(
              kConstants.getMaxLinearVelocityMetersPerSecond(),
              kConstants.getMaxLinearAccelerationMetersPerSecondPerSecond()));

  /** {@link ProfiledPIDController} Feedback controller for angular assists */
  private final ProfiledPIDController mAngularController =
      new ProfiledPIDController(
          kAngularFeedbackKp.get(),
          kAngularFeedbackKi.get(),
          kAngularFeedbackKd.get(),
          new Constraints(
              kConstants.getMaxAngularVelocityRadsPerSec(),
              kConstants.getAngularAccelerationRadsPerSecPerSec()));

  protected DriveSubsystem(
      GyroIO gyro,
      SwerveModuleIO frontRight,
      SwerveModuleIO backRight,
      SwerveModuleIO frontLeft,
      SwerveModuleIO backLeft) {
    mGyro = gyro;

    mFrontRightModule = frontRight;
    mBackRightModule = backRight;
    mFrontLeftModule = frontLeft;
    mBackLeftModule = backLeft;

    mModules =
        new SwerveModuleIO[] {
          mFrontRightModule, mBackRightModule, mFrontLeftModule, mBackLeftModule
        };

    mPoseEstimator =
        new SwerveDrivePoseEstimator(
            kConstants.getKinematics(), getGyroRotation2d(), getWheelPositions(), new Pose2d());
    // VecBuilder.fill(0.6, 0.6, 0.07),
    // VecBuilder.fill(0.9, 0.9, 0.4));
    mGyro.reset(Rotation2d.kZero);

    mAngularController.enableContinuousInput(0.0, 2 * Math.PI);

    mF2d = new Field2d();
    SmartDashboard.putData(mF2d);
  }

  @Override
  public void periodic() {
    for (var module : mModules) {
      module.periodic();
    }

    Tracer.traceFunc(
        "Update Odometry",
        () -> {
          mPoseEstimator.update(getGyroRotation2d(), getWheelPositions());
        });

    mF2d.setRobotPose(getPose2d());

    // Update tunables
    if (kTranslationalFeedbackKp.hasChanged(hashCode())
        || kTranslationalFeedbackKi.hasChanged(hashCode())
        || kTranslationalFeedbackKd.hasChanged(hashCode())
        || kAngularFeedbackKp.hasChanged(hashCode())
        || kAngularFeedbackKi.hasChanged(hashCode())
        || kAngularFeedbackKd.hasChanged(hashCode())) {
      mTranslationalXController.setPID(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get());
      mTranslationalYController.setPID(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get());

      mAngularController.setPID(
          kAngularFeedbackKp.get(), kAngularFeedbackKi.get(), kAngularFeedbackKd.get());

      resetFeedbackControllers();
    }
  }

  @Override
  public void simulationPeriodic() {
    mSimRotation =
        mSimRotation.rotateBy(
            Rotation2d.fromRadians(
                !Double.isNaN(getChassisSpeeds().omegaRadiansPerSecond)
                    ? getChassisSpeeds().omegaRadiansPerSecond * 0.02
                    : 0));
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Check if robot has approximately reached current setpoint translational position
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Within Translational Tolerance (bool)", importance = Importance.INFO)
  public boolean isAtTranslationalTarget() {
    return mTranslationalXController.atSetpoint() && mTranslationalYController.atSetpoint();
  }

  /**
   * Check if robot has approximately reached current setpoint angular position
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Within Angular Tolerance (bool)", importance = Importance.INFO)
  public boolean isFacingAngularTarget() {
    return mAngularController.atSetpoint();
  }

  /**
   * Get estimated yaw rotation of robot
   *
   * @return {@link Rotation2dd}
   */
  public Rotation2d getGyroRotation2d() {
    return (Robot.isReal()) ? mGyro.getYawRotation2d() : mSimRotation;
  }

  public Rotation2d getRotation2d() {
    return getPose2d().getRotation();
  }

  /**
   * Get estimated position of robot in 2-Dimensional space (x, y)
   *
   * @return {@link Pose2d}
   */
  @Logged(name = "Field Position (Pose2d)", importance = Importance.INFO)
  public Pose2d getPose2d() {
    return mPoseEstimator.getEstimatedPosition();
  }

  /**
   * Get measured speed (meters per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Speed (meters per second)", importance = Importance.INFO)
  public double getSpeedMetersPerSecond() {
    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
  }

  /**
   * Get velocity components of robot WRT field (x, y, omega rate)
   *
   * @return {@link ChassisSpeeds}
   */
  public ChassisSpeeds getChassisSpeedsWrtField() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
  }

  /**
   * Get velocity components of robot (dx, dy, omega rate)
   *
   * @return {@link ChassisSpeeds}
   */
  @Logged(name = "Velocity Components (ChassisSpeeds)", importance = Importance.INFO)
  public ChassisSpeeds getChassisSpeeds() {
    return kConstants.getKinematics().toChassisSpeeds(getWheelStates());
  }

  /**
   * Get array measured wheel positions
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModulePosition}
   */
  @Logged(name = "Wheel Positions (SwerveModulePosition)", importance = Importance.INFO)
  public SwerveModulePosition[] getWheelPositions() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getWheelPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Get array measured wheel states (velocity vectors)
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModuleState}
   */
  @Logged(name = "Swerve Module Velocity States (SwerveModuleState)", importance = Importance.INFO)
  public SwerveModuleState[] getWheelStates() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getWheelState)
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * Get array setpoint wheel states (velocity vectors)
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModuleState}
   */
  @Logged(
      name = "Setpoint Swerve Module Velocity States (SwerveModuleState)",
      importance = Importance.INFO)
  public SwerveModuleState[] getWheelSetpointStates() {
    return mSetpointWheelStates;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  public void addVisionMeasurement(Pose2d estimate, double timestamp, Matrix<N3, N1> stdevs) {
    mPoseEstimator.addVisionMeasurement(estimate, timestamp, stdevs);
  }

  public void resetFeedbackControllers() {
    mTranslationalXController.reset(
        getPose2d().getX(), getChassisSpeedsWrtField().vxMetersPerSecond);
    mTranslationalYController.reset(
        getPose2d().getY(), getChassisSpeedsWrtField().vyMetersPerSecond);
    mAngularController.reset(
        getRotation2d().getRadians(), getChassisSpeedsWrtField().omegaRadiansPerSecond);
  }

  /**
   * Set setpoint {@link ChassisSpeeds} for drivetrain to follow
   *
   * @param setpoint {@link ChassisSpeeds} Desired drivetrain speeds
   */
  public void setChassisSpeedsSetpoint(ChassisSpeeds setpoint) {
    // Generate a time-specific setpoint from continous-time speeds
    setpoint = ChassisSpeeds.discretize(setpoint, 0.02);

    // Convert to SwerveModuleState setpoints & clamp their velocities
    var states = kConstants.getKinematics().toSwerveModuleStates(setpoint);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, kConstants.getMaxLinearVelocityMetersPerSecond());

    // Auto FOC Toggle calculations;
    // if drivetrain is under 90% of max speed, FOC should be enabled
    // to maximize acceleration
    var focEnabled =
        getSpeedMetersPerSecond()
            < (kConstants.getMaxLinearVelocityMetersPerSecond()
                * kConstants.getFocAutoToggleMagnitude());

    // Send setpoints
    setSetpointWheelStates(states, focEnabled);
  }

  /**
   * Get a {@link Consumer} for driving based off {@link SwerveSample}
   *
   * @return {@link Consumer} of {@link SwerveSample}
   */
  @SuppressWarnings("resource")
  public Consumer<SwerveSample> getChoreoSwerveSampleConsumer() {
    final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    final PIDController angularController = new PIDController(6.0, 0.0, 0.0);
    angularController.enableContinuousInput(-Math.PI, Math.PI);

    return (sample) -> {
      // Get sample velocities & feedback velocities
      var fbSpeedsWrtField =
          new ChassisSpeeds(
              xController.calculate(getPose2d().getX(), sample.x),
              yController.calculate(getPose2d().getY(), sample.y),
              angularController.calculate(getRotation2d().getRadians(), sample.heading));

      var speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.getChassisSpeeds().plus(fbSpeedsWrtField), getRotation2d());

      var states = kConstants.getKinematics().toSwerveModuleStates(speeds);

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
        var torque = force.dot(forceDirection) * kConstants.getWheelRadiusMeters();

        // Send setpoint
        // mModules[i].setSetpoint(states[i], torque);
        // TODO fix and use torque
        mModules[i].setSetpoint(states[i], true);
      }

      // Log setpoints
      mSetpointWheelStates = states;
    };
  }

  public void setSetpointWheelStates(SwerveModuleState[] desiredStates, double[] wheelTorquesNm) {
    mSetpointWheelStates = desiredStates;

    for (int i = 0; i < mModules.length; i++) {
      mModules[i].setSetpoint(desiredStates[i], wheelTorquesNm[i]);
    }
  }

  public void setSetpointWheelStates(SwerveModuleState[] desiredStates, boolean focEnabled) {
    mSetpointWheelStates = desiredStates;

    for (int i = 0; i < mModules.length; i++) {
      mModules[i].setSetpoint(desiredStates[i], focEnabled);
    }
  }

  public void reset(Pose2d pose) {
    mPoseEstimator.resetPosition(getGyroRotation2d(), getWheelPositions(), pose);
  }

  /** Stop drivetrain completely */
  public void stop() {
    mSetpointWheelStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    for (int i = 0; i < mModules.length; i++) {
      mModules[i].neutral();
    }
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  public Command driveTeleoperatedFacingTarget(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Translation2d> target, boolean flipped) {
    return driveTeleoperatedWithAngularAsisst(
        vx,
        vy,
        () ->
            target
                .get()
                .minus(getPose2d().getTranslation())
                .getAngle()
                .plus(flipped ? Rotation2d.k180deg : Rotation2d.kZero));
  }

  public Command driveTeleoperatedWithAngularAsisst(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Rotation2d> angle) {
    return driveTeleoperated(
            vx,
            vy,
            () ->
                mAngularController.calculate(getRotation2d().getRadians(), angle.get().getRadians())
                    + mAngularController.getSetpoint().velocity)
        // mAngularController.calculate(
        //     getRotation2d().getRadians(), angle.get().getRadians()))
        .beforeStarting(() -> resetFeedbackControllers());
  }

  public Command driveTeleoperated(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(
        () ->
            setChassisSpeedsSetpoint(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx.getAsDouble(),
                    vy.getAsDouble(),
                    omega.getAsDouble(),
                    getRotation2d().plus(Flags.getAllianceRotation()))));
  }
}
