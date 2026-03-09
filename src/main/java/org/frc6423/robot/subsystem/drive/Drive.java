// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.util.Tracer;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Rebuilt;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.RobotState.OdometryMeasurement;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.GyroIONone;
import org.frc6423.robot.subsystem.drive.component.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOComp;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOSim;
import org.frc6423.robot.subsystem.drive.constants.SwerveConstants;

public class Drive extends SubsystemBase {
  /**
   * Create new {@link Drive}
   *
   * @return {@link Drive}
   */
  public static Drive create() {
    return (Robot.isReal())
        ? new Drive(
            new GyroIOPigeon2(kConstants.getGyroConfig()),
            new SwerveModuleIOComp(kConstants.getFrontRightModuleConfig()),
            new SwerveModuleIOComp(kConstants.getBackRightModuleConfig()),
            new SwerveModuleIOComp(kConstants.getFrontLeftModuleConfig()),
            new SwerveModuleIOComp(kConstants.getBackLeftModuleConfig()))
        : new Drive(
            new GyroIONone(),
            new SwerveModuleIOSim(kConstants.getFrontRightModuleConfig()),
            new SwerveModuleIOSim(kConstants.getBackRightModuleConfig()),
            new SwerveModuleIOSim(kConstants.getFrontLeftModuleConfig()),
            new SwerveModuleIOSim(kConstants.getBackLeftModuleConfig()));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link SwerveConstants} Physical/hardware constants to initialize subsystem with */
  public static final SwerveConstants kConstants = Flags.kDriveConstants;

  /** {@link Lock} Mutex for high frequency odometry */
  public static Lock kLock = new ReentrantLock();

  /** {@link String} The directory to store tunables in NT */
  public static final String kTunablesPrefix = "/drive";

  /** {@link Enum} Method of control drivetrain is currently being controlled by */
  public static enum ControlMode {
    BRAKE,
    TELEOPERATED_FULL,
    TELEOPERATED_ASSIST,
    TELEOPERATED_ASSIST_ANGULAR,
    AUTONOMOUS
  }

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  public static final TunableNumber kTranslationalKp =
      new TunableNumber(kTunablesPrefix + "Translational kP");
  public static final TunableNumber kTranslationalKi =
      new TunableNumber(kTunablesPrefix + "Translational kI");
  public static final TunableNumber kTranslationalKd =
      new TunableNumber(kTunablesPrefix + "Translational kD");
  public static final TunableNumber kTranslationalToleranceCm =
      new TunableNumber(kTunablesPrefix + "Translational Tolernace (centimeters)");

  public static final TunableNumber kAngularKp = new TunableNumber(kTunablesPrefix + "Angular kP");
  public static final TunableNumber kAngularKi = new TunableNumber(kTunablesPrefix + "Angular kI");
  public static final TunableNumber kAngularKd = new TunableNumber(kTunablesPrefix + "Angular kD");
  public static final TunableNumber kAngularToleranceDeg =
      new TunableNumber(kTunablesPrefix + "Angular Tolerance (degrees)");

  static {
    if (Robot.isReal()) {
      kTranslationalKp.initDefault(4.0);
      kTranslationalKi.initDefault(0.0);
      kTranslationalKd.initDefault(0.05);
      kTranslationalToleranceCm.initDefault(1.0);

      kAngularKp.initDefault(11.5);
      kAngularKi.initDefault(0.0);
      kAngularKd.initDefault(0.05);
      kAngularToleranceDeg.initDefault(2.0);
    } else {
      kTranslationalKp.initDefault(4.0);
      kTranslationalKi.initDefault(0.0);
      kTranslationalKd.initDefault(0.05);
      kTranslationalToleranceCm.initDefault(1.0);

      kAngularKp.initDefault(4.5);
      kAngularKi.initDefault(0.0);
      kAngularKd.initDefault(0.05);
      kAngularToleranceDeg.initDefault(2.0);
    }
  }

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

  private SwerveModuleState[] mSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private final Field2d mF2d;

  private ControlMode mControlMode = ControlMode.BRAKE;

  private final PIDController mTranslationalXController,
      mTranslationalYController,
      mAngularController;

  protected Drive(
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

    mTranslationalXController =
        new PIDController(kTranslationalKp.get(), kTranslationalKi.get(), kTranslationalKd.get());
    mTranslationalXController.setTolerance(kTranslationalToleranceCm.get() / 100.0);

    mTranslationalYController =
        new PIDController(kTranslationalKp.get(), kTranslationalKi.get(), kTranslationalKd.get());
    mTranslationalYController.setTolerance(kTranslationalToleranceCm.get() / 100.0);

    mAngularController = new PIDController(kAngularKp.get(), kAngularKi.get(), kAngularKd.get());
    mAngularController.setTolerance(Units.degreesToRadians(kAngularToleranceDeg.get()));
    mAngularController.enableContinuousInput(-Math.PI, Math.PI);

    mF2d = new Field2d();

    if (Robot.isSimulation()) RobotState.getInstance().resetPose(Rebuilt.kMidPose);

    setDefaultCommand(brake());
  }

  @Override
  public void periodic() {
    Tracer.traceFunc(
        "Update Odometry",
        () -> {
          RobotState.getInstance()
              .addOdometryMeasurement(
                  new OdometryMeasurement(
                      Timer.getFPGATimestamp(),
                      getWheelPositions(),
                      Robot.isReal()
                          ? Optional.of(Rotation2d.fromDegrees(mGyro.getYawDegrees()))
                          : Optional.empty()));
          RobotState.getInstance().setChassisSpeeds(getChassisSpeeds());
        });

    // Update tunables if needed
    if (kTranslationalKp.hasChanged(hashCode())
        || kTranslationalKi.hasChanged(hashCode())
        || kTranslationalKd.hasChanged(hashCode())
        || kTranslationalToleranceCm.hasChanged(hashCode())) {
      mTranslationalXController.setPID(
          kTranslationalKp.get(), kTranslationalKi.get(), kTranslationalKd.get());
      mTranslationalXController.setTolerance(kTranslationalToleranceCm.get() / 100.0);
      mTranslationalYController.setPID(
          kTranslationalKp.get(), kTranslationalKi.get(), kTranslationalKd.get());
      mTranslationalYController.setTolerance(kTranslationalToleranceCm.get() / 100.0);
    }

    if (kAngularKp.hasChanged(hashCode())
        || kAngularKi.hasChanged(hashCode())
        || kAngularKd.hasChanged(hashCode())) {
      mAngularController.setPID(kAngularKp.get(), kAngularKi.get(), kAngularKd.get());
    }

    mF2d.setRobotPose(getPose2d());
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Get {@link ControlMode} being used to control drivetrain
   *
   * @return {@link ControlMode}
   */
  @Logged(name = "Method of Control", importance = Importance.INFO)
  public ControlMode getControlMode() {
    return mControlMode;
  }

  /**
   * Check if robot is facing towards current aim target
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Facing Target (bool)", importance = Importance.INFO)
  public boolean isFacingTarget() {
    return mAngularController.atSetpoint();
  }

  /**
   * Get {@link RobotState} estimated yaw rotation of robot
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return getPose2d().getRotation();
  }

  /**
   * Get {@link RobotState} estimated position of robot on field
   *
   * @return {@link Pose2d}
   */
  @Logged(name = "Pose2d", importance = Importance.INFO)
  public Pose2d getPose2d() {
    return RobotState.getInstance().getEstimatedPosition();
  }

  /**
   * Get measured speed of robot
   *
   * @return {@link Double}
   */
  @Logged(name = "Speed (meters per second)", importance = Importance.INFO)
  public double getSpeedMetersPerSecond() {
    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
  }

  /**
   * Get x, y, and rotational speeds of drivetrain relative to field view
   *
   * @return {@link ChassisSpeeds}
   */
  @Logged(name = "ChassisSpeeds (field relative)", importance = Importance.INFO)
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
  }

  /**
   * Get x, y, and rotational speeds of drivetrain
   *
   * @return {@link ChassisSpeeds}
   */
  @Logged(name = "ChassisSpeeds", importance = Importance.INFO)
  public ChassisSpeeds getChassisSpeeds() {
    return kConstants.getKinematics().toChassisSpeeds(getWheelStates());
  }

  /**
   * Get the positions of drive wheels (distance vectors)
   *
   * @return {@link Array} of {@link SwerveModulePosition}
   */
  @Logged(name = "Swerve Module Positions", importance = Importance.INFO)
  public SwerveModulePosition[] getWheelPositions() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getWheelPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Get the states of drive wheels (velocity vectors)
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModuleState}
   */
  @Logged(name = "Swerve Module States", importance = Importance.INFO)
  public SwerveModuleState[] getWheelStates() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getWheelState)
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * Get setpoint states of drive wheels
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModuleState}
   */
  @Logged(name = "Setpoint Swerve Module States", importance = Importance.INFO)
  public SwerveModuleState[] getWheelSetpointStates() {
    return mSetpointStates;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  /**
   * Set x, y, and rotational speed setpoints for drivetrain to achive
   *
   * @param setpoint {@link ChassisSpeeds} Setpoint speeds
   */
  public void setChassisSpeedsSetpoint(ChassisSpeeds setpoint) {
    // Generate a time-specific setpoint from continous-time speeds
    setpoint = ChassisSpeeds.discretize(setpoint, 0.02);

    // Convert to SwerveModuleState setpoints & clamp their velocities
    var states = kConstants.getKinematics().toSwerveModuleStates(setpoint);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, kConstants.getMaxLinearVelocityMetersPerSecond());

    // Auto FOC Toggle calculations
    var focEnabled =
        getSpeedMetersPerSecond()
            > (kConstants.getMaxLinearVelocityMetersPerSecond()
                * kConstants.getFocAutoToggleMagnitude());

    // Send setpoints
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].setSetpoint(states[i], focEnabled);
    }

    // Log setpoints
    mSetpointStates = states;
  }

  /**
   * Get Choreo autonomous controller
   *
   * @return {@link Consumer} of {@link SwerveSample}
   */
  public Consumer<SwerveSample> getChoreoSwerveSampleConsumer() {
    return (sample) -> {
      // Get sample velocities & feedback velocities
      var speeds = sample.getChassisSpeeds();
      var feedbackSpeeds =
          new ChassisSpeeds(
              mTranslationalYController.calculate(getPose2d().getX(), sample.x),
              mTranslationalYController.calculate(getPose2d().getY(), sample.y),
              mAngularController.calculate(getRotation2d().getRadians(), sample.heading));

      // Create full velocities & convert to states
      speeds = speeds.plus(feedbackSpeeds);
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
        mModules[i].setSetpoint(states[i], torque);
      }

      // Log setpoints
      mSetpointStates = states;
      mControlMode = ControlMode.AUTONOMOUS;
    };
  }

  /** Stop drivetrain completely */
  public void stop() {
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].neutral();
    }
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Drive robot from field relative x & y speeds while facing a specified target
   *
   * @param vx {@link DoubleSupplier} X speed stream
   * @param vy {@link DoubleSupplier} Y speed stream
   * @param target {@link Translation2d} Coordinates of target on field
   * @return {@link Command}
   */
  public Command driveWhileFacingTarget(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Translation2d> target) {
    return driveWhileFacingAngle(
        vx, vy, () -> target.get().minus(getPose2d().getTranslation()).getAngle());
  }

  /**
   * Drive robot from field relative x & y speeds while facing a specified heading
   *
   * @param vx {@link DoubleSupplier} X speed stream
   * @param vy {@link DoubleSupplier} Y speed stream
   * @param heading {@link Supplier} of {@link Rotation2d} Heading stream
   * @return {@link Command}
   */
  public Command driveWhileFacingAngle(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Rotation2d> heading) {
    return this.run(
        () ->
            setChassisSpeedsSetpoint(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx.getAsDouble() * kConstants.getMaxLinearVelocityMetersPerSecond(),
                    vy.getAsDouble() * kConstants.getMaxLinearVelocityMetersPerSecond(),
                    mAngularController.calculate(
                        getRotation2d().getRadians(), heading.get().getRadians()),
                    getRotation2d().plus(Flags.getAllianceRotation()))));
  }

  /**
   * Drive robot from field relative x, y, and omega speeds
   *
   * @param vx {@link DoubleSupplier} X speed stream
   * @param vy {@link DoubleSupplier} Y speed stream
   * @param omega {@link DoubleSupplier} omega speed stream
   * @return {@link Command}
   */
  public Command drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return this.run(
            () ->
                setChassisSpeedsSetpoint(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx.getAsDouble() * kConstants.getMaxLinearVelocityMetersPerSecond(),
                        vy.getAsDouble() * kConstants.getMaxLinearVelocityMetersPerSecond(),
                        omega.getAsDouble() * kConstants.getMaxAngularVelocityRadsPerSec(),
                        getRotation2d().plus(Flags.getAllianceRotation()))))
        .beforeStarting(() -> mControlMode = ControlMode.TELEOPERATED_FULL);
  }

  /**
   * Apply brake
   *
   * @return {@link Command}
   */
  public Command brake() {
    return this.run(() -> stop()).beforeStarting(() -> mControlMode = ControlMode.BRAKE);
  }
}
