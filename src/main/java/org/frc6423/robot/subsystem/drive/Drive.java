// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.frc6423.lib.util.Tracer;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.RobotState.OdometryMeasurement;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOTalonFx;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOTalonFxSim;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;

/** TODO WIP */
public class Drive extends SubsystemBase {
  private static final DriveConstants mConstants = Flags.kRobotType.mDriveConstants;

  /**
   * Create new {@link Drive}
   *
   * @param robotState {@link RobotState} Robot Tracker to send odometry measurements to
   * @return {@link Drive}
   */
  public static Drive create(RobotState robotState) {
    return (Robot.isReal())
        ? new Drive(
            robotState,
            new GyroIOPigeon2(
                mConstants.getGyroConfig().deviceId(),
                mConstants.getGyroConfig().canBus(),
                mConstants.getGyroConfig().config()),
            new SwerveModuleIOTalonFx(
                "Front Right", mConstants.getFrontRightModuleConfig(), mConstants),
            new SwerveModuleIOTalonFx(
                "Front Left", mConstants.getFrontLeftModuleConfig(), mConstants),
            new SwerveModuleIOTalonFx(
                "Back Left", mConstants.getBackLeftModuleConfig(), mConstants),
            new SwerveModuleIOTalonFx(
                "Back Right", mConstants.getBackRightModuleConfig(), mConstants))
        : new Drive(
            robotState,
            new GyroIOPigeon2(
                mConstants.getGyroConfig().deviceId(),
                mConstants.getGyroConfig().canBus(),
                mConstants.getGyroConfig().config()),
            new SwerveModuleIOTalonFxSim(
                "Front Right", mConstants.getFrontRightModuleConfig(), mConstants),
            new SwerveModuleIOTalonFxSim(
                "Front Left", mConstants.getFrontLeftModuleConfig(), mConstants),
            new SwerveModuleIOTalonFxSim(
                "Back Left", mConstants.getBackLeftModuleConfig(), mConstants),
            new SwerveModuleIOTalonFxSim(
                "Back Right", mConstants.getBackRightModuleConfig(), mConstants));
  }

  private final RobotState mRobotState;

  // * HARDWARE MEMBERS
  @Logged private final SwerveModuleIO mFrModule;
  @Logged private final SwerveModuleIO mFlModule;
  @Logged private final SwerveModuleIO mBlModule;
  @Logged private final SwerveModuleIO mBrModule;

  private final SwerveModuleIO[] mModules;
  private final GyroIO mGyro;

  // * CONTROL MEMBERS
  private SwerveModuleState[] mSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private final PIDController mPositionXController, mPositionYController, mRotationController;

  // * SYSID MEMBERS
  private final SysIdRoutine mPivotCharacterization,
      mDriveLinearCharacterization,
      mDriveAngularCharacterization;

  /**
   * Create new {@link Drive}
   *
   * @param robotState {@link RobotState} Robot Tracker to send odometry measurements to
   * @param gyro {@link GyroIO} Gyro for odometry
   * @param frontRightModule {@link SwerveModuleIO} Front Right Swerve Module
   * @param frontLeftModule {@link SwerveModuleIO} Front Left Swerve Module
   * @param backLeftModule {@link SwerveModuleIO} Back Left Swerve Module
   * @param backRightModule {@link SwerveModuleIO} Back Right Swerve Module
   */
  public Drive(
      RobotState robotState,
      GyroIO gyro,
      SwerveModuleIO frontRightModule,
      SwerveModuleIO frontLeftModule,
      SwerveModuleIO backLeftModule,
      SwerveModuleIO backRightModule) {
    mRobotState = robotState;

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
    mPositionXController = new PIDController(0.0, 0.0, 0.0);
    mPositionYController = new PIDController(0.0, 0.0, 0.0);
    mRotationController = new PIDController(0.0, 0.0, 0.0);

    // Init SysId
    mPivotCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(15).per(Second),
                Volts.of(35),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {
                  for (int i = 0; i < mModules.length; i++) {
                    mModules[i].setPivotCurrent(Amps.of(Voltage.in(Volts)));
                  }
                },
                null,
                this,
                "SwervePivotSysId"));

    mDriveLinearCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(15).per(Second),
                Volts.of(35),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {
                  for (var module : mModules) {
                    module.setSetpointState(
                        new SwerveModuleState(MetersPerSecond.zero(), Rotation2d.kZero), true);
                    module.setDriveCurrent(Amps.of(Voltage.in(Volts)));
                  }
                },
                null,
                this,
                "SwerveLinearSysId"));

    mDriveAngularCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(15).per(Second),
                Volts.of(35),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {}, // TODO
                null,
                this,
                "SwerveAngularSysId"));

    // Publish SysId commands if in tuning mode
    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData(runPivotCharacterizationSequence());
      SmartDashboard.putData(runLinearDriveCharacterization());
      SmartDashboard.putData(runAngularDriveCharacterization());
    }
  }

  @Override
  public void periodic() {
    Tracer.traceFunc(
        "Swerve Full Periodic",
        () -> {
          // Update Hardware
          for (var module : mModules) {
            Tracer.traceFunc("Swerve Update Module " + module.mName, module::periodic);
          }

          // Send odometry measurements to robot state
          Tracer.traceFunc(
              "Swerve Update Odometry",
              () -> {
                mRobotState.addOdometryMeasurement(
                    new OdometryMeasurement(
                        Timer.getFPGATimestamp(),
                        getSwerveModulePositions(),
                        Optional.of(mGyro.getRotation3d())));
              });
        });
  }

  // * GETTERS
  /**
   * Check if robot is accelerating abnormally
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Colliding (bool)", importance = Importance.INFO)
  public boolean isColliding() {
    return mGyro.getAccelerationMetersPerSecondPerSecond().norm()
        > mConstants.getMaxLinearAcceleration().in(MetersPerSecondPerSecond);
  }

  /**
   * Check if robot wheels are slipping
   *
   * @return {@link Boolean}
   */
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

  /**
   * Get Rotation in 3-Dimensional Space
   *
   * @return {@link Rotation3d}
   */
  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public Rotation3d getRotation3d() {
    return mRobotState.getRotation3d();
  }

  /**
   * Get Field Position in 3-Dimensional Space
   *
   * @return {@link Pose3d}
   */
  @Logged(name = "Pose3d", importance = Importance.INFO)
  public Pose3d getPose3d() {
    return mRobotState.getPose3d();
  }

  /**
   * Get magnitude of velocity
   *
   * @return {@link LinearVelocity}
   */
  @Logged(name = "Linear Velocity (meters per second)", importance = Importance.INFO)
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(getVelocityMetersPerSecond().norm());
  }

  /**
   * Get velocity with reference to field origin as a 3-Dimensional vector (vx, vy, omega)
   *
   * @return {@link Vector} of length {@link N3}
   */
  public Vector<N3> getVelocityWrtFieldMetersPerSecond() {
    return VecBuilder.fill(
        getChassisSpeedsWrtField().vxMetersPerSecond,
        getChassisSpeedsWrtField().vyMetersPerSecond,
        getChassisSpeedsWrtField().omegaRadiansPerSecond);
  }

  /**
   * Get setpoint velocity as a 3-Dimensional vector (vx, vy, omega)
   *
   * @return {@link Vector} of length (@link N3)
   */
  public Vector<N3> getSetpointVelocityMetersPerSecond() {
    return VecBuilder.fill(
        getSetpointChassisSpeeds().vxMetersPerSecond,
        getSetpointChassisSpeeds().vyMetersPerSecond,
        getSetpointChassisSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Get velocity as a 3-Dimensional vector (vx, vy, omega)
   *
   * @return {@link Vector} of length {@link N3}
   */
  public Vector<N3> getVelocityMetersPerSecond() {
    return VecBuilder.fill(
        getChassisSpeeds().vxMetersPerSecond,
        getChassisSpeeds().vyMetersPerSecond,
        getChassisSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Get velocity components with refence to field origin
   *
   * @return {@link ChassisSpeeds}
   */
  @Logged(name = "Chassis Speeds (wrt field)", importance = Importance.INFO)
  public ChassisSpeeds getChassisSpeedsWrtField() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getChassisSpeeds(), getRotation3d().toRotation2d());
  }

  /**
   * Get velocity components
   *
   * @return {@link ChassisSpeeds}
   */
  @Logged(name = "Setpoint Chassis Speeds", importance = Importance.INFO)
  public ChassisSpeeds getSetpointChassisSpeeds() {
    return mConstants.getKinematics().toChassisSpeeds(getSetpointSwerveModuleState());
  }

  /**
   * Get velocity components
   *
   * @return {@link ChassisSpeeds}
   */
  @Logged(name = "Chassis Speeds", importance = Importance.INFO)
  public ChassisSpeeds getChassisSpeeds() {
    return mConstants.getKinematics().toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * Get positions of all swerve modules
   *
   * @return {@link SwerveModulePosition} array
   */
  @Logged(name = "Swerve Module Positions", importance = Importance.INFO)
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] poses = new SwerveModulePosition[mModules.length];
    for (int i = 0; i < mModules.length; i++) {
      poses[i] = mModules[i].getSwerveModulePosition();
    }

    return poses;
  }

  /**
   * Get desired states of all swerve modules
   *
   * @return {@link SwerveModuleState} array
   */
  @Logged(name = "Setpoint Swerve Module State", importance = Importance.INFO)
  public SwerveModuleState[] getSetpointSwerveModuleState() {
    return mSetpointStates;
  }

  /**
   * Get states of all swerve modules
   *
   * @return {@link SwerveModuleState} array
   */
  @Logged(name = "Swerve Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[mModules.length];
    for (int i = 0; i < mModules.length; i++) {
      states[i] = mModules[i].getSwerveModuleState();
    }

    return states;
  }

  /**
   * Get a consumer for applying {@link SwerveSample}
   *
   * @return {@link Consumer} of {@link SwerveSample}
   */
  public Consumer<SwerveSample> getSwerveSampleConsumer() {
    return (sample) -> {
      // Get sample velocities & feedback velocities
      var speeds = sample.getChassisSpeeds();
      var feedbackSpeeds =
          new ChassisSpeeds(
              mPositionXController.calculate(getPose3d().getX(), sample.x),
              mPositionYController.calculate(getPose3d().getY(), sample.y),
              mRotationController.calculate(
                  getRotation3d().toRotation2d().getRadians(), sample.heading));

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

  // * SETTERS
  /**
   * Set a {@link ChassisSpeeds} setpoint
   *
   * @param speeds {@link ChassisSpeeds} Desired velocity components
   */
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

  // * COMMANDS
  public Command runPivotCharacterizationSequence() {
    return Commands.sequence(
            mPivotCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mPivotCharacterization.quasistatic(Direction.kReverse),
            Commands.waitSeconds(1.5),
            mPivotCharacterization.dynamic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mPivotCharacterization.dynamic(Direction.kReverse))
        .withName("Drive Pivot Characterization");
  }

  public Command runLinearDriveCharacterization() {
    return Commands.sequence(
            mDriveLinearCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveLinearCharacterization.dynamic(Direction.kReverse),
            Commands.waitSeconds(1.5),
            mDriveLinearCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveLinearCharacterization.dynamic(Direction.kReverse))
        .withName("Drive Linear Characterization");
  }

  public Command runAngularDriveCharacterization() {
    return Commands.sequence(
            mDriveAngularCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveAngularCharacterization.dynamic(Direction.kReverse),
            Commands.waitSeconds(1.5),
            mDriveAngularCharacterization.quasistatic(Direction.kForward),
            Commands.waitSeconds(1.5),
            mDriveAngularCharacterization.dynamic(Direction.kReverse))
        .withName("Drive Angular Characterization");
  }

  public Command driveFromTeleoperatedInputs(
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.none();
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
            ;
          }
        });
  }

  public Command lock() {
    return Commands.none();
  }
}
