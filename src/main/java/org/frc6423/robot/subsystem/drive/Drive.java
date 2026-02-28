// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.Consumer;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.util.Tracer;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.RobotState.OdometryMeasurement;
import org.frc6423.robot.subsystem.drive.component.GyroIO;
import org.frc6423.robot.subsystem.drive.component.GyroIOPigeon2;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystem.drive.component.SwerveModuleIOServo;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;

/** TODO WIP */
public class Drive extends SubsystemBase {
  public static Drive create() {
    return new Drive(
        new GyroIOPigeon2(0, CANBus.roboRIO(), new Pigeon2Configuration()),
        new SwerveModuleIOServo(
            "placeholder1",
            new EncoderIOCanCoder(0, CANBus.roboRIO(), new CANcoderConfiguration()),
            new ServoIONone("placeholder1p"),
            new ServoIONone("placeholder1d"),
            Flags.kRobotType.mDriveConstants.getBackLeftModuleConfig(),
            Flags.kRobotType.mDriveConstants),
        new SwerveModuleIOServo(
            "placeholder2",
            new EncoderIOCanCoder(0, CANBus.roboRIO(), new CANcoderConfiguration()),
            new ServoIONone("placeholder2p"),
            new ServoIONone("placeholder2d"),
            Flags.kRobotType.mDriveConstants.getBackLeftModuleConfig(),
            Flags.kRobotType.mDriveConstants),
        new SwerveModuleIOServo(
            "placeholder3",
            new EncoderIOCanCoder(0, CANBus.roboRIO(), new CANcoderConfiguration()),
            new ServoIONone("placeholder3p"),
            new ServoIONone("placeholder3d"),
            Flags.kRobotType.mDriveConstants.getBackLeftModuleConfig(),
            Flags.kRobotType.mDriveConstants),
        new SwerveModuleIOServo(
            "placeholder4",
            new EncoderIOCanCoder(0, CANBus.roboRIO(), new CANcoderConfiguration()),
            new ServoIONone("placeholder4p"),
            new ServoIONone("placeholder4d"),
            Flags.kRobotType.mDriveConstants.getBackLeftModuleConfig(),
            Flags.kRobotType.mDriveConstants));
  }

  private final DriveConstants mConstants = Flags.kRobotType.mDriveConstants;
  private final RobotState mRobotState = RobotState.getInstance();

  // * HARDWARE MEMBERS
  @Logged private final SwerveModuleIO mFrModule;
  @Logged private final SwerveModuleIO mFlModule;
  @Logged private final SwerveModuleIO mBlModule;
  @Logged private final SwerveModuleIO mBrModule;

  private final SwerveModuleIO[] mModules;
  private final GyroIO mGyro;

  // * SYSID MEMBERS
  private final SysIdRoutine mPivotCharacterization,
      mDriveLinearCharacterization,
      mDriveAngularCharacterization;

  /**
   * Create new {@link Drive}
   *
   * @param gyro {@link GyroIO} Gyro for odometry
   * @param frontRightModule {@link SwerveModuleIO} Front Right Swerve Module
   * @param frontLeftModule {@link SwerveModuleIO} Front Left Swerve Module
   * @param backLeftModule {@link SwerveModuleIO} Back Left Swerve Module
   * @param backRightModule {@link SwerveModuleIO} Back Right Swerve Module
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

    // Init SysId
    mPivotCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
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
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {}, // TODO
                null,
                this,
                "SwerveLinearSysId"));

    mDriveAngularCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
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
        "Swerve Periodic",
        () -> {
          // Update Hardware
          for (var module : mModules) {
            module.periodic();
          }

          // Send odometry measurements to robot state
          mRobotState.addOdometryMeasurement(
              new OdometryMeasurement(
                  Timer.getFPGATimestamp(),
                  getSwerveModulePositions(),
                  Optional.of(mGyro.getRotation3d())));
        });
  }

  // * GETTERS
  public Rotation3d getRotation3d() {
    return mRobotState.getRotation3d();
  }

  public Pose3d getPose3d() {
    return mRobotState.getPose3d();
  }

  public ChassisSpeeds getChassisSpeedsWrtField() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getChassisSpeeds(), getRotation3d().toRotation2d());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return mConstants.getKinematics().toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] poses = new SwerveModulePosition[mModules.length];
    for (int i = 0; i < mModules.length; i++) {
      poses[i] = mModules[i].getSwerveModulePosition();
    }

    return poses;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[mModules.length];
    for (int i = 0; i < mModules.length; i++) {
      states[i] = mModules[i].getSwerveModuleState();
    }

    return states;
  }

  public Consumer<SwerveSample> getSwerveSampleConsumer() {
    return (sample) -> {};
  }

  // * SETTERS

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
        .withName("Drive Linear Characterization");
  }
}
