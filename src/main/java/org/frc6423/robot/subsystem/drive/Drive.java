// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.EncoderIOCancoder;
import org.frc6423.lib.io.GyroIO;
import org.frc6423.lib.io.GyroIOPigeon;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.subsystem.drive.SwerveModule.SwerveModuleConfig;

public class Drive extends SubsystemBase {
  public static final CANBus kDriveCanbus = new CANBus("DRIVEBUS");

  public static final int kBlPivot = 1;
  public static final int kBlDrive = 2;
  public static final int kBlAbsEncoder = 3;

  public static final int kFlPivot = 4;
  public static final int kFlDrive = 5;
  public static final int kFlAbsEncoder = 6;

  public static final int kFrPivot = 7;
  public static final int kFrDrive = 8;
  public static final int kFrAbsEncoder = 9;

  public static final int kBrPivot = 10;
  public static final int kBrDrive = 11;
  public static final int kBrAbsEncoder = 12;

  public static final int kPigeon = 13;

  public static final Distance kCenterToEdge = Inches.of(10.875);

  public static final Distance kRotationRadius;

  static {
    var cte = kCenterToEdge.in(Inches);

    kRotationRadius = Inches.of(Math.sqrt(Math.pow(cte, 2) * 2));
  }

  public static final Distance kWheelRadius = Inches.of(2);

  public static final SwerveModuleConfig[] kModuleConfigs =
      new SwerveModuleConfig[] {
        new SwerveModuleConfig(
            "BL Module",
            new ServoIOTalonFx("BL Pivot", kBlPivot, kDriveCanbus, getPivotTalonFXConfiguration()),
            new ServoIOTalonFx("BL Drive", kBlDrive, kDriveCanbus, getDriveTalonFXConfiguration()),
            new EncoderIOCancoder(kBlAbsEncoder, kDriveCanbus, getEncoderCANcoderConfiguration()),
            kWheelRadius,
            new Translation2d(kCenterToEdge, kCenterToEdge).times(-1)),
        new SwerveModuleConfig(
            "FL Module",
            new ServoIOTalonFx("FL Pivot", kFlPivot, kDriveCanbus, getPivotTalonFXConfiguration()),
            new ServoIOTalonFx("FL Drive", kFlDrive, kDriveCanbus, getDriveTalonFXConfiguration()),
            new EncoderIOCancoder(kFlAbsEncoder, kDriveCanbus, getEncoderCANcoderConfiguration()),
            kWheelRadius,
            new Translation2d(kCenterToEdge.times(-1), kCenterToEdge)),
        new SwerveModuleConfig(
            "FR Module",
            new ServoIOTalonFx("FR Pivot", kFrPivot, kDriveCanbus, getPivotTalonFXConfiguration()),
            new ServoIOTalonFx("FR Drive", kFrDrive, kDriveCanbus, getDriveTalonFXConfiguration()),
            new EncoderIOCancoder(kFrAbsEncoder, kDriveCanbus, getEncoderCANcoderConfiguration()),
            kWheelRadius,
            new Translation2d(kCenterToEdge, kCenterToEdge)),
        new SwerveModuleConfig(
            "BR Module",
            new ServoIOTalonFx("BR Pivot", kBrPivot, kDriveCanbus, getPivotTalonFXConfiguration()),
            new ServoIOTalonFx("BR Drive", kBrDrive, kDriveCanbus, getDriveTalonFXConfiguration()),
            new EncoderIOCancoder(kBrAbsEncoder, kDriveCanbus, getEncoderCANcoderConfiguration()),
            kWheelRadius,
            new Translation2d(kCenterToEdge, kCenterToEdge.times(-1))),
      };

  public static final Translation2d[] kModuleDisplacementsWrtCenter =
      new Translation2d[kModuleConfigs.length];

  public static final LinearVelocity kMaxLinearVelocity = FeetPerSecond.of(17.4);

  public static final AngularVelocity kMaxAngularVelocity =
      RadiansPerSecond.of(kMaxLinearVelocity.in(InchesPerSecond) / kRotationRadius.in(Inches));

  static {
    for (int i = 0; i < kModuleConfigs.length; i++) {
      kModuleDisplacementsWrtCenter[i] = kModuleConfigs[i].displacementWrtCenter();
    }
  }

  public static TalonFXConfiguration getPivotTalonFXConfiguration() {
    return new TalonFXConfiguration();
  }

  public static TalonFXConfiguration getDriveTalonFXConfiguration() {
    return new TalonFXConfiguration();
  }

  public static CANcoderConfiguration getEncoderCANcoderConfiguration() {
    return new CANcoderConfiguration();
  }

  public static Pigeon2Configuration getPigeon2Configuration() {
    return new Pigeon2Configuration();
  }

  private final SwerveModule[] modules;
  private final GyroIO gyro;

  private final SwerveDriveKinematics kinematics;

  private final SwerveDrivePoseEstimator3d poseEstimator;

  public Drive() {
    modules =
        new SwerveModule[] {
          new SwerveModule(kModuleConfigs[0]),
          new SwerveModule(kModuleConfigs[1]),
          new SwerveModule(kModuleConfigs[2]),
          new SwerveModule(kModuleConfigs[3])
        };

    gyro = new GyroIOPigeon(kPigeon, kDriveCanbus, getPigeon2Configuration());

    kinematics = new SwerveDriveKinematics(kModuleDisplacementsWrtCenter);

    poseEstimator =
        new SwerveDrivePoseEstimator3d(
            kinematics,
            gyro.getRotation3d(),
            null,
            new Pose3d(),
            VecBuilder.fill(0.0, 0.0, 0.0, 0.0),
            VecBuilder.fill(0.0, 0.0, 0.0, 0.0)); // TODO fill vecs
  }

  @Override
  public void periodic() {
    poseEstimator.update(gyro.getRotation3d(), getSwerveModulePositions());

    for (var module : modules) {
      module.periodic();
    }

    gyro.periodic();
  }

  /** Stop all movement */
  public void stop() {
    for (var module : modules) {
      module.stop();
    }
  }

  /**
   * Run a {@link ChassisSpeeds} setpoint
   *
   * @param speeds {@link ChassisSpeeds} representing setpoint speeds
   */
  public void runChassisSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    var states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        speeds,
        kMaxLinearVelocity.in(MetersPerSecond),
        kMaxLinearVelocity.in(MetersPerSecond),
        kMaxAngularVelocity.in(RadiansPerSecond));

    var currentSpeeds = getChassisSpeeds();
    var currentVelocity =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    boolean focEnabled =
        (currentVelocity >= kMaxLinearVelocity.times(0.9).in(MetersPerSecond)) ? false : true;

    for (int i = 0; i < modules.length; i++) {
      modules[i].runSwerveModuleState(states[i], focEnabled);
    }
  }

  /**
   * @return {@link ChassisSpeeds} representing the setpoint drive chassis velocity
   */
  public ChassisSpeeds getSetpointChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSetpointSwerveModuleStates());
  }

  /**
   * @return {@link ChassisSpeeds} representing the drive chassis velocity
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * @return {@link SwerveModuleState} array representing the positional states of swerve modules
   */
  @Logged(name = "Swerve Module Positions", importance = Importance.INFO)
  public SwerveModulePosition[] getSwerveModulePositions() {
    var poses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      poses[i] = modules[i].getSwerveModulePosition();
    }

    return poses;
  }

  /**
   * @return {@link SwerveModuleState} array representing the setpoint velocity states of swerve
   *     modules
   */
  @Logged(name = "Setpoint Swerve Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSetpointSwerveModuleStates() {
    var states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getSetpointSwerveModuleState();
    }

    return states;
  }

  /**
   * @return {@link SwerveModuleState} array representing the velocity states of swerve modules
   */
  @Logged(name = "Swerve Module States", importance = Importance.INFO)
  public SwerveModuleState[] getSwerveModuleStates() {
    var states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getSwerveModuleState();
    }

    return states;
  }
}
