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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
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

  public static final double kPivotReduction = 26 / 1;

  public static final double kDriveReduction = 6.03 / 1;

  public static final SwerveModuleConfig[] kModuleConfigs =
      new SwerveModuleConfig[] {
        new SwerveModuleConfig(
            "BL Module",
            new ServoIOTalonFx(
                "BL Pivot", kBlPivot, kDriveCanbus, getPivotTalonFXConfiguration(kBlAbsEncoder)),
            new ServoIOTalonFx("BL Drive", kBlDrive, kDriveCanbus, getDriveTalonFXConfiguration()),
            new EncoderIOCancoder(kBlAbsEncoder, kDriveCanbus, getEncoderCANcoderConfiguration()),
            kWheelRadius,
            new Translation2d(kCenterToEdge, kCenterToEdge).times(-1)),
        new SwerveModuleConfig(
            "FL Module",
            new ServoIOTalonFx(
                "FL Pivot", kFlPivot, kDriveCanbus, getPivotTalonFXConfiguration(kFlAbsEncoder)),
            new ServoIOTalonFx("FL Drive", kFlDrive, kDriveCanbus, getDriveTalonFXConfiguration()),
            new EncoderIOCancoder(kFlAbsEncoder, kDriveCanbus, getEncoderCANcoderConfiguration()),
            kWheelRadius,
            new Translation2d(kCenterToEdge.times(-1), kCenterToEdge)),
        new SwerveModuleConfig(
            "FR Module",
            new ServoIOTalonFx(
                "FR Pivot", kFrPivot, kDriveCanbus, getPivotTalonFXConfiguration(kFrAbsEncoder)),
            new ServoIOTalonFx("FR Drive", kFrDrive, kDriveCanbus, getDriveTalonFXConfiguration()),
            new EncoderIOCancoder(kFrAbsEncoder, kDriveCanbus, getEncoderCANcoderConfiguration()),
            kWheelRadius,
            new Translation2d(kCenterToEdge, kCenterToEdge)),
        new SwerveModuleConfig(
            "BR Module",
            new ServoIOTalonFx(
                "BR Pivot", kBrPivot, kDriveCanbus, getPivotTalonFXConfiguration(kBrAbsEncoder)),
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

  public static final double kMaxFocVelocityNorm = 0.9;

  static {
    for (int i = 0; i < kModuleConfigs.length; i++) {
      kModuleDisplacementsWrtCenter[i] = kModuleConfigs[i].displacementWrtCenter();
    }
  }

  public static TalonFXConfiguration getPivotTalonFXConfiguration(int cancoderId) {
    var config = new TalonFXConfiguration();

    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO

    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = cancoderId;
    config.Feedback.RotorToSensorRatio = kPivotReduction;

    config.ClosedLoopGeneral.ContinuousWrap = true;

    // Torque Current configs
    config.Slot0.kS = 600.0;
    config.Slot0.kV = 50.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 10.0;
    config.Slot0.kD = 0.014;

    // Voltage Configs
    config.Slot1.kS = 0.0;
    config.Slot1.kV = 0.0;
    config.Slot1.kA = 0.0;
    config.Slot1.kP = 0.0;
    config.Slot1.kD = 0.0;

    return config;
  }

  public static TalonFXConfiguration getDriveTalonFXConfiguration() {
    var config = new TalonFXConfiguration();

    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    // Torque Current configs
    config.Slot0.kS = 35.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 5.0;

    // Voltage Configs
    config.Slot1.kS = 0.0;
    config.Slot1.kV = 0.0;
    config.Slot1.kA = 0.0;
    config.Slot1.kP = 0.0;
    config.Slot1.kD = 0.0;

    return config;
  }

  public static CANcoderConfiguration getEncoderCANcoderConfiguration(Rotation2d offset) {
    var config = new CANcoderConfiguration();

    config.MagnetSensor.MagnetOffset = offset.getRotations();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // TODO

    return config;
  }

  public static Pigeon2Configuration getPigeon2Configuration() {
    var config = new Pigeon2Configuration();

    return config;
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
            VecBuilder.fill(0.6, 0.6, 0.0, 0.07),
            VecBuilder.fill(0.9, 0.9, 0.0, 0.4)); // TODO fill vecs
  }

  @Override
  public void periodic() {
    poseEstimator.update(gyro.getRotation3d(), getSwerveModulePositions());

    for (var module : modules) {
      module.periodic();
    }

    gyro.periodic();

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /**
   * Drive based on speeds
   *
   * @param xMagSupplier {@link DoubleSupplier} representing x velocity input stream
   * @param yMagSupplier {@link DoubleSupplier} representing y velocity input stream
   * @param omegaMagSupplier {@link DoubleSupplier} representing omega velocity input stream
   * @return {@link Command}
   */
  public Command driveFromSpeedsCmd(
      DoubleSupplier xMagSupplier, DoubleSupplier yMagSupplier, DoubleSupplier omegaMagSupplier) {
    return this.run(
        () ->
            ChassisSpeeds.fromFieldRelativeSpeeds(
                kMaxLinearVelocity, kMaxLinearVelocity, kMaxAngularVelocity, getRotation2d()));
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
        (currentVelocity >= kMaxLinearVelocity.times(kMaxFocVelocityNorm).in(MetersPerSecond))
            ? false
            : true;

    for (int i = 0; i < modules.length; i++) {
      modules[i].runSwerveModuleState(states[i], focEnabled);
    }
  }

  /** Stop all movement */
  public void stop() {
    for (var module : modules) {
      module.stop();
    }
  }

  public Rotation2d getRotation2d() {
    return getRotation3d().toRotation2d();
  }

  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public Rotation3d getRotation3d() {
    return getPose3d().getRotation();
  }

  @Logged(name = "Pose3d", importance = Importance.INFO)
  public Pose3d getPose3d() {
    return poseEstimator.getEstimatedPosition();
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
