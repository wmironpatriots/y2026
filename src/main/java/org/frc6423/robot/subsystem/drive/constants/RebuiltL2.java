// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SwerveConstants} extension for artifactor's configs (L2 ratio)
 *
 * <p>https://cad.onshape.com/documents?nodeId=e1e360f455a945b4649bb992&resourceType=folder
 *
 * <p>MK5i /w L1 drive ratio (https://www.swervedrivespecialties.com/products/mk5i-swerve-module)
 *
 * <p>Kraken x60 for pivot & drive, CANcoder for encoder
 */
public class RebuiltL2 extends SwerveConstants {
  @Override
  public double getTrackWidthMeters() {
    return Units.inchesToMeters(23.000);
  }

  @Override
  public double getBumperThicknessInches() {
    return Units.inchesToMeters(5.25);
  }

  @Override
  public double getMaxLinearVelocityMetersPerSecond() {
    // https://www.swervedrivespecialties.com/products/mk5i-swerve-module
    return Units.feetToMeters(17.4);
  }

  @Override
  public double getMaxLinearAccelerationMetersPerSecondPerSecond() {
    return 17.594;
  }

  @Override
  public double getFocAutoToggleMagnitude() {
    return 0.9;
  }

  @Override
  public double getAngularAccelerationRadsPerSecPerSec() {
    return 84.759;
  }

  @Override
  public double getMassKg() {
    // TODO measure value irl; Values from CAD
    return Units.lbsToKilograms(113);
  }

  @Override
  public double getRotationalInertiaKgSquaredMeters() {
    // TODO measure value irl; Values from CAD
    return 14075.351 * 0.0002926397;
  }

  @Override
  public double getWheelRadiusMeters() {
    return Units.inchesToMeters(2.0);
  }

  @Override
  public double getPivotRotorToSensorRatio() {
    return (26.0 / 1.0);
  }

  @Override
  public double getPivotSensorToMechanismRatio() {
    return (1.0 / 1.0);
  }

  @Override
  public boolean getPivotInverted() {
    // https://www.swervedrivespecialties.com/products/mk5i-swerve-module
    return false;
  }

  @Override
  public double getDriveRotorToMechRatio() {
    return (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0);
  }

  @Override
  public CANBus getCANBus() {
    return Matrix.kDriveCanBus;
  }

  @Override
  public Pigeon2Configuration getPigeon2Config() {
    return new Pigeon2Configuration();
  }

  @Override
  public TalonFXConfiguration getPivotServoConfig(int CANcoderId) {
    return new TalonFXConfiguration()
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(
                    getPivotInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40.0)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(120.0)
                .withStatorCurrentLimitEnable(true))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withFeedbackRemoteSensorID(CANcoderId)
                .withSensorToMechanismRatio(getPivotSensorToMechanismRatio())
                .withRotorToSensorRatio(getPivotRotorToSensorRatio()))
        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity((7368 / 60) / getPivotRotorToSensorRatio())
                .withMotionMagicAcceleration((7368 / 60) / getPivotRotorToSensorRatio() * 0.005));
  }

  @Override
  public TalonFXConfiguration getDriveServoConfig() {
    return new TalonFXConfiguration()
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(120.0)
                .withStatorCurrentLimitEnable(true))
        .withTorqueCurrent(new TorqueCurrentConfigs().withTorqueNeutralDeadband(10.0))
        // .withPeakForwardTorqueCurrent(120.0)
        // .withPeakReverseTorqueCurrent(120.0))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(getDriveRotorToMechRatio()))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                    getMaxLinearVelocityMetersPerSecond() / getWheelRadiusMeters())
                .withMotionMagicAcceleration(
                    getMaxLinearAccelerationMetersPerSecondPerSecond() / getWheelRadiusMeters()));
  }

  @Override
  public double getDriveMotorKt() {
    return DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
  }

  @Override
  public CANcoderConfiguration getCANcoderConfig(double angularOffsetRevs) {
    return new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withSensorDirection(
                    getPivotInverted()
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(angularOffsetRevs));
  }

  @Override
  public GyroConfig getGyroConfig() {
    return new GyroConfig(getCANBus(), Matrix.kDriveGyroId, getPigeon2Config());
  }

  // TODO offsets

  @Override
  public ModuleConfig getFrontLeftModuleConfig() {
    return new ModuleConfig(
        "FL",
        getCANBus(),
        Matrix.kDriveFlPivotId,
        Matrix.kDriveFlDriveId,
        Matrix.kDriveFlEncoderId,
        getPivotServoConfig(Matrix.kDriveFlEncoderId),
        getDriveServoConfig(),
        getCANcoderConfig(0.27783203125),
        getWheelRadiusMeters());
  }

  @Override
  public ModuleConfig getFrontRightModuleConfig() {
    return new ModuleConfig(
        "FR",
        getCANBus(),
        Matrix.kDriveFrPivotId,
        Matrix.kDriveFrDriveId,
        Matrix.kDriveFrEncoderId,
        getPivotServoConfig(Matrix.kDriveFrEncoderId),
        getDriveServoConfig(),
        getCANcoderConfig(0.36279296875),
        getWheelRadiusMeters());
  }

  @Override
  public ModuleConfig getBackLeftModuleConfig() {
    return new ModuleConfig(
        "BL",
        getCANBus(),
        Matrix.kDriveBlPivotId,
        Matrix.kDriveBlDriveId,
        Matrix.kDriveBlEncoderId,
        getPivotServoConfig(Matrix.kDriveBlEncoderId),
        getDriveServoConfig(),
        getCANcoderConfig(0.371060546875),
        getWheelRadiusMeters());
  }

  @Override
  public ModuleConfig getBackRightModuleConfig() {
    return new ModuleConfig(
        "BR",
        getCANBus(),
        Matrix.kDriveBrPivotId,
        Matrix.kDriveBrDriveId,
        Matrix.kDriveBrEncoderId,
        getPivotServoConfig(Matrix.kDriveBrEncoderId),
        getDriveServoConfig(),
        getCANcoderConfig(0.397705078125),
        getWheelRadiusMeters());
  }
}
