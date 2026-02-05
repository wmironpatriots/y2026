// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link DriveConstants} extension that represents the configs for FRC 6423's 2026 Robot:
 * <ROBOT-NAME>
 *
 * <p>https://cad.onshape.com/documents?nodeId=e1e360f455a945b4649bb992&resourceType=folder
 *
 * <p>MK5i /w L1 drive ratio (https://www.swervedrivespecialties.com/products/mk5i-swerve-module)
 *
 * <p>Kraken x60 for pivot & drive, CANcoder for encoder
 */
public class RebuiltL1 extends DriveConstants {
  @Override
  public Distance getTrackWidth() {
    return Inches.of(23.000);
  }

  @Override
  public Distance getBumperThickness() {
    return Inches.of(5.25);
  }

  @Override
  public LinearVelocity getMaxLinearVelocity() {
    // https://www.swervedrivespecialties.com/products/mk5i-swerve-module
    return FeetPerSecond.of(14.9);
  }

  @Override
  public LinearAcceleration getMaxLinearAcceleration() {
    // TODO derive in choreo; values copied from L2 ratio
    return MetersPerSecondPerSecond.of(12.624);
  }

  @Override
  public AngularAcceleration getAngularAcceleration() {
    // TODO derive in choreo; values copied from L2 ratio
    return RadiansPerSecondPerSecond.of(50.022);
  }

  @Override
  public Mass getMass() {
    // TODO measure value irl; Values from CAD
    return Pounds.of(105.9);
  }

  @Override
  public MomentOfInertia getRotationalInertia() {
    // TODO measure value irl; Values from CAD
    return KilogramSquareMeters.of(17452.55455 * 0.0002926397);
  }

  @Override
  public double getPivotRotorToSensorRatio() {
    return (26.0 / 1.0);
  }

  @Override
  public double getPivotSensorToMechanismRatio() {
    return 1.0;
  }

  @Override
  public boolean getPivotInverted() {
    // https://www.swervedrivespecialties.com/products/mk5i-swerve-module
    return false;
  }

  @Override
  public double getDriveRotorToMechRatio() {
    return (54.0 / 12.0) * (25.0 / 32.0) * (30.0 / 15.0);
  }

  @Override
  public Distance getWheelRadius() {
    return Inches.of(2.0);
  }

  @Override
  public CANBus getCANBus() {
    return Matrix.kDriveCanBus;
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
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120.0))
                .withStatorCurrentLimitEnable(true))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withFeedbackRemoteSensorID(CANcoderId)
                .withRotorToSensorRatio(getPivotRotorToSensorRatio()))
        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity((5800 / 60) / getPivotRotorToSensorRatio())
                .withMotionMagicAcceleration((5800 / 60) / getPivotRotorToSensorRatio() * 0.005))
        // TODO tune; current values stolen from cascade
        .withSlot0(
            new Slot0Configs()
                .withKS(0.014)
                .withKV(10.0)
                .withKA(0.0)
                .withKP(600.0)
                .withKD(50.0)); // Torque Based Motion Magic Position Controls
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
                .withStatorCurrentLimit(Amps.of(20.0))
                .withStatorCurrentLimitEnable(true))
        .withTorqueCurrent(new TorqueCurrentConfigs().withTorqueNeutralDeadband(Amps.of(10.0)))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(getDriveRotorToMechRatio()))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                    getMaxLinearVelocity().div(getWheelRadius().in(Feet)).in(FeetPerSecond))
                .withMotionMagicAcceleration(
                    getMaxLinearAcceleration()
                        .div(getWheelRadius().in(Feet))
                        .in(FeetPerSecondPerSecond)))
        // TODO tune; current values stolen from cascade
        .withSlot0(
            new Slot0Configs()
                .withKS(5.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(35.0)
                .withKD(0.0)) // Torque Based Motion Magic Velocity Controls
        // TODO tune; no values
        .withSlot1(
            new Slot1Configs()
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(0.0)
                .withKD(0.0)); // Voltage Based Motion Magic Velocity Controls
  }

  @Override
  public CANcoderConfiguration getCANcoderConfig(Angle offset) {
    return new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withSensorDirection(
                    getPivotInverted()
                        ? SensorDirectionValue.CounterClockwise_Positive
                        : SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(offset));
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    // TODO derive offsets
    return new ModuleConfig[] {
      new ModuleConfig(
          "FR",
          getCANBus(),
          Matrix.kDriveFrPivotId,
          Matrix.kDriveFrDriveId,
          Matrix.kDriveFrEncoderId,
          getPivotServoConfig(Matrix.kDriveFrEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(0.0))),
      new ModuleConfig(
          "FL",
          getCANBus(),
          Matrix.kDriveFlPivotId,
          Matrix.kDriveFlDriveId,
          Matrix.kDriveFlEncoderId,
          getPivotServoConfig(Matrix.kDriveFlEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(0.0))),
      new ModuleConfig(
          "BL",
          getCANBus(),
          Matrix.kDriveBlPivotId,
          Matrix.kDriveBlDriveId,
          Matrix.kDriveBlEncoderId,
          getPivotServoConfig(Matrix.kDriveBlEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(0.0))),
      new ModuleConfig(
          "BR",
          getCANBus(),
          Matrix.kDriveBrPivotId,
          Matrix.kDriveBrDriveId,
          Matrix.kDriveBrEncoderId,
          getPivotServoConfig(Matrix.kDriveBrEncoderId),
          getDriveServoConfig(),
          getCANcoderConfig(Degrees.of(0.0)))
    };
  }
}
