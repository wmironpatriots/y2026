// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import org.frc6423.robot.Constants.Matrix;

/** Constant static members for the {@link Hood} Subsystem */
@Deprecated
public class HoodConstants {
  // * PHYSICAL CONSTANTS
  /** {@link Distance} 'Length' of Pivot System */
  public static final Distance kLength = Inches.of(7.510000);

  /** {@link MomentOfInertia} Rotational Inertia of Pivot System */
  public static final MomentOfInertia kRotationalInertia =
      KilogramSquareMeters.of(75.752248 * 0.0002926397);

  // * CONTROL CONSTANTS
  /** {@link Angle} Max allowed angular position error in subsystem */
  public static final Angle kEpsilon = Degrees.of(0.5);

  /** {@link Angle} Lower limit on angular position */
  public static final Angle kMinAngle = Degrees.of(14.703759);

  /** {@link Angle} upper limit on angular position */
  public static final Angle kMaxAngle = Degrees.of(45.812);

  /** {@link AngularVelocity} The maximum allowed angular velocity of subsystem */
  public static final AngularVelocity kMaxVelocity = RevolutionsPerSecond.of(1);

  /** {@link AngularAcceleration} The maximum allowed angular acceleration of subsystem */
  public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(15.0);

  // * HARDWARE CONSTANTS

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} CAN ID of servo */
  public static final int kServoCanDeviceId = Matrix.kHoodId;

  /** {@link Current} Stator current limit of servo */
  public static final Current kServoStatorCurrentLimit = Amps.of(40.0);

  /** {@link Double} Gear ratio between the servo rotor and the encoder shaft */
  public static final double kRotorToSensorRatio = 2.57142857143;

  /** {@link Double} Gear ratio between the encoder shaft and the mechanism pivot */
  public static final double kSensorToMechRatio = 10.83;

  /** {@link Integer} CAN ID of abs encoder */
  public static final int kEncoderCanDeviceId = Matrix.kHoodEncoderId;

  /** {@link Angle} Angular Offset to the stowed angle of abs encoder */
  public static final Angle kEncoderAngularOffset =
      Revolutions.of(-0.354736328125).plus(kMinAngle.times(kSensorToMechRatio)); // TODO

  /** {@link Angle} Angular Position in the middle of the 'unreachable' area of pivot */
  public static final Angle kEncoderSensorDiscontinuityPoint =
      Degrees.of(360).minus(kMaxAngle.minus(kMinAngle)).div(2).plus(kMaxAngle);

  /** {@link CANcoderConfiguration} Hardware config of abs encoder */
  public static final CANcoderConfiguration kEncoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(kEncoderAngularOffset)
                  .withAbsoluteSensorDiscontinuityPoint(kEncoderSensorDiscontinuityPoint));

  /** {@link TalonFXConfiguration} Hardware config of servo */
  public static final TalonFXConfiguration kServoTalonConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(kServoStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
                  .withRotorToSensorRatio(kRotorToSensorRatio)
                  .withSensorToMechanismRatio(kSensorToMechRatio))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(1)
                  .withMotionMagicAcceleration(2))
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKP(250.0)
                  .withKD(10.0)); // TODO Torque Current Control Gains (accelerating)

  // * CHARACTERIZATON
  /**
   * {@link Velocity} of {@link CurrentUnit} Rate at which current output ramps up at in Quasistatic
   * Characterization
   */
  public static final Velocity<CurrentUnit> kCharacterizationRampRate = Amps.of(15.0).per(Second);

  /** {@link Current} Current step size for Dynamic Characterization */
  public static final Current kCharacterizationStepSize = Amps.of(6.0);
}
