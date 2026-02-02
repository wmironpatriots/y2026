// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.frc6423.lib.sim.FlywheelSim;

/**
 * This is a globally accessible class for storing immutable values.
 *
 * <p>All values in this class are public, static, and final
 *
 * <p>To utilize values in this class, you should statically import the entire class or its
 * subclasses
 */
public final class Constants {
  /** The matrix contains the CAN identification information for all devices */
  public static final class Matrix {
    /** {@link CANBus} all drive CAN devices are on */
    public static final CANBus kDriveCanBus = new CANBus("DRIVE");

    public static final int kDriveBrPivotId = 1;
    public static final int kDriveBrEncoderId = 2;
    public static final int kDriveBrDriveId = 3;
    public static final int kDriveFrPivotId = 4;
    public static final int kDriveFrEncoderId = 5;
    public static final int kDriveFrDriveId = 6;
    public static final int kDriveFlPivotId = 7;
    public static final int kDriveFlEncoderId = 8;
    public static final int kDriveFlDriveId = 9;
    public static final int kDriveBlPivotId = 10;
    public static final int kDriveBlEncoderId = 11;
    public static final int kDriveBlDriveId = 12;

    /** {@link CANBus} all subsystem CAN devices are on */
    public static final CANBus kSubsystemCanBus = new CANBus("SOUP");

    public static final int kIntakePivotId = 1;
    public static final int kIntakeEncoderId = 2;
    public static final int kIntakeRollerId = 3;
    public static final int kIndexerId = 4;
    public static final int kFeederId = 5;
    public static final int kHoodId = 6;
    public static final int kFlywheelLeftId = 7;
    public static final int kFlywheelRightId = 8;
  }

  /** Constants for initializing the {@link Intake} subsystem */
  public static final class IntakeConstants {
    // * PIVOT CONSTANTS

    /** {@link String} representing the name of the {@link IntakePivot} servo */
    public static final String kPivotName = "Intake Pivot";

    /** Represents the CAN device ID of the {@link IntakePivot} servo */
    public static final int kEncoderId = Matrix.kIntakeEncoderId;

    /** Represents the CAN device ID of the {@link IntakePivot} encoder */
    public static final int kPivotId = Matrix.kIntakePivotId;

    /** {@link Current} representing the stator current limit of the {@link IntakePivot} servo */
    public static final Current kPivotStatorLimit = Amps.of(20);

    /** Represents the gear ratio between the {@link IntakePivot} servo and encoder */
    public static final double kPivotRotorToSensor = (3.0 / 1.0) * (3.0 / 1.0) * (20.0 / 42.0);

    /** Represents the gear ratio between the {@link IntakePivot} encoder and mechanism */
    public static final double kPivotSensorToMech = (16.0 / 36.0);

    /** {@link Distance} representing the length of the {@link IntakePivot} arm */
    public static final Distance kPivotArmLength = Inches.of(1.0);

    /**
     * {@link MomentOfInertia} representing the Rotational Inertia of the {@link IntakePivot} arm
     */
    public static final MomentOfInertia kPivotArmMoi = KilogramSquareMeters.of(0.07799);

    /** {@link Angle} representing the reverse angular position limit of the {@link IntakePivot} */
    public static final Angle kPivotAngleMin = Degrees.of(89.87);

    /** {@link Angle} representing the forward angular position limit of the {@link IntakePivot} */
    public static final Angle kPivotAngleMax = Degrees.of(148.2);

    /** {@link AngularVelocity} representing the velocity limit of the {@link IntakePivot} */
    public static final AngularVelocity kPivotCruiseVel = RevolutionsPerSecond.of(2.0);

    /**
     * {@link AngularAcceleration} representing the maximum acceleration of the {@link IntakePivot}
     */
    public static final AngularAcceleration kPivotMaxAccel = RadiansPerSecondPerSecond.of(5555);

    /** {@link Slot0Configs} representing the control gains for {@link IntakePivot} */
    public static final Slot0Configs kPivotGains =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withKG(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(0.0)
            .withKD(0.0); // TODO

    /** {@link TalonFXConfiguration} representing the config for the {@link IntakePivot} servo */
    public static final TalonFXConfiguration kPivotConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(kPivotStatorLimit)
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(kEncoderId)
                    .withRotorToSensorRatio(kPivotRotorToSensor)
                    .withSensorToMechanismRatio(kPivotSensorToMech))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kPivotCruiseVel)
                    .withMotionMagicAcceleration(kPivotMaxAccel))
            .withSlot0(kPivotGains);

    /** {@link Angle} representing the angular position offset of the {@link IntakePivot} encoder */
    public static final Angle kEncoderOffset = Degrees.of(0.0); // TODO

    /** {@link CANcoderConfiguration} representing the config for the {@link IntakePivot} encoder */
    public static final CANcoderConfiguration kEncoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(
                        (kPivotConfig.MotorOutput.Inverted
                                == InvertedValue.CounterClockwise_Positive)
                            ? SensorDirectionValue.Clockwise_Positive
                            : SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(kEncoderOffset));

    // * ROLLER CONSTANTS

    /** {@link String} representing the name of the {@link IntakeRoller} servo */
    public static final String kRollerName = "Intake Roller";

    /** Represents the CAN device ID {@link IntakeRoller} servo */
    public static final int kRollerId = Matrix.kIntakeRollerId;

    /** {@link Current} representing the stator current limit of the {@link IntakeRoller} servo */
    public static final Current kRollerSupplyLimit = Amps.of(80);

    /** {@link Current} representing the stator current limit of the {@link IntakeRoller} servo */
    public static final Current kRollerStatorLimit = Amps.of(20);

    /** Represents the gear ratio between the {@link IntakeRoller} servo encoder and mechanism */
    public static final double kRollerSensorToMech = (1.0 / 1.0);

    /** {@link MomentOfInertia} representing the Rotational Inertia of the {@link IntakeRoller} */
    public static final MomentOfInertia kRollerMoi = FlywheelSim.kGenericRollerMoi;

    /** {@link Slot0Configs} representing the control gains for {@link IntakeRoller} */
    public static final Slot0Configs kRollerGains =
        new Slot0Configs()
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withKV(0.0)
            .withKS(0.0)
            .withKA(0.0)
            .withKP(0.0)
            .withKD(0.0); // TODO

    /** {@link TalonFXConfiguration} representing the config for the {@link IntakeRoller} servo */
    public static final TalonFXConfiguration kRollerConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(kRollerSupplyLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(kRollerStatorLimit)
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kRollerSensorToMech))
            .withSlot0(kRollerGains);
  }
}
