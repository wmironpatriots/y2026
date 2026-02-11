// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import org.frc6423.lib.io.ServoConfig;
import org.frc6423.robot.subsystem.drive.constants.Cascade;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.RebuiltL1;
import org.frc6423.robot.subsystem.drive.constants.RebuiltL2;
import org.frc6423.robot.subsystem.flywheel.Flywheel;
import org.frc6423.robot.subsystem.hood.Hood;
import org.frc6423.robot.subsystem.indexer.Indexer;
import org.frc6423.robot.subsystem.intake.Intake;

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
    public static final int kDriveGyroId = 13;

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

  /** Represents the type of robot codebase is running on */
  public static enum RobotType {
    /** {@link RobotType} representing the 2026 competition robot chassis /w L1 Ratio */
    Y2026_L1(new RebuiltL1()),
    /** {@link RobotType} representing the 2026 competition robot chassis /w L2 Ratio */
    Y2026_L2(new RebuiltL2()),
    /** {@link RobotType} representing the 2026 competition robot chassis /w L3 Ratio */
    Y2026_L3(new RebuiltL2()),
    /** {@link Robot} representing the 2025 competition robot chassis, cascade */
    Y2025(new Cascade());

    public final DriveConstants mDriveConstants;

    private RobotType(DriveConstants drivetrainConstants) {
      this.mDriveConstants = drivetrainConstants;
    }
  }

  /** Static constants class for the {@link Intake} subsystem */
  public static class IntakeConstants {}

  /** Static constants class for the {@link Indexer} subsystem */
  public static class IndexerConstants {}

  /** Static constants class for the {@link Feeder} subsystem */
  public static class FeederConstants {
    /** {@link Double} representing the gear ratio between the sensor to mechanism shaft */
    public static double kSensorToMechRatio = 1.0 / 1.0;

    /** {@link Angle} representing acceptable amount of error */
    public static Angle kEpsilon = Degrees.of(0.4);

    /**
     * {@link TalonFXConfiguration} representing the hardware configuration of servo driving
     * subsystem
     */
    public static TalonFXConfiguration kServoHardwareConfig =
        new TalonFXConfiguration()
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
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kSensorToMechRatio))
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKP(0.0)
                    .withKD(0.0)); // Torque Based Velocity Controls

    /** {@link ServoConfig} representing the full config of servo driving subsystem */
    public static ServoConfig kServoConfig =
        new ServoConfig(
            "Feeder",
            Matrix.kSubsystemCanBus,
            Matrix.kFeederId,
            kServoHardwareConfig,
            KilogramSquareMeters.of(0.001),
            kSensorToMechRatio,
            0,
            0);
  }

  /** Static constants class for the {@link Flywheel} subsystem */
  public static class FlywheelConstants {}

  /** Static constants class for the {@link Hood} subsystem */
  public static class HoodConstants {}
}
