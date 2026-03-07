// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.feeder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.util.NetworkTableUtil;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

// TODO beambreak
/**
 * {@link SubsystemBase} Controller for the feeder subsystem
 *
 * <p>{@link Feeder} is a roller drivebn by a single Kraken x60
 *
 * <p>The purpose of {@link Feeder} is to hold/provide fuel to the shooter when required
 *
 * <p>{@link Feeder} has three actions: neutral, load, feed
 *
 * <p>Neutral is where {@link Feeder} is stopped completely
 *
 * <p>Loading is where {@link Feeder} will get fuel ready for feed
 *
 * <p>Feeding is where {@link Feeder} will supply fuel for shooter
 */
public class Feeder extends SubsystemBase {
  /**
   * Create new {@link Feeder}
   *
   * @return {@link Feeder}
   */
  public Feeder create() {
    return (Robot.isReal())
        ? new Feeder(
            new ServoIOTalonFx("Servo", MotorType.KrakenX60, kCanBus, kCanDeviceId, kServoConfig))
        : new Feeder(new ServoIONone("Servo"));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus servo is on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device identifier for the feeder servo */
  public static final int kCanDeviceId = Matrix.kFeederId;

  /** {@link TalonFXConfiguration} Hardware config of servo */
  public static final TalonFXConfiguration kServoConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(40.0)
                  .withStatorCurrentLimitEnable(true));

  /** {@link Double} Time in seconds for loading to take in simulation */
  public static final double kSimLoadingTimeSec = 1.0;

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  private double mLoadingSpeedVolts = 3.0;
  private double mFeedingSpeedVolts = 9.0;
  private final DoubleEntry mLoadingSpeedTunable =
      NetworkTableUtil.createEntry("Tunables/Feeder/Loading Speed (volts)", mLoadingSpeedVolts);
  private final DoubleEntry mFeedingSpeedTunable =
      NetworkTableUtil.createEntry("Tunables/Feeder/Feeding Speed (volts)", mLoadingSpeedVolts);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final ServoIO mServo;

  private boolean mIsRunning = false;
  private final Debouncer mIsStuck = new Debouncer(0.1);

  protected Feeder(ServoIO servo) {
    mServo = servo;
  }

  @Override
  public void periodic() {
    mServo.periodic();

    mIsStuck.calculate(
        mIsRunning && MathUtil.isNear(0.0, mServo.getAngularVelocityRevsPerSec(), 0.1));

    if (Flags.kTuningModeEnabled) {
      mLoadingSpeedVolts = mLoadingSpeedTunable.get();
      mFeedingSpeedVolts = mFeedingSpeedTunable.get();
    }
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Check if subsystem is trying to move but is unable to
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Stuck (bool)", importance = Importance.INFO)
  public boolean isStuck() {
    return mIsStuck.calculate(
        mIsRunning && MathUtil.isNear(0.0, mServo.getAngularVelocityRevsPerSec(), 0.1));
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~
  /**
   * Constructs a sequence where subsystem stops completely
   *
   * @return {@link Command}
   */
  public Command getNeutralCmd() {
    return this.run(
        () -> {
          mServo.setNeutral();
          mIsRunning = false;
        });
  }

  /**
   * Constructs a sequence where subsystem continiously runs at feeding speed
   *
   * @return {@link Command}
   */
  public Command getFeedCmd() {
    return this.run(
        () -> {
          mServo.setVoltageOutput(mFeedingSpeedVolts, true);
          mIsRunning = true;
        });
  }
}
