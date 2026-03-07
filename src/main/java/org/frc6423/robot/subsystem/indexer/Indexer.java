// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.indexer;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.util.NetworkTableUtil;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/**
 * {@link SubsystemBase} Controller for the indexing subsystem
 *
 * <p>{@link Indexer} is a belt system drien by a single Kraken x60
 *
 * <p>The purpose of {@link Indexer} is to push balls towards the {@link Feeder}
 *
 * <p>{@link Indexer} has three actions: neutral, pulse, and run. Each of these actions will be run
 * pointing towards the feeder
 */
@Logged(name = "Indexer")
public class Indexer extends SubsystemBase {
  /**
   * Create new {@link Indexer}
   *
   * @return {@link Indexer}
   */
  public Indexer create() {
    return (Robot.isReal())
        ? new Indexer(
            new ServoIOTalonFx("Servo", MotorType.KrakenX60, kCanBus, kCanDeviceId, kServoConfig))
        : new Indexer(new ServoIONone("Servo"));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus servo is on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device identifier for the indexer servo */
  public static final int kCanDeviceId = Matrix.kIndexerId;

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

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  private double mIndexingSpeedVolts = 6.5;
  private double mPulsePeriodSec = 0.5;
  private final DoubleEntry mIndexingSpeedTunable =
      NetworkTableUtil.createEntry("Tunables/Indexer/Indexing Speed (volts)", mIndexingSpeedVolts);
  private final DoubleEntry mPulsePeriodTunable =
      NetworkTableUtil.createEntry("Tunables/Indexer/Pulse Period (sec)", mPulsePeriodSec);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final ServoIO mServo;

  private boolean mIsRunning = false;
  private final Debouncer mIsStuck = new Debouncer(0.1);

  protected Indexer(ServoIO servo) {
    mServo = servo;
  }

  @Override
  public void periodic() {
    mServo.periodic();

    mIsStuck.calculate(
        mIsRunning && MathUtil.isNear(0.0, mServo.getAngularVelocityRevsPerSec(), 0.1));

    if (Flags.kTuningModeEnabled) {
      mIndexingSpeedVolts = mIndexingSpeedTunable.get();
      mPulsePeriodSec = mPulsePeriodTunable.get();
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
   * Constructs a sequence where subsystem continiously runs at indexing speed
   *
   * @return {@link Command}
   */
  public Command getIndexCmd() {
    return this.run(
        () -> {
          mServo.setVoltageOutput(mIndexingSpeedVolts, true);
          mIsRunning = true;
        });
  }

  /**
   * Constructs a sequence where subsystem periodically uns at indexing speed
   *
   * @return {@link Command}
   */
  public Command getPulseCmd() {
    return Commands.waitSeconds(mPulsePeriodSec)
        .andThen(getIndexCmd())
        .withTimeout(mPulsePeriodSec)
        .repeatedly();
  }
}
