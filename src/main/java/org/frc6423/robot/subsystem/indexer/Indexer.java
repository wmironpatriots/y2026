// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/** {@link SubsystemBase} extension representing the belt indexer subsystem */
public class Indexer extends SubsystemBase {
  /** Constants for the {@link Indexer} */
  public class Constants {
    // * CONTROL CONSTANTS
    /** {@link Voltage} Voltage speed for indexing */
    public static final Voltage kIndexingSpeed = Volts.of(5.0);

    /** {@link Voltage} Voltage speed for outdexing */
    public static final Voltage kOutdexingSpeed = kIndexingSpeed.times(-1);

    // * HARDWARE CONSTANTS
    /** {@link Integer} CAN ID of servo */
    public static final int kServoCanDeviceId = Matrix.kIndexerId;

    /** {@link Current} Stator current limit of servo */
    public static final Current kServoStatorCurrentLimit = Amps.of(40.0);

    /** {@link TalonFXConfiguration} Hardware config of servo */
    public static final TalonFXConfiguration kServoTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(kServoStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true));

    /** {@link CANBus} CAN bus devices are on */
    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;
  }

  /**
   * Create new {@link Indexer}
   *
   * @return {@link Indexer}
   */
  public static Indexer create() {
    return (Robot.isReal())
        ? new Indexer(
            new ServoIOTalonFx(
                "Servo",
                Constants.kCanBus,
                Constants.kServoCanDeviceId,
                Constants.kServoTalonConfig))
        : new Indexer(new ServoIONone("Servo"));
  }

  @Logged private final ServoIO mServo;

  private boolean mIsRunning = false;

  /**
   * Create new {@link Indexer}
   *
   * @param servo {@link ServoIO} Servo powering subsystem
   */
  protected Indexer(ServoIO servo) {
    mServo = servo;
  }

  @Override
  public void periodic() {
    // Update Hardware
    mServo.periodic();
  }

  // * GETTERS
  /**
   * Check if roller subsystem is running
   *
   * @return
   */
  @Logged(name = "Is Running (bool)", importance = Importance.INFO)
  public boolean isRunning() {
    return mIsRunning;
  }

  /**
   * Check if roller subsystem is stuck
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Stuck (bool)", importance = Importance.INFO)
  public boolean isStuck() {
    return !mIsRunning && !(Math.abs(mServo.getAngularVelocity().in(RadiansPerSecond)) > 0.0);
  }

  // * COMMANDS
  /**
   * Request subsystem to stop
   *
   * @return {@link Command}
   */
  public Command stop() {
    return this.run(
            () -> {
              mIsRunning = false;
              mServo.stop();
            })
        .withName("Indexer Stop");
  }

  /**
   * Request subsystem to run inwards and 'index'
   *
   * @return {@link Command}
   */
  public Command index() {
    return this.run(
            () -> {
              mIsRunning = true;
              mServo.setVoltageSetpoint(Constants.kIndexingSpeed, true);
            })
        .withName("Indexer Index");
  }

  /**
   * Request subsystem to run outwards and 'outdex' (aka eject)
   *
   * @return {@link Command}
   */
  public Command outdex() {
    return this.run(
            () -> {
              mIsRunning = true;
              mServo.setVoltageSetpoint(Constants.kOutdexingSpeed, true);
            })
        .withName("Indexer Outdex");
  }
}
