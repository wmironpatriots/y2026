// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.DIO;
import org.frc6423.lib.io.DIONone;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/** {@link SubsystemBase} Feeder Subsystem */
public class Feeder extends SubsystemBase {
  /** Constants for the {@link Feeder} */
  public class Constants {
    // * CONTROL CONSTANTS
    /** {@link Voltage} Voltage speed for loading */
    public static final Voltage kLoadingSpeed = Volts.of(3.0);

    /** {@link Voltage} Voltage speed for feeding */
    public static final Voltage kFeedingSpeed = Volts.of(9.0);

    // * SIMULATION CONSTANTS
    /** {@link Time} Time for loading in sim */
    public static final Time kLoadTime = Seconds.of(1);

    // * HARDWARE CONSTANTS
    /** {@link Integer} CAN ID of servo */
    public static final int kServoCanDeviceId = Matrix.kFeederId;

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

    /** {@link Integer} The DIO port beambreak is plugged into */
    public static final int kBeamBreakDioPort = Matrix.kFeederBeamBreakDio;
  }

  /**
   * Create new {@link Feeder}
   *
   * @return {@link Feeder}
   */
  public static Feeder create() {
    return (Robot.isReal())
        ? new Feeder(
            new ServoIOTalonFx(
                "Servo",
                Constants.kCanBus,
                Constants.kServoCanDeviceId,
                Constants.kServoTalonConfig),
            new DIONone())
        : new Feeder(new ServoIONone("Servo"), new DIONone());
  }

  @Logged private final ServoIO mServo;
  @Logged private final DIO mDIO;

  private boolean mIsRunning = false;

  private Timer mSimTimer = new Timer();

  /**
   * Create new {@link Feeder}
   *
   * @param servo {@link ServoIO} Servo powering subsystem
   * @param dio {@link DIO} Beambreak digital signal
   */
  protected Feeder(ServoIO servo, DIO dio) {
    mServo = servo;
    mDIO = dio;
  }

  @Override
  public void periodic() {
    // Update All Hardware
    mServo.periodic();
    mDIO.periodic();
  }

  // * GETTERS
  /**
   * Check if roller subsystem is running
   *
   * @return {@link Boolean}
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

  /**
   * Check if subsystem is loaded
   *
   * <p>If beambreak is broken, returns true
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Loaded (bool)", importance = Importance.INFO)
  public boolean isLoaded() {
    return !mDIO.getState();
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
        .withName("Feeder Stop");
  }

  /**
   * Request subsystem to load a fuel for shooting
   *
   * @return {@link Command}
   */
  public Command load() {
    return this.run(
            () -> {
              mSimTimer.restart();
              mServo.setVoltageSetpoint(Constants.kLoadingSpeed, true);
              mIsRunning = true;
            })
        .until(() -> (Robot.isReal()) ? isLoaded() : mSimTimer.hasElapsed(Constants.kLoadTime))
        .andThen(stop())
        .withName("Feeder Load");
  }

  /**
   * Request subsystem to start feeding fuel to shooter
   *
   * @return {@link Command}
   */
  public Command feed() {
    return this.run(
            () -> {
              mServo.setVoltageSetpoint(Constants.kFeedingSpeed, true);
              mIsRunning = true;
            })
        .withName("Feeder Feed");
  }
}
