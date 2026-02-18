// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.indexer;

import static edu.wpi.first.units.Units.Amps;
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the indexer subsystem
 *
 * <p>This subsystem's only component is a roller
 *
 * <p>when indexing, belt is rolling towards shooter
 *
 * <p>when outdexing, belt is rolling to intake
 */
public class Indexer extends SubsystemBase {
  /** {@link Indexer} subsystem constants */
  public class Constants {
    /** {@link CANbus} representing the bus devices are connected to */
    private static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    /** {@link Integer} representing the servo's CAN ID on CANBUS */
    private static final int kServoCanDeviceId = Matrix.kIndexerId;

    /** {@link TalonFXConfiguration} representing the hardware config of the servo */
    private static final TalonFXConfiguration kServoTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(20.0))
                    .withStatorCurrentLimitEnable(true));

    /** {@link Voltage} representing the indexing speed */
    private static final Voltage kIndexingSpeed = Volts.of(5);

    /** {@link Voltage} representing the outdexing speed */
    private static final Voltage kOutdexingSpeed = kIndexingSpeed.times(-1);
  }

  @Logged private final ServoIO mServo;

  private State mState = State.STOPPED;

  /**
   * Create new {@link Indexer}
   *
   * @return {@link Indexer}
   */
  public static Indexer create() {
    // TODO sim
    return new Indexer(
        new ServoIOTalonFx(
            "IndexerServo",
            Constants.kCanBus,
            Constants.kServoCanDeviceId,
            Constants.kServoTalonConfig));
  }

  /**
   * Create new {@link Indexer}
   *
   * @param servo {@link ServoIO} representing roller servo
   */
  protected Indexer(ServoIO servo) {
    mServo = servo;
  }

  @Override
  public void periodic() {
    // Update hardware
    mServo.periodic();

    // Run state logic
    switch (mState) {
      case STOPPED:
        mServo.stop();
        break;

      case RUNNING_IN:
        setSpeed(Constants.kIndexingSpeed);
        break;

      case RUNNING_OUT:
        setSpeed(Constants.kOutdexingSpeed);
        break;
    }
  }

  /**
   * Set servo speed
   *
   * @param speed {@link Voltage} representing desired indexer speed
   */
  private void setSpeed(Voltage speed) {
    mServo.setVoltageSetpoint(speed, true);
  }

  /**
   * @return {@link State} representing the current mode of being subsystem is in
   */
  @Logged(name = "State", importance = Importance.INFO)
  public State getState() {
    return mState;
  }

  /** Request subsystem to stop */
  public void stop() {
    mState = State.STOPPED;
  }

  /** Request subsystem to start indexing */
  public void index() {
    mState = State.RUNNING_IN;
  }

  /** Request subsystem to start outdexing */
  public void outdex() {
    mState = State.RUNNING_OUT;
  }

  /** Represents a mode of being the {@link Indexer} subsystem can be in */
  public static enum State {
    /** {@link State} where the {@link Indexer} is not running */
    STOPPED,
    /** {@link State} where the {@link Indexer} is running */
    RUNNING_IN,
    /** {@link State} where the {@link Indexer} is running inverse */
    RUNNING_OUT
  }
}
