// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.DIO;
import org.frc6423.lib.io.DIORio;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the feeder subsystem
 *
 * <p>This subsystem's only component is a roller
 *
 * <p>The feeder will only spin towards the shooter
 */
public class Feeder extends SubsystemBase {
  /** {@link Feeder} subsystem constants */
  public class Constants {
    /** {@link CANBus} representing the bus devices are connected to */
    private static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    /** {@link Integer} representing the servo's CAN ID on CANBUS */
    private static final int kServoCanDeviceId = Matrix.kFeederId;

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

    /** {@link Voltage} representing the feeding speed */
    private static final Voltage kFeedingSpeed = Volts.of(9.0);

    /** {@link Voltage} representing the slow loading speed */
    private static final Voltage kLoadingSpeed = Volts.of(3.0);

    /** {@link Integer} representing the hardware config of the beam break */
    private static final int kBeamBreakPort = 0;
  }

  @Logged private final ServoIO mServo;

  @Logged private final DIO mDIO;

  private State mState = State.STOPPED;

  /**
   * Create new {@link Feeder}
   *
   * @return {@link Feeder}
   */
  public static Feeder create() {
    return new Feeder(
        new ServoIOTalonFx(
            "FeederServo",
            Constants.kCanBus,
            Constants.kServoCanDeviceId,
            Constants.kServoTalonConfig),
        new DIORio(0));
  }

  /**
   * Create new {@link Feeder}
   *
   * @param servo {@link ServoIO} representing roller servo
   */
  protected Feeder(ServoIO servo, DIO dio) {
    mServo = servo;
    mDIO = dio;
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

      case LOADED:
        mServo.stop();
        break;

      case LOADING:
        if (mDIO.getState()) {
          mState = State.LOADED;
          mServo.stop();
        } else {
          setSpeed(Constants.kLoadingSpeed);
        }
        break;

      case FEEDING:
        setSpeed(Constants.kFeedingSpeed);
        break;
    }
  }

  /**
   * Set servo speed
   *
   * @param speed {@link Voltage} representing desired feeder speed
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

  /**
   * Request feeder to stop
   *
   * @return {@link Command}
   */
  public Command stop() {
    return Commands.runOnce(
        () -> {
          mState = State.STOPPED;
        });
  }

  /**
   * Request feeder to load game pieces
   *
   * @return {@link Command}
   */
  public Command load() {
    return Commands.runOnce(
        () -> {
          mState = State.LOADING;
        });
  }

  /**
   * Request feeder to feed
   *
   * @return {@link Command}
   */
  public Command feed() {
    return Commands.runOnce(
        () -> {
          mState = State.FEEDING;
        });
  }

  /** Represents a mode of being the {@link Feeder} subsystem can be in */
  public static enum State {
    /** {@link State} where the {@link Feeder} is fully stopped */
    STOPPED,
    /** {@link State} where the {@link Feeder} is slowly loading a ball */
    LOADING,
    /** {@link State} where the {@link Feeder} is stopped with a ball loaded */
    LOADED,
    /** {@link State} where the {@link Feeder} is feeding at full speed */
    FEEDING
  }
}
