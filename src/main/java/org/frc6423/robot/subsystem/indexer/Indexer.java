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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;

/** {@link SubsystemBase} extension representing the indexer subsystem */
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
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .withStatorCurrentLimitEnable(true));

    /** {@link Voltage} representing the indexing speed */
    private static final Voltage kIndexingSpeed = Volts.of(5);

    /** {@link Voltage} representing the outdexing speed */
    private static final Voltage kOutdexingSpeed = kIndexingSpeed.times(-1);
  }

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

  @Logged private final ServoIO mServo;

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
  }

  /**
   * Attempt to stop indexer
   *
   * @return {@link Command}
   */
  public Command stop() {
    return this.run(() -> mServo.stop());
  }

  /**
   * Attempt to index
   *
   * @return {@link Command}
   */
  public Command index() {
    return this.run(() -> setSpeed(Constants.kIndexingSpeed));
  }

  /**
   * Attempt to outdex
   *
   * @return {@link Command}
   */
  public Command outdex() {
    return this.run(() -> setSpeed(Constants.kOutdexingSpeed));
  }

  /**
   * Set servo speed
   *
   * @param speed {@link Voltage} representing desired indexer speed
   */
  private void setSpeed(Voltage speed) {
    mServo.setVoltageSetpoint(speed, true);
  }
}
