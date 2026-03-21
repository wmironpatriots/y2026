// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.RollerIO;
import org.frc6423.lib.io.RollerIONone;
import org.frc6423.lib.io.RollerIOTalonFx;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/**
 * {@link SubsystemBase} Roller subsystem responsible for moving fuel towards {@link
 * FeederSubsystem}
 */
public class IndexerSubsystem extends SubsystemBase {
  /**
   * Static Factory for automatically configuring and creating a {@link IndexerSubsystem}
   *
   * @return {@link IndexerSubsystem}
   */
  public static IndexerSubsystem create() {
    return (Robot.isReal())
        ? new IndexerSubsystem(new RollerIOTalonFx(kCanDeviceId, kCanBus, kTalonConfig))
        : new IndexerSubsystem(new RollerIONone());
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~
  /** {@link CANBus} CAN Bus hardware is connected to */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique id of servo */
  public static final int kCanDeviceId = Matrix.kIndexerId;

  /** {@link TalonFXConfiguration} Hardware config of servo */
  public static final TalonFXConfiguration kTalonConfig =
      RollerIOTalonFx.createGenericRollerConfig(true)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(30.0)
                  .withStatorCurrentLimitEnable(true));

  // * ~~~~~~~~ TUNABLES ~~~~~~~~
  /** {@link TunableNumber} Voltage speed to index at */
  private static final TunableNumber kIndexingSpeedVolts =
      new TunableNumber("/Indexer/Indexing Speed (volts)", 6.5);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~
  @Logged private final RollerIO mHardware;

  protected IndexerSubsystem(RollerIO hardware) {
    mHardware = hardware;

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    mHardware.periodic();
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Push fuel to feeder
   *
   * @return {@link Command}
   */
  public Command index() {
    return this.run(() -> mHardware.setVoltageOutput(kIndexingSpeedVolts.get()));
  }

  public Command feedInverse() {
    return this.run(() -> mHardware.setVoltageOutput(kIndexingSpeedVolts.get() * -1));
  }

  /**
   * Stop running
   *
   * @return {@link Command}
   */
  public Command stop() {
    return this.runOnce(() -> mHardware.stop());
  }
}
