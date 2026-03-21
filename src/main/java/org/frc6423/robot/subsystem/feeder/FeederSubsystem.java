// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.feeder;

import com.ctre.phoenix6.CANBus;
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

/** {@link SubsystemBase} Roller subsystem responsible for feeding {@link ShooterSubsystem} */
public class FeederSubsystem extends SubsystemBase {
  /**
   * Static Factory for automatically configuring and creating a {@link FeederSubsystem}
   *
   * @return {@link FeederSubsystem}
   */
  public static FeederSubsystem create() {
    return (Robot.isReal())
        ? new FeederSubsystem(new RollerIOTalonFx(kCanDeviceId, kCanBus, kTalonConfig))
        : new FeederSubsystem(new RollerIONone());
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~
  /** {@link CANBus} CAN Bus hardware is connected to */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique id of servo */
  public static final int kCanDeviceId = Matrix.kFeederId;

  /** {@link TalonFXConfiguration} Hardware config of servo */
  public static final TalonFXConfiguration kTalonConfig =
      RollerIOTalonFx.createGenericRollerConfig(true);

  // * ~~~~~~~~ TUNABLES ~~~~~~~~
  /** {@link TunableNumber} Voltage speed to feed at */
  private static final TunableNumber kFeedingSpeedVolts =
      new TunableNumber("/Feeder/Feeding Speed (volts)", 4.0);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~
  @Logged private final RollerIO mHardware;

  public FeederSubsystem(RollerIO hardware) {
    mHardware = hardware;

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    mHardware.periodic();
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Feed fuel
   *
   * @return {@link Command}
   */
  public Command feed() {
    return this.run(() -> mHardware.setVoltageOutput(kFeedingSpeedVolts.get()));
  }

  /**
   * Stop feeder completely
   *
   * @return {@link Command}
   */
  public Command stop() {
    return this.runOnce(() -> mHardware.stop());
  }
}
