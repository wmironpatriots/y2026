// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Revolutions;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class EncoderIOCanCoderSim extends EncoderIOCanCoder {
  private double previousTimestamp;
  private final Notifier mNotifier;

  private Supplier<Angle> mRawAngleOverride = () -> Revolutions.zero();

  public EncoderIOCanCoderSim(int canDeviceId, CANBus canBusId, CANcoderConfiguration config) {
    super(canDeviceId, canBusId, config);

    mEncoder.getSimState().Orientation =
        mConfig.MagnetSensor.SensorDirection == SensorDirectionValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    mNotifier =
        new Notifier(
            () -> {
              final double timestamp = Timer.getFPGATimestamp();
              final double deltaTime = timestamp - previousTimestamp;
              previousTimestamp = timestamp;

              mEncoder.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

              mEncoder.getSimState().setRawPosition(mRawAngleOverride.get());
            });

    mNotifier.startPeriodic(0.002);
  }

  public void setRawAngleOverride(Supplier<Angle> angleSource) {
    mRawAngleOverride = angleSource;
  }
}
