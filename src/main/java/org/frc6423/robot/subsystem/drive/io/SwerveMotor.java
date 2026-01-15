// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Temperature;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.subsystem.drive.PhoneixOdometryThread;

public class SwerveMotor extends ServoIOTalonFx {
  public static final Temperature kDefaultTemperature = DEFAULT_SHUTDOWN_TEMPERATURE;

  public SwerveMotor(String name, int canDeviceId, CANBus canBus, TalonFXConfiguration config) {
    super(name, canDeviceId, canBus, config, kDefaultTemperature);

    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoneixOdometryThread.kThreadFrequency,
        inCurrentSignal,
        outCurrentSignal,
        torqueCurrentSignal,
        angleSignal,
        velocitySignal,
        accelerationSignal,
        temperatureSignal);

    servo.optimizeBusUtilization(PhoneixOdometryThread.kThreadFrequency);
  }
}
