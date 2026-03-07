// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ServoIOTalonFxFlywheelSim extends ServoIOTalonFx {
  private final FlywheelSim mModel;

  private final double mRatio;

  private double mPreviousTimestamp;
  private final Notifier mNotifier;

  protected ServoIOTalonFxFlywheelSim(
      String name,
      MotorType type,
      CANBus canBus,
      int canDeviceId,
      TalonFXConfiguration config,
      double rotationalInertiaKgSquaredMeters,
      DCMotor gearbox,
      double rotorToMechRatio) {
    super(name, type, canBus, canDeviceId, config);

    mRatio = rotorToMechRatio;

    // Define Physics Model
    mModel =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                gearbox, rotationalInertiaKgSquaredMeters, rotorToMechRatio),
            gearbox);

    // Configure Phoneix 6 Sim
    mTalonConfig =
        mTalonConfig.withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(mRatio));

    mServo.getConfigurator().apply(mTalonConfig);

    mServo.getSimState().setMotorType(type);
    mServo.getSimState().Orientation =
        mTalonConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    // Start loop
    mNotifier =
        new Notifier(
            () -> {
              final double timestamp = Timer.getFPGATimestamp();
              final double deltaTime = timestamp - mPreviousTimestamp;
              mPreviousTimestamp = timestamp;

              mServo.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

              mModel.setInputVoltage(mServo.getSimState().getMotorVoltage());
              mModel.update(deltaTime);

              // mServo
              //     .getSimState()
              //     .setRawRotorPosition((mModel.getAngleRads() / (Math.PI * 2)) * mRatio);
              mServo
                  .getSimState()
                  .setRotorVelocity(
                      (mModel.getAngularVelocityRadPerSec() / (Math.PI * 2)) * mRatio);
            });

    mNotifier.startPeriodic(0.002);
  }
}
