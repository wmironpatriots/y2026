// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim extends HoodIOReal {
  private final SingleJointedArmSim mPhysicsModel;

  private final Notifier mSimulationNotifier;
  private double mPreviousUpdateTimestamp = 0.0;

  public HoodIOSim(
      int canDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      double gearRatio,
      Rotation2d minAngle,
      Rotation2d maxAngle) {
    super(canDeviceId, canBus, config);

    // Setup physics model
    mPhysicsModel =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            gearRatio,
            0.1,
            Units.inchesToMeters(10),
            minAngle.getRadians(),
            maxAngle.getRadians(),
            true,
            minAngle.getRadians());

    // Setup talonfx sim
    mServo.getSimState().setMotorType(MotorType.KrakenX44);
    mServo.getSimState().Orientation =
        config.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    // Main sim thread
    mSimulationNotifier =
        new Notifier(
            () -> {
              // Get change in time since last update
              double currentTime = Utils.getCurrentTimeSeconds();
              double dt = currentTime - mPreviousUpdateTimestamp;
              mPreviousUpdateTimestamp = currentTime;

              // Set simulated battery voltage
              mServo.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

              // Update physics model
              mPhysicsModel.setInputVoltage(mServo.getSimState().getMotorVoltage());
              mPhysicsModel.update(dt);

              // Update motor model
              mServo
                  .getSimState()
                  .setRawRotorPosition(
                      Units.radiansToRotations(mPhysicsModel.getAngleRads()) * gearRatio);
              mServo
                  .getSimState()
                  .setRotorVelocity(
                      Units.radiansToRotations(mPhysicsModel.getVelocityRadPerSec()) * gearRatio);
            });

    mSimulationNotifier.startPeriodic(0.002);
  }
}
