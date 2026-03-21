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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim extends FlywheelIOReal {
  private final DCMotorSim mPhysicsModel;

  private final Notifier mSimulationNotifier;
  private double mPreviousUpdateTimestamp = 0.0;

  public FlywheelIOSim(
      int leftCanDeviceId,
      int rightCanDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      double gearRatio) {
    super(leftCanDeviceId, rightCanDeviceId, canBus, config);

    // Setup physics model
    mPhysicsModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2),
                ShooterSubsystem.kFlywheelRotationalInertiaKgSquaredMeters,
                gearRatio),
            DCMotor.getKrakenX60Foc(2));

    // Setup talonfx sim
    mLeft.getSimState().setMotorType(MotorType.KrakenX60);
    mLeft.getSimState().Orientation =
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
              mLeft.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

              // Update physics model
              mPhysicsModel.setInputVoltage(mLeft.getSimState().getMotorVoltage());
              mPhysicsModel.update(dt);

              // Update motor model
              mLeft
                  .getSimState()
                  .setRawRotorPosition(
                      Units.radiansToRotations(mPhysicsModel.getAngularPositionRad()) * gearRatio);
              mLeft
                  .getSimState()
                  .setRotorVelocity(
                      Units.radiansToRotations(mPhysicsModel.getAngularVelocityRadPerSec())
                          * gearRatio);
            });

    mSimulationNotifier.startPeriodic(0.002);
  }
}
