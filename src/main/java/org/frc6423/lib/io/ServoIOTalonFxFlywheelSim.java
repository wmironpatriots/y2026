// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** {@link ServoIOTalon} extension for flywheel system simulation */
public class ServoIOTalonFxFlywheelSim extends ServoIOTalonFx {
  private final FlywheelSim mPhysicsModel;

  private double previousTimestamp;
  private final Notifier mNotifier;

  /**
   * Create new {@link ServoIOTalonFxRollerSim}
   *
   * @param name {@link String} representing servo nickname
   * @param canBus {@link CANBus} representing CAN bus loop device is in
   * @param canDeviceId {@link Integer} representing the id of CAN device
   * @param talonConfig {@link TalonFXConfiguration} representing the servo config
   * @param rotationalInertia {@link MomentOfInertia} representing the rotational inertia of system
   * @param motorType {@link MotorType} representing the type of motors used for gearbox input
   * @param gearbox {@link DCMotor} representing the gearbox input
   * @param sensorToMechanismRatio {@link Double} representing the gear ratio between the encoder
   *     and mechanism output
   */
  public ServoIOTalonFxFlywheelSim(
      String name,
      CANBus canBus,
      int deviceId,
      TalonFXConfiguration talonConfig,
      MomentOfInertia rotationalInertia,
      MotorType motorType,
      DCMotor gearbox,
      double sensorToMechanismRatio) {
    super(name, canBus, deviceId, talonConfig);

    mPhysicsModel =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                gearbox, rotationalInertia.in(KilogramSquareMeters), sensorToMechanismRatio),
            gearbox);

    mServo.getSimState().setMotorType(motorType);
    mServo.getSimState().Orientation =
        mTalonConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    mNotifier =
        new Notifier(
            () -> {
              final double timestamp = Timer.getFPGATimestamp();
              final double deltaTime = timestamp - previousTimestamp;
              previousTimestamp = timestamp;

              mServo.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

              mPhysicsModel.setInputVoltage(mServo.getSimState().getMotorVoltage());
              mPhysicsModel.update(deltaTime);

              // ! Not going to add something for setRawRotorPosition cuz flywheel sim doesn't have
              // a way to return it
              mServo
                  .getSimState()
                  .setRotorVelocity(
                      mPhysicsModel.getAngularVelocity().in(RotationsPerSecond)
                          * sensorToMechanismRatio);
            });

    mNotifier.startPeriodic(0.002);
  }
}
