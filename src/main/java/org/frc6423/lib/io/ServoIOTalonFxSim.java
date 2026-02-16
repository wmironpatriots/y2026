// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Notifier;
import org.frc6423.lib.sim.MechSim;

/** {@link ServoIOTalon} extension for simulation */
public class ServoIOTalonFxSim extends ServoIOTalonFx {
  private final MechSim mModel;
  private final Notifier mUpdater;

  /**
   * Create new {@link ServoIOTalonFxSim}
   *
   * @param name {@link String} representing servo nickname
   * @param canBus {@link CANBus} representing CAN bus loop device is in
   * @param canDeviceId {@link Integer} representing the id of CAN device
   * @param talonConfig {@link TalonFXConfiguration} representing the servo config
   * @param motorKt {@link Double} representing the servo's kT rating
   * @param model {@link MechSim} representing the physics simulation to use for modeling system
   */
  public ServoIOTalonFxSim(
      String name, CANBus canBus, int deviceId, TalonFXConfiguration talonConfig, MechSim model) {
    super(name, canBus, deviceId, talonConfig);

    mModel = model;

    mServo.getSimState().Orientation =
        mTalonConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    mUpdater = new Notifier(() -> updateSimulation());
    mUpdater.startPeriodic(0.005);
  }

  /** Update high fidelity sim */
  protected void updateSimulation() {
    mServo.getSimState().setSupplyVoltage(12.0);

    // Set system model input voltage using simulation input
    mModel.setInputVoltage(
        MechSim.addFriction(mServo.getSimState().getMotorVoltageMeasure(), Volts.of(0.25)));

    // Update system model
    mModel.update();

    // Update simulation using system model
    mServo
        .getSimState()
        .setRawRotorPosition(mModel.getAngle().times(mTalonConfig.Feedback.SensorToMechanismRatio));
    mServo
        .getSimState()
        .setRotorVelocity(
            mModel.getAngularVelocity().times(mTalonConfig.Feedback.SensorToMechanismRatio));
  }

  @Override
  public Angle getAngle() {
    return mModel.getAngle();
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return mModel.getAngularVelocity();
  }

  @Override
  public Current getStatorCurrent() {
    return mModel.getStatorCurrent();
  }
}
