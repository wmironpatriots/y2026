// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Volts;

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
   * @param config {@link SwerveConfig} representing the configuration of servo
   * @param model {@link MechSim} representing the physics simulation to use for modeling system
   */
  protected ServoIOTalonFxSim(ServoConfig config, MechSim model) {
    super(config);

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
