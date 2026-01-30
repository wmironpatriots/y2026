// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc6423.lib.sim.MechSim;

/** Simulated extension of {@link ServoIOTalonFx} */
public class ServoIOTalonFxSim extends ServoIOTalonFx {
  private final MechSim mModel;
  private final Notifier mUpdater;

  /**
   * Create new {@link ServoIOTalonFxSim}
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer ID on CAN loop
   * @param canBusId {@link CANBus} representing the CAN bus device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   * @param sim {@link MechSim} representing the simulation model
   */
  public ServoIOTalonFxSim(
      String name, int canDeviceId, CANBus canBus, TalonFXConfiguration config, MechSim sim) {
    super(name, canDeviceId, canBus, config);

    mModel = sim;

    mServo.getSimState().Orientation =
        config.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    mUpdater = new Notifier(() -> updateSimulation());
    // mUpdater.startPeriodic(0.005);
  }

  @Override
  public void periodic() {
    super.periodic();
    updateSimulation();
  }

  /** Update {@link DCMotorSim} model and {@link TalonFX} {@link TalonFXSimState} */
  private void updateSimulation() {
    // Set system model input voltage using simulation input
    mModel.setInputVoltage(
        mServo.getSimState().getMotorVoltageMeasure().times(getInvertedMultiplier()));

    // Update system model
    mModel.update();

    // Update simulation using system model
    mServo
        .getSimState()
        .setRawRotorPosition(mModel.getAngle().div(mConfig.Feedback.SensorToMechanismRatio));
    mServo
        .getSimState()
        .setRotorVelocity(mModel.getAngularVelocity().div(mConfig.Feedback.SensorToMechanismRatio));
    mServo.getSimState().setSupplyVoltage(12.0);
  }

  /**
   * @return double representing the multiplier accounting for the motor's inversion
   */
  protected double getInvertedMultiplier() {
    return mConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive ? 1.0 : -1.0;
  }

  @Override
  public Voltage getAppliedVoltage() {
    return mServo.getSimState().getMotorVoltageMeasure().times(getInvertedMultiplier());
  }

  public Current getPhysicsModelStatorCurrent() {
    return mModel.getStatorCurrent();
  }

  @Override
  public Current getSupplyCurrent() {
    return mServo.getSimState().getSupplyCurrentMeasure().times(getInvertedMultiplier());
  }

  /**
   * @return {@link Angle} representing the angular position of the {@link MechSim} model
   */
  @Logged(name = "Physics Model Angle", importance = Importance.INFO)
  public Angle getPhysicsModelAngle() {
    return mModel.getAngle();
  }

  /**
   * @return {@link AngularVelocity} representing the angular velocity of the {@link MechSim} model
   */
  @Logged(name = "Physics Model Angular Velocity", importance = Importance.INFO)
  public AngularVelocity getPhysicsModelAngularVelocity() {
    return mModel.getAngularVelocity();
  }
}
