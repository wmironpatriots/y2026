// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulated extension of {@link ServoIOTalonFx} */
public class ServoIOTalonFxSim extends ServoIOTalonFx {
  private final DCMotorSim sim;

  private boolean invertVoltage = false;

  private Angle simAngle = Radians.of(0.0);

  private final Notifier simUpdater;
  private double previousUpdateTimestamp = 0.0;

  /**
   * Create new {@link ServoIOTalonFxSim}
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer representing the CAN identification
   * @param canBusId {@link CANBus} representing the CAN bus device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   * @param jKgMetersSquared Moment of Inertia in jKgM^2
   */
  public ServoIOTalonFxSim(
      String name,
      int canDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      double jKgMetersSquared) {
    super(name, canDeviceId, canBus, config, Celsius.of(9999));

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                jKgMetersSquared,
                1.0 / config.Feedback.SensorToMechanismRatio),
            DCMotor.getKrakenX60Foc(1),
            0.001,
            0.001);

    servo.getSimState().Orientation =
        config.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    simUpdater = new Notifier(() -> updateSimulation());
    simUpdater.startPeriodic(0.005);
  }

  /** Update {@link DCMotorSim} model and {@link TalonFX} {@link TalonFXSimState} */
  private void updateSimulation() {
    var simState = servo.getSimState();

    // Calculate voltage /w friction from Talon sim state & apply to DCMotorSim model
    double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);
    simVoltage *= invertVoltage ? -1 : 1;
    sim.setInputVoltage(simVoltage);

    // Update DCMotorSim
    double timestamp = Timer.getFPGATimestamp();
    sim.update(timestamp - previousUpdateTimestamp);
    previousUpdateTimestamp = timestamp;

    simAngle = sim.getAngularPosition();

    // Set Talon sim state raw rotor pose and velocity from DCMotorSim model's state
    simState.setRawRotorPosition(simAngle.div(config.Feedback.SensorToMechanismRatio));
    simState.setRotorVelocity(sim.getAngularVelocity().div(config.Feedback.SensorToMechanismRatio));
  }

  /**
   * @return angle of the {@link DCMotorSim} model
   */
  @Logged(name = "Physics Simulation Angle", importance = Importance.INFO)
  public Angle getPhysicsSimulationAngle() {
    return simAngle;
  }

  /**
   * Enable inverted voltage
   *
   * @param enabled boolean representing if inverted voltage should be enabled or disabled
   */
  public void setInvertedVoltage(boolean enabled) {
    invertVoltage = enabled;
  }

  /**
   * @param motorVoltage
   * @param frictionVoltage
   * @return
   */
  public static final double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }

    return motorVoltage;
  }
}
