// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** {@link MechSim} implementation for a Flywheel system */
public class FlywheelSim extends MechSim {
  private final DCMotorSim mSim;

  public FlywheelSim(Config config) {
    mSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                config.gearbox, config.flywheelMoi.in(KilogramSquareMeters), config.gearing),
            config.gearbox,
            0.001,
            0.001);
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(mSim.getCurrentDrawAmps());
  }

  @Override
  public Angle getAngle() {
    return mSim.getAngularPosition();
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return mSim.getAngularVelocity();
  }

  @Override
  protected void update(Time dt) {
    mSim.update(dt.in(Seconds));
  }

  @Override
  public void setInputVoltage(Voltage voltage) {
    mSim.setInputVoltage(voltage.in(Volts));
  }

  /**
   * Represents a configuration for a {@link FlywheelSim} system
   *
   * @param gearbox {@link DCMotor} representing the motor/gearbox of system
   * @param gearing double representing the gearing of gearbox
   * @param flywheelMoi {@link MomentOfInertia} representing the rotational inertia of the system
   */
  public static record Config(DCMotor gearbox, double gearing, MomentOfInertia flywheelMoi) {}
}
