// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** {@link MechSim} implementation for a pivot system */
public class PivotMechSim extends MechSim {
  private final SingleJointedArmSim mSim;

  /**
   * Create new {@link PivotMechSim}
   *
   * @param config {@link Config} representing the system config
   */
  public PivotMechSim(Config config) {
    mSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                config.gearbox, config.armMoi.in(KilogramSquareMeters), config.gearing),
            config.gearbox,
            config.gearing,
            config.armLength.in(Meters),
            config.minAngle.in(Radians),
            config.maxAngle.in(Radians),
            config.simulateGravity,
            config.initAngle.in(Radians),
            0.0,
            0.0);
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(mSim.getCurrentDrawAmps());
  }

  @Override
  public Angle getAngle() {
    return Radians.of(mSim.getAngleRads());
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return RadiansPerSecond.of(mSim.getVelocityRadPerSec());
  }

  @Override
  protected void update(Time dt) {
    mSim.update(dt.in(Seconds));
  }

  @Override
  public void setInputVoltage(Voltage volts) {
    mSim.setInputVoltage(volts.in(Volts));
  }

  /**
   * Represents a configuration for a {@link PivotMechSim} system
   *
   * @param gearbox {@link DCMotor} representing the motor/gearbox of system
   * @param gearing double representing the gearing of gearbox
   * @param armLength {@link Distance} representing the length of the system's arm
   * @param armMoi {@link MomentOfInertia} representing the rotational inertia of the system's arm
   * @param simulateGravity when true, the system will simulate gravity
   * @param minAngle {@link Angle} representing the lower angular position limit of system
   * @param maxAngle {@link Angle} representing the high angular position limit of system
   * @param initAngle {@link Angle} representing the angular position the system should start at
   * @param stdevs represents the error the system should simulate
   */
  public static record Config(
      DCMotor gearbox,
      double gearing,
      Distance armLength,
      MomentOfInertia armMoi,
      boolean simulateGravity,
      Angle minAngle,
      Angle maxAngle,
      Angle initAngle) {}
}
