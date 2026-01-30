// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.sim;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

/** Base class for mechanism simulations */
public abstract class MechSim {
  private double mPreviousUpdateTimestamp = 0.0;

  public abstract Current getStatorCurrent();

  /**
   * @return {@link Angle} representing the angular position of sim
   */
  public abstract Angle getAngle();

  /**
   * @return {@link AngularVelocity} representing the angular velocity of sim
   */
  public abstract AngularVelocity getAngularVelocity();

  /**
   * Update the simulation
   *
   * @param dt {@link Time} representing how long its been since last update
   */
  protected abstract void update(Time dt);

  /** Update the simulation */
  public void update() {
    double timestamp = Timer.getFPGATimestamp();
    update(Seconds.of(timestamp - mPreviousUpdateTimestamp));
    mPreviousUpdateTimestamp = timestamp;
  }

  /**
   * Set the input voltage for the sim
   *
   * @param volts {@link Voltage} representing the input
   */
  public abstract void setInputVoltage(Voltage volts);

  /**
   * Calculate the input voltage /w friction for a system
   *
   * @param inputVoltage {@link Voltage} representing the input of system
   * @param frictionVoltage {@link Voltage} representing the friction of the system
   * @return {@link Voltage}
   */
  public static final Voltage addFriction(Voltage inputVoltage, Voltage frictionVoltage) {
    if (inputVoltage.abs(Volts) < frictionVoltage.in(Volts)) {
      return Volts.of(0.0);
    } else if (inputVoltage.gt(Volts.zero())) {
      return inputVoltage.minus(frictionVoltage);
    } else {
      return inputVoltage.plus(frictionVoltage);
    }
  }
}
