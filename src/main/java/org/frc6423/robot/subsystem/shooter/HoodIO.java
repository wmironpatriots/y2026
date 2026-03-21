// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

public abstract class HoodIO {
  @Logged(name = "Is Connected (bool)", importance = Importance.CRITICAL)
  public abstract boolean isConnected();

  /** Update Logged Signals */
  public abstract void periodic();

  @Logged(name = "Output Voltage (volts)", importance = Importance.DEBUG)
  public abstract double getOutputVoltage();

  @Logged(name = "Stator Current (amps)", importance = Importance.DEBUG)
  public abstract double getStatorCurrentAmps();

  @Logged(name = "Supply Current (amps)", importance = Importance.DEBUG)
  public abstract double getSupplyCurrentAmps();

  @Logged(name = "Temperature (celsius)", importance = Importance.DEBUG)
  public abstract double getTemperatureCelsius();

  @Logged(name = "Position (revolutions)", importance = Importance.DEBUG)
  public abstract double getPositionRevs();

  public abstract void setTargetVoltage(double volts);

  public abstract void setTargetPosition(double positionRevs);

  public abstract void stop();

  public abstract void resetEncoder(double positionRevs);

  public abstract void setProfilingConstraints(double cruiseVelocity, double acceleration);

  public abstract void setGains(double kS, double kG, double kV, double kA, double kP, double kD);

  public abstract void enableBrake(boolean enabled);
}
