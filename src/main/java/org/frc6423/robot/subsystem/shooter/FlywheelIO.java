// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

public abstract class FlywheelIO {
  @Logged(name = "Is Left Connected (bool)", importance = Importance.CRITICAL)
  public abstract boolean isLeftConnected();

  @Logged(name = "Is Right Connected (bool)", importance = Importance.CRITICAL)
  public abstract boolean isRightConnected();

  /** Update Logged Signals */
  public abstract void periodic();

  @Logged(name = "Output Voltage (volts)", importance = Importance.DEBUG)
  public abstract double getOutputVoltage();

  @Logged(name = "Stator Current (amps)", importance = Importance.DEBUG)
  public abstract double getStatorCurrentAmps();

  @Logged(name = "Supply Current (amps)", importance = Importance.DEBUG)
  public abstract double getSupplyCurrentAmps();

  @Logged(name = "Left Temperature (celsius)", importance = Importance.DEBUG)
  public abstract double getLeftTemperatureCelsius();

  @Logged(name = "Right Temperature (celsius)", importance = Importance.DEBUG)
  public abstract double getRightTemperatureCelsius();

  @Logged(name = "Position (revolutions)", importance = Importance.DEBUG)
  public abstract double getPositionRevs();

  @Logged(name = "Velocity (revolutions per second)", importance = Importance.DEBUG)
  public abstract double getVelocityRevsPerSec();

  @Logged(name = "Velocity (revolutions per second per second)", importance = Importance.DEBUG)
  public abstract double getAccelerationRevsPerSecPerSec();

  public abstract void setTargetTorqueCurrent(double amps);

  public abstract void setTargetVelocity(double revsPerSec);

  public abstract void setTargetVelocity(double revsPerSec, double feedforwardAmps);

  public abstract void stop();

  public abstract void setProfilingConstraints(double cruiseVelocity, double acceleration);

  public abstract void setGains(double kS, double kG, double kV, double kA, double kP, double kD);

  public abstract void enableBrake(boolean enabled);
}
