// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

/** Abstract Interface for controlling a roller servo */
public abstract class RollerIO {
  /** Update Logged Signals */
  public abstract void periodic();

  /**
   * Check connection status to servo
   *
   * @return {@link Boolean} Connection status
   */
  @Logged(name = "Is Connected (bool)", importance = Importance.CRITICAL)
  public abstract boolean isConnected();

  /**
   * Get output voltage
   *
   * @return {@link Double} Voltage output in volts
   */
  @Logged(name = "Applied Voltage", importance = Importance.DEBUG)
  public abstract double getAppliedVoltage();

  /**
   * Set output voltage setpoint
   *
   * @param volts {@link Double} Desired voltage output in volts
   */
  public abstract void setVoltageOutput(double volts);

  /** Stop roller completely */
  public abstract void stop();
}
