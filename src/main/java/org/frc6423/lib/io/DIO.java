// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

/**
 * An {@link IO} for controlling a DIO device
 *
 * @see https://docs.wpilib.org/en/stable/docs/hardware/sensors/digital-inputs-hardware.html
 */
public abstract class DIO extends IO {
  /**
   * @return true if DIO port returns 1
   */
  @Logged(name = "RawState", importance = Importance.INFO)
  public abstract boolean getRawState();
}
