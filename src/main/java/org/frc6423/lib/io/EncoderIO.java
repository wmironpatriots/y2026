// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;

/**
 * A Hardware Interface for controlling a servo A {@link EncoderIO} instance <strong>must</strong>
 * have its <strong>periodic method called every robot loop for values to be properly
 * logged</strong>
 */
public abstract class EncoderIO {
  /** Update all logged values */
  public abstract void periodic();

  /**
   * Reset encoder to specified angular position
   *
   * @param angle {@link Angle} angular position to reset to
   */
  public abstract void reset(Angle angle);

  /**
   * @return {@link Angle} representing angular position of encoder
   */
  @Logged(name = "Angle", importance = Importance.INFO)
  public abstract Angle getAngle();
}
