// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;

/** An {@link IO} for controlling an encoder */
public abstract class EncoderIO extends IO {
  /**
   * Reset encoder to specified angular position
   *
   * @param angle {@link Angle} angular position to reset to
   */
  public abstract void reset(Angle angle);

  /**
   * @return {@link Angle} representing angular posotion of encooder
   */
  @Logged(name = "Angle", importance = Importance.INFO)
  public abstract Angle getAngle();
}
