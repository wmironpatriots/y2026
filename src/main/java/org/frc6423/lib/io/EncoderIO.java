// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;

/** A hardware interface for controlling an encoder */
public abstract class EncoderIO {
  public abstract void periodic();

  /**
   * @return {@link Rotation2d} representing rotational position of encoder
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public abstract Rotation2d getRotation2d();
}
