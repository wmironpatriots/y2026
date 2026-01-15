// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** A hardware interface for controlling a gyro */
public abstract class GyroIO {
  public abstract void periodic();

  /**
   * @return {@link Rotation2d} representing the yaw rotation of gyro
   */
  public Rotation2d getRotation2d() {
    return getRotation3d().toRotation2d();
  }

  /**
   * @return {@link Rotation3d} representing the rotation of gyro in 3d space
   */
  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public abstract Rotation3d getRotation3d();
}
