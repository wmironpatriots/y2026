// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

/** A hardware interface for controlling a gyro */
public abstract class GyroIO {
  /** Update logged signals */
  public abstract void periodic();

  /**
   * @return {@link Angle} representing measured yaw
   */
  public Angle getYaw() {
    return getRotation2d().getMeasure();
  }

  /**
   * @return {@link Rotation2d} representing the measured orientation in 2D space
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return getRotation3d().toRotation2d();
  }

  /**
   * @return {@link Rotation3d} representing the measured orientation in 3D space
   */
  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public abstract Rotation3d getRotation3d();
}
