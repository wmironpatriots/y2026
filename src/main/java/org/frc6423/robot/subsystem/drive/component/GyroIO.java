// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

public abstract class GyroIO {
  /**
   * Get yaw rotation of gyro
   *
   * @return {@link Angle}
   */
  public Angle getYaw() {
    return getRotation2d().getMeasure();
  }

  /**
   * Get yaw rotation of gyro
   *
   * @return {@link Rotation2d}
   */
  public Rotation2d getRotation2d() {
    return getRotation3d().toRotation2d();
  }

  /**
   * Get rotation of gyro in 3-Dimensional Space
   *
   * @return {@link Rotation3d}
   */
  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public abstract Rotation3d getRotation3d();
}
