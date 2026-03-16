// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import edu.wpi.first.math.geometry.Rotation2d;

/** Abstract Interface for interacting with a Gyro */
public abstract class GyroIO {
  public abstract void periodic();

  public abstract Rotation2d getYawRotation2d();
}
