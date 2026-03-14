// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

public class GyroIONone extends GyroIO {
  @Override
  public double getYawDegrees() {
    return 0.0;
  }

  @Override
  public void periodic() {}
}
