// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.localization;

import org.frc6423.robot.subsystem.drive.localization.VisionConstants.CameraConfig;

public class CameraIONone extends CameraIO {
  public CameraIONone(CameraConfig config) {
    super(config);
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public VisionMeasurement[] getUnreadMeasurements() {
    return new VisionMeasurement[] {};
  }
}
