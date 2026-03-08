// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import org.frc6423.robot.subsystem.RobotState.VisionEstimate;
import org.frc6423.robot.subsystem.vision.Vision.CameraConfig;

public abstract class CameraIO {
  public final CameraConfig mConfig;

  public CameraIO(CameraConfig config) {
    mConfig = config;
  }

  public String getName() {
    return mConfig.name();
  }

  public abstract boolean isConnected();

  public abstract VisionEstimate[] getUnreadMeasurements();
}
