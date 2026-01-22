// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import org.frc6423.lib.io.IO;
import org.frc6423.robot.RobotState.VisionMeasurement;

/** An {@link IO} for controlling a camera */
public abstract class CameraIO extends IO {

  /**
   * @return {@link VisionMeasurement} array representing all unread vision position estimates
   */
  public abstract VisionMeasurement[] getUnreadVisionMeasurements();
}
