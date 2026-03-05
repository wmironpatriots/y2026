// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.localization.VisionConstants.CameraConfig;

public class Vision extends SubsystemBase {
  public static Vision create() {
    CameraConfig[] configs = VisionConstants.kCameraConfigs;
    CameraIO[] ios = new CameraIO[configs.length];

    for (int i = 0; i < configs.length; i++) {
      ios[i] = new CameraIOIronSight(configs[i]);
    }

    return (Robot.isReal()) ? new Vision(ios) : new Vision(new CameraIONone(configs[0]));
  }

  private final CameraIO[] mCameras;

  /**
   * Create new {@link Vision}
   *
   * @param cameras {@link Array} of {@link CameraIO} Cameras to initialize
   */
  public Vision(CameraIO... cameras) {
    mCameras = cameras;
  }
}
