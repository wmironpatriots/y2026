// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.localization;

import edu.wpi.first.math.geometry.Transform3d;

/** Constants for {@link Vision} */
public class VisionConstants {
  /** {@link Array} of {@link CameraConfig} Configurations of all cameras on robot */
  public static final CameraConfig[] kCameraConfigs =
      new CameraConfig[] {
        new CameraConfig(
            "bessie", Transform3d.kZero), // TODO leave transform as zero for now until we fix api
        new CameraConfig("elsie", Transform3d.kZero)
      };

  /**
   * A configuration for a camera
   *
   * @param name {@link String} Unique "nickname" for identifiying the camera
   * @param displacementWrtRobot {@link Transform3d} Displacement of camera with reference to the
   *     center of robot
   */
  public record CameraConfig(String name, Transform3d displacementWrtRobot) {}
}
