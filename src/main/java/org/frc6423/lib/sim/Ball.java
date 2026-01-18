// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Ball {
  public Ball(Pose3d initialPose) {}

  public Rotation3d getRotation3d() {
    return getPose3d().getRotation();
  }

  public Pose3d getPose3d() {
    return new Pose3d();
  }
}
