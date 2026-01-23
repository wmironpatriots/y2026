// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.RobotState.VisionMeasurement;

public class Vision extends SubsystemBase {
  private final CameraIO[] cameras;

  public Vision(CameraIO... cameras) {
    this.cameras = cameras;
  }

  @Override
  public void periodic() {
    ArrayList<VisionMeasurement> poseEstimates = new ArrayList<>();
    for (CameraIO camera : cameras) {
      Arrays.stream(camera.getUnreadVisionMeasurements()).forEach(p -> poseEstimates.add(p));
    }

    RobotState.getInstace().addVisionMeasurement(poseEstimates.toArray(VisionMeasurement[]::new));
  }
}
