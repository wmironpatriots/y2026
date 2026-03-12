// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Array;
import java.util.ArrayList;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.RobotState.VisionEstimate;

/** {@link SubsystemBase} Manager subsystem for camera hardware */
public class Vision extends SubsystemBase {
  /**
   * Create new {@link Vision}
   *
   * @return {@link Vision}
   */
  public static Vision create() {
    var list = new ArrayList<>();

    for (var config : kCameraConfigs) {
      list.add(new CameraIOIronSight(config));
    }

    return new Vision(list.toArray(new CameraIO[] {}));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link Array} of {@link CameraConfig} Configurations of all cameras on robot */
  public static final CameraConfig[] kCameraConfigs =
      new CameraConfig[] {
        new CameraConfig(
            "bessie",
            new Transform3d(new Translation3d(Units.inchesToMeters(12.255), Units.inchesToMeters(0.0), Units.inchesToMeters(-14.207)), new Rotation3d(0.0, -0.523599, 0.0))),
        new CameraConfig(
            "elsie",
            new Transform3d(new Translation3d(Units.inchesToMeters(-7.51), Units.inchesToMeters(-1.5), Units.inchesToMeters(-20.1)), new Rotation3d(0.0, 0.523599, 0.0))),
        new CameraConfig(
            "beatrice",
            new Transform3d(new Translation3d(Units.inchesToMeters(-7.897), Units.inchesToMeters(12.259), Units.inchesToMeters(-20.056)), new Rotation3d(0.0, 0.261799, 0.261799))),
        new CameraConfig(
            "belinda",
            new Transform3d(new Translation3d(Units.inchesToMeters(-7.897), Units.inchesToMeters(-12.259), Units.inchesToMeters(-20.056)), new Rotation3d(0.0, 0.261799, -0.261799))),
      };

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private final CameraIO[] mCameras;

  protected Vision(CameraIO... cameras) {
    mCameras = cameras;
  }

  @Override
  public void periodic() {
    var estimates = getUnreadMeasurements();
    for (var estimate : estimates) {
      RobotState.getInstance().addVisionEstimate(estimate);
    }
  }

  public VisionEstimate[] getUnreadMeasurements() {
    ArrayList<VisionEstimate> allMeasurements = new ArrayList<>();
    for (var camera : mCameras) {
      var measurements = camera.getUnreadMeasurements();

      for (var measurement : measurements) {
        allMeasurements.add(measurement);
      }
    }

    return allMeasurements.toArray(new VisionEstimate[0]);
  }

  public record CameraConfig(String name, Transform3d displacementWrtRobot) {}
}
