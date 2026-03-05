// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructSubscriber;
import org.frc6423.robot.subsystem.drive.localization.VisionConstants.CameraConfig;

public class CameraIOIronSight extends CameraIO {
  private final StructSubscriber<Pose2d> mEstimateSubscriber;
  private final DoubleSubscriber mTimestampSubscriber;

  /**
   * Create new {@link CameraIO}
   *
   * @param config {@link CameraConfig} Configuration for camera
   */
  public CameraIOIronSight(CameraConfig config) {
    super(config);

    mEstimateSubscriber =
        NetworkTableInstance.getDefault()
            .getTable("iron-sight/estimates/" + mConfig.name())
            .getStructTopic("Pose2d", Pose2d.struct)
            .subscribe(
                new Pose2d(), PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.01));

    mTimestampSubscriber =
        NetworkTableInstance.getDefault()
            .getTable("iron-sight/estimates/" + mConfig.name())
            .getDoubleTopic("TimestampSeconds")
            .subscribe(0.0, PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.01));
  }

  @Override
  public boolean isConnected() {
    // TODO
    return true;
  }

  @Override
  public VisionMeasurement[] getUnreadMeasurements() {
    // TODO get multiple when support is added
    return new VisionMeasurement[] {
      new VisionMeasurement(
          mTimestampSubscriber.get(), mEstimateSubscriber.get(), VecBuilder.fill(0.0, 0.0, 0.0))
    };
  }
}
