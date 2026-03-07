// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.frc6423.robot.subsystem.vision.Vision.CameraConfig;

/** An abstract hardware interface for interacting with camera hardware */
public abstract class CameraIO {
  public final CameraConfig mConfig;

  /**
   * Create new {@link CameraIO}
   *
   * @param config {@link CameraConfig} Hardware Configuration for camera
   */
  public CameraIO(CameraConfig config) {
    mConfig = config;
  }

  /**
   * Get nickname of camera
   *
   * @return {@link String}
   */
  public String getName() {
    return mConfig.name();
  }

  /**
   * Get status of camera
   *
   * @return {@link Boolean}
   */
  public abstract boolean isConnected();

  /**
   * Get array of unread {@link VisionMeasurement}
   *
   * @return {@link Array} of {@link VisionMeasurement}
   */
  public abstract VisionMeasurement[] getUnreadMeasurements();

  /**
   * An estimation of the robots position with a variance
   *
   * @param timestampSeconds {@link Double} FPAG timestamp of when the camera estimates the robot
   *     was in this position
   * @param positionEstimate {@link Pose2d} Estimated position derived by camera
   * @param stDevs {@link Matrix} of Length {@link N3} Standard Deviations of measurement in the x,
   *     y, & z directions
   */
  public record VisionMeasurement(
      double timestampSeconds, Pose2d positionEstimate, Matrix<N3, N1> stDevs) {}
}
