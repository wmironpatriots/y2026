// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Interface for interacting with a localization camera */
public abstract class CameraIO {
  public final String mName;

  /**
   * Create new {@link CameraIO}
   *
   * @param name {@link String} An identifier for camera
   */
  public CameraIO(String name) {
    mName = name;
  }

  /**
   * Get the identifier of camera
   *
   * @return {@link String}
   */
  public String getName() {
    return mName;
  }

  /**
   * Check status of camera connection
   *
   * @return {@link Boolean}
   */
  public abstract boolean isConnected();

  /**
   * Get all new camera position measurements
   *
   * @return {@Array} of {@link VisionMeasurements}
   */
  public abstract VisionMeasurement[] getUnreadMeasurements();

  /**
   * Represents a camera estimated position
   *
   * @param timestampSeconds {@link Double} When the camera thinks robot was at estimated position
   *     (in seconds)
   * @param positionEstimate {@link Pose2d} Estimated position in 2-Dimensional Space
   * @param stDevs {@link Matrix} of length {@link N3} Standard deviations of the vision pose
   *     measurement (x position in meters, y position in meters, and heading in radians).
   */
  public record VisionMeasurement(
      double timestampSeconds, Pose2d positionEstimate, Matrix<N3, N1> stDevs) {}
}
