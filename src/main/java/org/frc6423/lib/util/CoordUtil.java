// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Utility class for interacting with classes related to the Field Coordinate System
 *
 * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
 */
public class CoordUtil {
  /**
   * Convert specified {@link Pose2d} to its opposing alliance equivalent
   *
   * @param fieldMidpoint {@link Pose2d} representing the midpoint of the field
   * @param pose {@link Pose2d} to convert
   * @return {@link Pose2d}
   */
  public static Pose2d allianceFlipPose2d(Pose2d fieldMidpoint, Pose2d pose) {
    return pose.rotateAround(fieldMidpoint.getTranslation(), Rotation2d.k180deg);
  }

  /**
   * Mirror specified {@link Pose2d} over the midline of the y axis
   *
   * @param fieldMidpoint {@link Pose2d} representing the midpoint of the field
   * @param pose {@link Pose2d} to mirror
   * @return {@link Pose2d}
   */
  public static Pose2d mirrorPose2d(Pose2d fieldMidpoint, Pose2d pose) {
    Rotation2d angle = fieldMidpoint.relativeTo(pose).getTranslation().getAngle();

    // I think this should work?
    return pose.rotateAround(fieldMidpoint.getTranslation(), angle.times(2).times(-1));
  }
}
