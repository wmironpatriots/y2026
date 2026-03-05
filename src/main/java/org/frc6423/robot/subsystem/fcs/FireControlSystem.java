// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class FireControlSystem {
  private static final double kLatencySeconds = 0.1; // TODO tune
  private static final Transform3d kRobotToShooter = new Transform3d();

  private final Supplier<Pose2d> mEstimatedPositionSupplier;
  private final Supplier<ChassisSpeeds> mFieldRelativeSpeedsSupplier;

  /**
   * Create new {@link FireControlSystem}
   *
   * @param estimatedPositionSupplier {@link Supplier} of {@link Pose2d} Stream of estiamted field
   *     positions in 2-Dimensional Space (x, y)
   * @param fieldRelativeSpeedsSupplier {@link Supplier} of {@link ChassisSpeeds} Stream of measured
   *     field relative velocities
   */
  public FireControlSystem(
      Supplier<Pose2d> estimatedPositionSupplier,
      Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier) {
    mEstimatedPositionSupplier = estimatedPositionSupplier;
    mFieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
  }
}
