// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import org.frc6423.lib.util.GeometryUtil;
import org.frc6423.robot.Rebuilt;
import org.frc6423.robot.subsystem.hood.Hood;

/** System to calculating {@link ProjectileParameters} */
public class FireControlSystem {
  // * CONSTANTS
  /** {@link Double} The estimated average latency to compensate for */
  public static final double kLatencySeconds = 0.03; // TODO tune

  /** {@link Transform3d} Displacement from center of chassis to projectile exit point */
  public static final Transform3d kRobotToShooter = new Transform3d();

  /**
   * {@link InterpolatingProjectileParametersTree} Interpolating Tree for calculating projectile
   * parameters
   */
  public static final InterpolatingProjectileParametersTree kShotTree =
      new InterpolatingProjectileParametersTree();

  static {
    kShotTree.addSample(
        Inches.of(24 + 17),
        new ProjectileParameters(
            new Rotation2d(Hood.Constants.kMinAngle), MetersPerSecond.of(40), 1.04));

    kShotTree.addSample(
        Inches.of(24 * Math.sqrt(2) + 6 + 12),
        new ProjectileParameters(Rotation2d.fromDegrees(25), MetersPerSecond.of(35), 1.14));

    kShotTree.addSample(
        Inches.of(24 * Math.sqrt(2) + 6 + 3 * 12),
        new ProjectileParameters(Rotation2d.fromDegrees(26), MetersPerSecond.of(37), 1.10));

    kShotTree.addSample(
        Inches.of(24 * Math.sqrt(2) + 6 + 5 * 12),
        new ProjectileParameters(Rotation2d.fromDegrees(30), MetersPerSecond.of(37), 1.09));

    kShotTree.addSample(
        Inches.of(24 * Math.sqrt(2) + 6 + 7 * 12),
        new ProjectileParameters(Rotation2d.fromDegrees(33), MetersPerSecond.of(37), 1.15));

    kShotTree.addSample(
        Inches.of(24 * Math.sqrt(2) + 6 + 9 * 12),
        new ProjectileParameters(Rotation2d.fromDegrees(36), MetersPerSecond.of(38), 1.23));

    kShotTree.addSample(
        Inches.of(24 * Math.sqrt(2) + 6 + 11 * 12),
        new ProjectileParameters(Rotation2d.fromDegrees(38), MetersPerSecond.of(38), 1.33));
    kShotTree.addSample(
        Inches.of(24 * Math.sqrt(2) + 6 + 13 * 12),
        new ProjectileParameters(Rotation2d.fromDegrees(39), MetersPerSecond.of(38), 1.35));
  } // TODO calculate & add shots

  // * SUPPLIERS
  private final Supplier<Pose2d> mEstimatedPositionSupplier;
  private final Supplier<ChassisSpeeds> mFieldRelativeSpeedsSupplier;

  private Pose2d mCurrentTarget = Rebuilt.kMidPose;

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

  /**
   * Get the current target system is locked onto
   *
   * @return {@link Pose2d}
   */
  @Logged(name = "Current Target", importance = Importance.INFO)
  public Pose2d getCurrentTarget() {
    return mCurrentTarget;
  }

  // TODO
  public void getChassisYaw() {
    // // Get current position/velocity
    // var estimatedPosition = mEstimatedPositionSupplier.get();
    // var fieldRelativeSpeeds = mFieldRelativeSpeedsSupplier.get();

    // var setpointYaw =
  }

  public ProjectileParameters getProjectileParameters() {
    // Get current position/velocity
    var estimatedPosition = mEstimatedPositionSupplier.get();
    var fieldRelativeSpeeds = mFieldRelativeSpeedsSupplier.get();

    var robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, estimatedPosition.getRotation());

    // Calculate predicted position using approximated latency
    var predictedPosition =
        estimatedPosition.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * kLatencySeconds,
                robotRelativeSpeeds.vyMetersPerSecond * kLatencySeconds,
                robotRelativeSpeeds.omegaRadiansPerSecond * kLatencySeconds));

    // Calculate the target position
    var targetPose = // TODO account for ferrying
        GeometryUtil.applyForAlliance(Rebuilt.kMidPose, Rebuilt.kHubPose2d);

    return getProjectileParameters(
        predictedPosition, targetPose.getTranslation(), fieldRelativeSpeeds);
  }

  private ProjectileParameters getProjectileParameters(
      Pose2d robotPosition, Translation2d targetTranslation, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate unadjusted shot
    ProjectileParameters parameters = kShotTree.calculateShot(robotPosition, targetTranslation);

    // Caclulate virtual target from field relative speeds & tof
    Translation2d virtualTarget =
        getVirtualTarget(targetTranslation, fieldRelativeSpeeds, parameters.timeOfFlight());

    // Set target publisher
    mCurrentTarget = new Pose2d(virtualTarget, Rotation2d.kZero);

    return kShotTree.get(Meters.of(robotPosition.getTranslation().getDistance(virtualTarget)));
  }

  private Translation2d getVirtualTarget(
      Translation2d target, ChassisSpeeds fieldRelativeSpeeds, double timeOfFlight) {
    // Calculate where the robot will be
    Translation2d virtual =
        target.minus(
            new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlight,
                fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlight));

    return virtual;
  }
}
