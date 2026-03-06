// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.frc6423.lib.util.GeometryUtil;
import org.frc6423.robot.Rebuilt;
import org.frc6423.robot.subsystem.hood.HoodConstants;

/**
 * Static system for calculating optimal {@link ProjectileParameters}
 *
 * <p>To calculate a shot, utilize {@link #getProjectileParameters(Pose2d, ChassisSpeeds)}
 *
 * <p>To log calculations, you must create an instance of this class
 */
public class FireControlSystem {
  /** Constants for the {@link FireControlSystem} */
  public class Constants {
    /** {@link Double} The estimated average latency to compensate for */
    public static final double kLatencySeconds = 0.03;

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
          Units.inchesToMeters(24 + 17),
          new ProjectileParameters(HoodConstants.kMinAngle.in(Radians), 9.43194079202, 1.04));

      kShotTree.addSample(
          Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 12),
          new ProjectileParameters(Units.degreesToRadians(25), 8.25294819302, 1.14));

      kShotTree.addSample(
          Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 3 * 12),
          new ProjectileParameters(Units.degreesToRadians(26), 8.72454523262, 1.10));

      kShotTree.addSample(
          Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 5 * 12),
          new ProjectileParameters(Units.degreesToRadians(30), 8.72454523262, 1.09));

      kShotTree.addSample(
          Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 7 * 12),
          new ProjectileParameters(Units.degreesToRadians(33), 8.72454523262, 1.15));

      kShotTree.addSample(
          Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 9 * 12),
          new ProjectileParameters(Units.degreesToRadians(36), 8.96034375242, 1.23));

      kShotTree.addSample(
          Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 11 * 12),
          new ProjectileParameters(Units.degreesToRadians(38), 8.96034375242, 1.33));
      kShotTree.addSample(
          Units.inchesToMeters(24 * Math.sqrt(2) + 6 + 13 * 12),
          new ProjectileParameters(Units.degreesToRadians(39), 8.96034375242, 1.35));
    } // TODO calculate & add shots
  }

  // * SUPPLIERS
  private static Pose2d mCurrentTarget = Rebuilt.kMidPose;
  private static Twist3d mCurrentTrajectory = new Twist3d();

  /** Initialize logging */
  public FireControlSystem() {}

  /**
   * Get the target position system is currently locked onto
   *
   * @return {@link Pose2d}
   */
  @Logged(name = "Current Target", importance = Importance.INFO)
  public Pose2d getCurrentTarget() {
    return mCurrentTarget;
  }

  @Logged(name = "Current Trajectory", importance = Importance.INFO)
  public Twist3d getTrajectory() {
    return mCurrentTrajectory;
  }

  public static Rotation2d getRotation2d(
      Pose2d estimatedPosition, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate the target position
    var targetPose = // TODO account for different targets
        GeometryUtil.applyForAlliance(Rebuilt.kMidPose, Rebuilt.kHubPose2d);

    double tof =
        Constants.kShotTree
            .calculateProjectileParameters(estimatedPosition, targetPose.getTranslation())
            .timeOfFlightSec();

    Translation2d vtarget = getVirtualTarget(targetPose.getTranslation(), fieldRelativeSpeeds, tof);

    Translation2d robotToTarget = vtarget.minus(estimatedPosition.getTranslation());
    Rotation2d rotation =
        Rotation2d.fromRadians(Math.atan2(robotToTarget.getX(), robotToTarget.getY()));

    return rotation;
  }

  /**
   * Get the optimal {@ProjectileParameters} for the current target of the {@link FireControlSystem}
   *
   * @param estimatedPosition {@link Pose2d} Estimated position of robot
   * @param fieldRelativeSpeeds {@link ChassisSpeeds} Estimated velocity of robot
   * @return {@link ProjectileParameters}
   */
  public static ProjectileParameters getProjectileParameters(
      Pose2d estimatedPosition, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate predicted position using approximated latency
    var robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, estimatedPosition.getRotation());

    var predictedPosition =
        estimatedPosition.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * Constants.kLatencySeconds,
                robotRelativeSpeeds.vyMetersPerSecond * Constants.kLatencySeconds,
                robotRelativeSpeeds.omegaRadiansPerSecond * Constants.kLatencySeconds));

    // Calculate the target position
    var targetPose = // TODO account for different targets
        GeometryUtil.applyForAlliance(Rebuilt.kMidPose, Rebuilt.kHubPose2d);

    // Calculate unadjusted shot
    ProjectileParameters parameters =
        Constants.kShotTree.calculateProjectileParameters(
            predictedPosition, targetPose.getTranslation());

    // Caclulate virtual target from field relative speeds & tof
    Translation2d virtualTarget =
        getVirtualTarget(
            targetPose.getTranslation(), fieldRelativeSpeeds, parameters.timeOfFlightSec());

    // Set target publisher
    mCurrentTarget = new Pose2d(virtualTarget, Rotation2d.kZero);

    // Calculate adjust shot for sotm
    parameters =
        Constants.kShotTree.get(predictedPosition.getTranslation().getDistance(virtualTarget));

    return parameters;
  }

  /**
   * Calculate a 'virtual target' (a target compensating for the projectile launchers velocity)
   * based on robot position/velocity
   *
   * @param target {@link Translation2d} Target to aim at
   * @param fieldRelativeSpeeds {@link ChassisSpeeds} Current velocity of robot
   * @param timeOfFlightSec {@link Double} Projectile time of flight in seconds
   * @return {@link Translation2d}
   */
  private static Translation2d getVirtualTarget(
      Translation2d target, ChassisSpeeds fieldRelativeSpeeds, double timeOfFlightSec) {
    // Calculate where the robot will be
    Translation2d virtual =
        target.minus(
            new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlightSec,
                fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlightSec));

    return virtual;
  }
}
