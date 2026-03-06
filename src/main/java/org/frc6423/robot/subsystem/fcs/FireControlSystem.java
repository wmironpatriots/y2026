// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc6423.lib.util.GeometryUtil;
import org.frc6423.robot.Rebuilt;

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
    } // TODO calculate & add shots
  }

  // * SUPPLIERS
  private static Pose2d mCurrentTarget = Rebuilt.kMidPose;

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
    return Constants.kShotTree.get(predictedPosition.getTranslation().getDistance(virtualTarget));
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
