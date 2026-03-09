// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.TreeMap;

/**
 * Interpolating Projectile Parameters Tree maps are used to get {@link ProjectileParameters} that
 * are not defined by linearly interpolating between defined {@link ProjectileParameters} samples
 *
 * <p>Each {@link ProjectileParameters} sample has a key (Distance in meters) & an entry (the actual
 * parammeters)
 *
 * <p>To define a {@link ProjectileParameters}, use the {@link #addSammple(double,
 * ProjectileParameters)} method
 *
 * <p>To get a {@link ProjectileParameters}, use the {@link #calculateProjectileParameters(Pose2d,
 * Translation2d)} or {@link #get(double)} methods
 */
public class InterpolatingProjectileParametersTree {
  private final TreeMap<Double, ProjectileParameters> map = new TreeMap<>();

  public InterpolatingProjectileParametersTree() {}

  /**
   * Add new {@link ProjectileParameters} sample to map
   *
   * @param rMeters {@link Double} Distance from target in meters (key)
   * @param sample {@link ProjectileParameters} Sample (entry)
   */
  public void addSample(double rMeters, ProjectileParameters sample) {
    map.put(rMeters, sample);
  }

  /** Clear map completely */
  public void clear() {
    map.clear();
  }

  /**
   * Remove a sample from map
   *
   * @param rMeters {@link Double} Distance from target in meters (key)
   */
  public void remove(double rMeters) {
    map.remove(rMeters);
  }

  /**
   * Get sample with largest key in map
   *
   * @return {@link Double}
   */
  public double getMaxKeyMeters() {
    return map.lastKey();
  }

  /**
   * Calculate a {@link ProjectileParameters} sample from a specified robot position to a target
   * position
   *
   * @param robot {@link Pose2d} Robot Position on field
   * @param target {@link Translation2d} Target Position on field
   * @return {@link ProjectileParameters}
   */
  public ProjectileParameters calculateProjectileParameters(Pose2d robot, Translation2d target) {
    return get(robot.getTranslation().getDistance(target));
  }

  /**
   * Calculate a {@link ProjectileParameters} estimate from a specified distance to target
   *
   * @param rMeters {@link Double} Distance to calculate for
   * @return {@link ProjectileParameters}
   */
  public ProjectileParameters get(double rMeters) {
    ProjectileParameters parameters = map.get(rMeters);

    if (parameters == null) {
      Double ceilingKey = map.ceilingKey(rMeters);
      Double floorKey = map.floorKey(rMeters);

      if (ceilingKey == null && floorKey == null) {
        return null;
      } else if (ceilingKey == null) {
        return map.get(floorKey);
      } else if (floorKey == null) {
        return map.get(ceilingKey);
      }

      ProjectileParameters floor = map.get(floorKey);
      ProjectileParameters ceiling = map.get(ceilingKey);

      return interpolate(
          floor,
          ceiling,
          inverseInterpolate(ceilingKey.doubleValue(), rMeters, floorKey.doubleValue()));
    }

    return parameters;
  }

  // TODO doc these two functions I lowkey don't feel like it rn
  private ProjectileParameters interpolate(
      ProjectileParameters lowerParameters, ProjectileParameters higherParameters, double t) {
    return new ProjectileParameters(
        MathUtil.interpolate(
            lowerParameters.initialProjectilePitchRads(),
            higherParameters.initialProjectilePitchRads(),
            t),
        MathUtil.interpolate(
            lowerParameters.initialProjectileVelocityMps(),
            higherParameters.initialProjectileVelocityMps(),
            t),
        MathUtil.interpolate(
            lowerParameters.timeOfFlightSec(), higherParameters.timeOfFlightSec(), t));
  }

  private double inverseInterpolate(double upper, double query, double lower) {
    double upperToLower = upper - lower;
    if (upperToLower <= 0.0) {
      return 0.0;
    }

    double queryToLower = query - lower;
    if (queryToLower <= 0.0) {
      return 0.0;
    }

    return queryToLower / upperToLower;
  }
}
