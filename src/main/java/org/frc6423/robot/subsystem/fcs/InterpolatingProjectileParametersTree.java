// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.TreeMap;

// TODO doc
public class InterpolatingProjectileParametersTree {
  private final TreeMap<Double, ProjectileParameters> map = new TreeMap<>();

  public InterpolatingProjectileParametersTree() {}

  public void addSample(double rMeters, ProjectileParameters sample) {
    map.put(rMeters, sample);
  }

  public void clear() {
    map.clear();
  }

  public void remove(double rMeters) {
    map.remove(rMeters);
  }

  public double getMaxKeyMeters() {
    return map.lastKey();
  }

  public ProjectileParameters calculateShot(Pose2d robot, Translation2d target) {
    return get(robot.getTranslation().getDistance(target));
  }

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
