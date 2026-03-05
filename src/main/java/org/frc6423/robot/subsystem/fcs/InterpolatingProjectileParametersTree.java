// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.TreeMap;

// TODO doc
public class InterpolatingProjectileParametersTree {
  private final TreeMap<Distance, ProjectileParameters> map = new TreeMap<>();

  public InterpolatingProjectileParametersTree() {}

  public void addSample(Distance r, ProjectileParameters sample) {
    map.put(r, sample);
  }

  public void clear() {
    map.clear();
  }

  public void remove(Distance r) {
    map.remove(r);
  }

  public Distance getMaxKey() {
    return map.lastKey();
  }

  public ProjectileParameters get(Distance r) {
    ProjectileParameters parameters = map.get(r);

    if (parameters == null) {
      Distance ceilingKey = map.ceilingKey(r);
      Distance floorKey = map.floorKey(r);

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
          inverseInterpolate(ceilingKey.in(Meters), r.in(Meters), floorKey.in(Meters)));
    }

    return parameters;
  }

  private ProjectileParameters interpolate(
      ProjectileParameters lowerParameters, ProjectileParameters higherParameters, double t) {
    return new ProjectileParameters(
        Rotation2d.fromRadians(
            MathUtil.interpolate(
                lowerParameters.initialProjectileAngle().getRadians(),
                higherParameters.initialProjectileAngle().getRadians(),
                t)),
        MetersPerSecond.of(
            MathUtil.interpolate(
                lowerParameters.initialProjectileVelocity().in(MetersPerSecond),
                higherParameters.initialProjectileVelocity().in(MetersPerSecond),
                t)),
        MathUtil.interpolate(lowerParameters.timeOfFlight(), higherParameters.timeOfFlight(), t));
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
