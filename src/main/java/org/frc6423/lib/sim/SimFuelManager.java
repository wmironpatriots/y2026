// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;

/** A class for spawning & keeping track of {@link SimFuel} objects */
public class SimFuelManager {
  private final ArrayList<SimFuel> mFuel = new ArrayList<>();

  private final Notifier mNotifier;
  private final double mPeriodSec;

  /**
   * Create new {@link SimFuelManager}
   *
   * @param periodSec {@link Double} Update period in seconds
   */
  public SimFuelManager(double periodSec) {
    mNotifier = new Notifier(() -> update());

    mNotifier.startPeriodic(periodSec);
    mPeriodSec = periodSec;
  }

  /** Update all sim fuel */
  private void update() {
    for (int i = 0; i < mFuel.size(); i++) {
      var fuel = mFuel.get(i);

      // Update fuel
      fuel.update(mPeriodSec);

      // Handle Collisions (this feels very very stupid)
      for (int j = 0; j < mFuel.size(); j++) {
        var otherFuel = mFuel.get(j);

        var pose = fuel.getPose3d();
        var otherPose = otherFuel.getPose3d();
        // Calculate translation between fuel pose and other fuel pose
        var translation = pose.minus(otherPose).getTranslation();

        // Calculate distance
        double xDist = translation.getX();
        double yDist = translation.getY();
        double zDist = translation.getZ();
        double dist = Math.sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));

        if (dist < SimFuel.kRadius.times(2).in(Meters) && dist > 0.001) {
          // Normalize dist vector
          double nx = xDist / dist;
          double ny = yDist / dist;
          double nz = zDist / dist;

          // Calculate seperation scaler
          double overlap = SimFuel.kRadius.times(2).in(Meters) - dist;
          double seperation = overlap / 2.0 + 0.001;

          // Override displacements
          fuel.overrideDisplacement(
              new Translation3d(
                  pose.getX() + nx * seperation,
                  pose.getY() + ny * seperation,
                  pose.getZ() + nz * seperation));

          otherFuel.overrideDisplacement(
              new Translation3d(
                  otherPose.getX() - nx * seperation,
                  otherPose.getY() - ny * seperation,
                  otherPose.getZ() - nz * seperation));
        }
      }

      // Despawn fuel if dead
      if (fuel.isDead()) mFuel.remove(i);
    }
  }

  /**
   * Get fuel positions in 3-Dimensional space
   *
   * @return {@link Pose3d} array
   */
  @Logged(name = "Sim Fuel Poses", importance = Importance.INFO)
  public Pose3d[] getFuelPose3d() {
    return mFuel.stream().map(SimFuel::getPose3d).toArray(Pose3d[]::new);
  }

  /**
   * Spawn new fuel with an initial displacement & velocity, then get its ID
   *
   * @param initialDisplacementMeters {@link Translation3d} Initial displacement from origin in
   *     meters
   * @param initialVelocityMps {@link Vector} of length {@link N3} Initial velocity in meters per
   *     second
   * @return {@link Integer}
   */
  public int spawnFuel(Translation3d initialDisplacementMeters, Vector<N3> initialVelocityMps) {
    return spawnFuel(initialDisplacementMeters, initialVelocityMps, SimFuel.kDefaultLifespanLength);
  }

  /**
   * Spawn new fuel with an initial displacement, velocity, & lifespan length, then get its ID
   *
   * @param initialDisplacementMeters {@link Translation3d} Initial displacement from origin in
   *     meters
   * @param initialVelocityMps {@link Vector} of length {@link N3} Initial velocity in meters per
   *     second
   * @param lifespanLength {@link Time} Length of Lifespan
   * @return {@link Integer}
   */
  public int spawnFuel(
      Translation3d initialDisplacementMeters, Vector<N3> initialVelocityMps, Time lifespanLength) {
    mFuel.add(
        new SimFuel(initialDisplacementMeters, initialVelocityMps, lifespanLength.in(Seconds)));
    return mFuel.size() - 1;
  }
}
