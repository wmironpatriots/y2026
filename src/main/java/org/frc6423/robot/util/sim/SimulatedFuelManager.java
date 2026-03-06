// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.util.sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;

/** A class for spawning & keeping track of {@link SimulatedFuel} objects */
public class SimulatedFuelManager {
  private final ArrayList<SimulatedFuel> mFuel = new ArrayList<>();

  private final Notifier mNotifier;
  private final double mPeriodSec;

  /**
   * Create new {@link SimulatedFuelManager}
   *
   * @param periodSec {@link Double} Update period in seconds
   */
  public SimulatedFuelManager(double periodSec) {
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

        // Calculate translation between fuel pose and other fuel pose
        var pose = fuel.getPose3d();
        var otherPose = otherFuel.getPose3d();

        var translation = pose.minus(otherPose).getTranslation();
        var dist = translation.getNorm();

        if (dist < SimulatedFuel.kRadius.times(2).in(Meters) && dist > 0.001) {
          // Normalize dist vector
          double nx = translation.getX() / dist;
          double ny = translation.getY() / dist;
          double nz = translation.getZ() / dist;

          // Calculate seperation scaler
          double overlap = SimulatedFuel.kRadius.times(2).in(Meters) - dist;
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

          // Get velocities
          var vel = fuel.getVelocityVector();
          var otherVel = fuel.getVelocityVector();

          // Calculate relative velocities
          var relVel = vel.minus(otherVel);

          // If balls are moving towards eachother
          if (relVel.norm() > 0.0) {
            // Derive impluse using friction k as restitution
            double impulse = relVel.norm() * (1 + SimulatedFuel.kFloorFriction) / 2.0;

            // Override fuel vel
            fuel.overrideVelocity(
                VecBuilder.fill(
                    vel.get(0) - impulse * nx,
                    vel.get(1) - impulse * ny,
                    vel.get(2) - impulse * nz));

            // Override other fuel vel
            otherFuel.overrideVelocity(
                VecBuilder.fill(
                    otherVel.get(0) - impulse * nx,
                    otherVel.get(1) - impulse * ny,
                    otherVel.get(2) - impulse * nz));
          }
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
    return mFuel.stream().map(SimulatedFuel::getPose3d).toArray(Pose3d[]::new);
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
    return spawnFuel(
        initialDisplacementMeters, initialVelocityMps, SimulatedFuel.kDefaultLifespanLength);
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
        new SimulatedFuel(
            initialDisplacementMeters, initialVelocityMps, lifespanLength.in(Seconds)));
    return mFuel.size() - 1;
  }
}
