// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

/**
 * Represents a simulated fuel gamepiece obj
 *
 * <p>To update an object, run {@link #update()}
 *
 * <p><strong>DO NOT</strong> directly use this class. Spawn objects using {@link SimFuelManager}
 * instead
 */
public class SimFuel {
  // * CONSTANTS
  /** {@link LinearAcceleration} Constant acceleration induced by gravity */
  public static final LinearAcceleration kGravitationalAcceleration =
      MetersPerSecondPerSecond.of(9.81);

  /** {@link Double} Coefficent of fricition applied to ball velocity when on floor */
  public static final double kFloorFriction = 0.3;

  /** {@link LinearVelocity} Minimum velocity on the floor before the ball should completely stop */
  public static final LinearVelocity kFricitionVelocityThreshold = MetersPerSecond.of(0.03);

  /** {@link Distance} Radius of fuel */
  public static final Distance kRadius = Inches.of(5.91 / 2);

  /** {@link Time} Default length of lifespan */
  public static final Time kDefaultLifespanLength = Seconds.of(12.0);

  // * MEMBERS
  private boolean mIsDead = false;

  private Translation3d mDisplacementMeters;
  private Vector<N3> mVelocityMps;

  private double mLifespanStartSec;
  private double mLifespanLengthSec;

  private boolean mGrounded = false;

  /**
   * Create new {@link SimFuel}
   *
   * @param startingDisplacementMeters {@link Translation3d} Starting displacement in meters WRT
   *     origin
   * @param startingVelocityMps {@link Vector} of length {@link N3} Starting velocity in meters per
   *     second
   * @param lifespanLengthSec {@link Double} Length of Lifespan in seconds
   */
  public SimFuel(
      Translation3d startingDisplacementMeters,
      Vector<N3> startingVelocityMps,
      double lifespanLengthSec) {
    mDisplacementMeters = startingDisplacementMeters;
    mVelocityMps = startingVelocityMps;

    mLifespanLengthSec = lifespanLengthSec;
    mLifespanStartSec = Timer.getFPGATimestamp();
  }

  public void update(double dt) {
    if (!mIsDead) {
      // Calculate new Velocity
      mVelocityMps =
          VecBuilder.fill(
              mVelocityMps.get(0),
              mVelocityMps.get(1),
              mVelocityMps.get(2) - kGravitationalAcceleration.in(MetersPerSecondPerSecond) * dt);

      // Calculate new Displacement
      var nextPosition =
          new Translation3d(
              mDisplacementMeters.getX() + mVelocityMps.get(0) * dt,
              mDisplacementMeters.getY() + mVelocityMps.get(1) * dt,
              mDisplacementMeters.getZ() + mVelocityMps.get(2) * dt);

      // Handle floor collisions
      if (nextPosition.getMeasureZ().lt(kRadius)) {
        mDisplacementMeters =
            new Translation3d(
                nextPosition.getX(),
                nextPosition.getY(),
                nextPosition.getMeasureZ().lt(kRadius) ? kRadius.in(Meters) : nextPosition.getZ());

        mGrounded = true;
      } else {
        mDisplacementMeters = nextPosition;
      }

      // Handle Floor fricition
      if (mGrounded) {
        var vx = mVelocityMps.get(0);
        var vy = mVelocityMps.get(1);

        var mag = Math.hypot(vx, vy);

        if (mag > kFricitionVelocityThreshold.in(MetersPerSecond)) {
          var deaccel = kGravitationalAcceleration.times(kFloorFriction).times(dt);

          double newMag = Math.max(0.0, mag - deaccel.in(MetersPerSecondPerSecond));
          double scale = newMag / mag;

          mVelocityMps =
              VecBuilder.fill(
                  mVelocityMps.get(0) * scale, mVelocityMps.get(1) * scale, mVelocityMps.get(2));
        } else {
          mVelocityMps = VecBuilder.fill(0.0, 0.0, 0.0);
        }
      }
    }

    // Check if lifespan limit has been exceeded
    if ((mLifespanLengthSec != 0.0)
        && ((Timer.getFPGATimestamp() - mLifespanStartSec) > mLifespanLengthSec)) {
      mIsDead = true;
    }
  }

  /**
   * Check if simulated object has exceeded its lifespan
   *
   * @return {@link Boolean}
   */
  public boolean isDead() {
    return mIsDead;
  }

  /**
   * Get Position of fuel in 3d space
   *
   * @return {@link Pose3d}
   */
  public Pose3d getPose3d() {
    return new Pose3d(mDisplacementMeters, Rotation3d.kZero);
  }

  /**
   * Get Velocity of fuel in 3d space as a N3 vec
   *
   * @return {@link Vector} of size {@link N3}
   */
  public Vector<N3> getVelocityVector() {
    return mVelocityMps;
  }

  /**
   * Override object displacement
   *
   * @param displacement {@link Translation3d} Displacement to replace current displacement with
   */
  public void overrideDisplacement(Translation3d displacement) {
    mDisplacementMeters = displacement;
  }

  /**
   * Override object velocity
   *
   * @param velocityMps {@link Vector} of length {@link N3} Velocity to replace current velocity
   *     with
   */
  public void overrideVelocity(Vector<N3> velocityMps) {
    mVelocityMps = velocityMps;
  }

  /** Kill simulated object before its lifespan limit */
  public void kill() {
    mIsDead = true;
  }
}
