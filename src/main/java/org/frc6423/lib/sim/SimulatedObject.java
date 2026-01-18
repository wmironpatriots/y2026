// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;

/** A template class for a simulated object */
public abstract class SimulatedObject {
  /** {@link Vector} representing the directional acceleration of gravity */
  public static final Vector<N3> kGravityNewtonsPerKg = VecBuilder.fill(0, 0, 9.8);

  private static ArrayList<SimulatedObject> objects;

  /** Tick all register {@link SimulatedObject} */
  public static void updateObjects() {
    for (var object : objects) {
      object.tick();
    }
  }

  /**
   * Register new {@link SimulatedObject} to the static {@link SimulatedObject} array
   *
   * @param object {@link SimulatedObject} representing object to register
   */
  private static void registerObject(SimulatedObject object) {
    objects.add(object);
  }

  private Vector<N3> newtonsPerKg = VecBuilder.fill(0, 0, 0);
  private Vector<N3> metersPerSec = VecBuilder.fill(0, 0, 0);
  private Vector<N3> meters = VecBuilder.fill(0, 0, 0);
  private Rotation3d rotation3d = Rotation3d.kZero;

  /** Create new {@link SimulatedObject} */
  public SimulatedObject() {
    registerObject(this);
  }

  /** Run a simulation tick */
  public void tick() {
    metersPerSec.plus(newtonsPerKg);
    meters.plus(metersPerSec);
  }

  /**
   * @return {@link Vector} representing the default net directional acceleration applied on object
   */
  protected abstract Vector<N3> getDefaultAcceleration();

  public Pose3d getPose3d() {
    return new Pose3d(meters.get(0), meters.get(1), meters.get(2), rotation3d);
  }

  public void applyAcceleration(Vector<N3> newtonsPerKg) {
    this.newtonsPerKg.plus(newtonsPerKg);
  }
}
