// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import org.frc6423.lib.sim.SimulatedFuel;
import org.frc6423.lib.sim.SimulatedFuelManager;
import org.frc6423.robot.Rebuilt;

/** Utility methods for configuring simulation for Rebuilt */
public class RebuiltSimUtils {
  /**
   * Spawns the central mass of neutral zone fuel
   *
   * @param manager {@link SimulatedFuelManager} The simulation to use
   */
  public static void spawnNeutralZone(SimulatedFuelManager manager) {
    int r =
        (int) Rebuilt.kNeutralMassLength.div(SimulatedFuel.kRadius.times(2)).baseUnitMagnitude();
    int c = (int) Rebuilt.kNeutralMassWidth.div(SimulatedFuel.kRadius.times(2)).baseUnitMagnitude();

    Distance rLength = Rebuilt.kNeutralMassLength.div(r);
    Distance cLength = Rebuilt.kNeutralMassWidth.div(c);

    var start =
        new Translation2d(
            Rebuilt.kMidPose.getX() - Rebuilt.kNeutralMassLength.div(2).in(Meters),
            Rebuilt.kMidPose.getY() - Rebuilt.kNeutralMassWidth.div(2).in(Meters));

    for (int i = 0; i < r; i++) {
      for (int j = 0; j < c; j++) {
        manager.spawnFuel(
            new Translation3d(
                start.getX() + rLength.times(i).in(Meters),
                start.getY() + cLength.times(j).in(Meters),
                0.0),
            VecBuilder.fill(0.0, 0.0, 0.0),
            Seconds.of(0.0));
      }
    }
  }
}
