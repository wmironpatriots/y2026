// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.energy;

import java.util.ArrayList;
import java.util.function.Supplier;

/** Static watcher/logger of energy usage accross subsystems */
public class EnergyWatcher {
  /** List of current users accross subsystems */
  private static final ArrayList<Supplier<Double>> kCurrentReaders = new ArrayList<>();

  public static void registerCurrentUser(Supplier<Double> currentReader) {
    kCurrentReaders.add(currentReader);
  }
}
