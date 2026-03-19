// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.energy;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/** Static watcher/logger of energy usage accross subsystems */
public class EnergyWatcher {
  /** List of current users accross subsystems */
  private final Map<String, Supplier<Double>> mCurrentUsers = new HashMap<>();

  private double mUsedCurrentAmps = 0.0;
  private double mUsedPowerWatts = 0.0;
  private double mUsedEnergyJoules = 0.0;

  public EnergyWatcher() {
    registerCurrentUser("Controls/RoboRIO", () -> RobotController.getInputCurrent());
    registerCurrentUser("Controls/CANcoders", () -> 0.05 * 4.0);
    registerCurrentUser("Controls/Pigeon2", () -> 0.04);
    registerCurrentUser("Controls/CANivore DRIVE", () -> 0.03);
    registerCurrentUser("Controls/CANivore SOUP", () -> 0.03);
    registerCurrentUser("Controls/Radio", () -> 0.5);
  }

  @Logged(name = "Battery Voltage (volts)", importance = Importance.CRITICAL)
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  @Logged(name = "Current Usage (ampheres)", importance = Importance.CRITICAL)
  public double getCurrentUsageAmps() {
    return mUsedCurrentAmps;
  }

  @Logged(name = "Power Usage (watts)", importance = Importance.CRITICAL)
  public double getPowerUsageWatts() {
    return mUsedPowerWatts;
  }

  @Logged(name = "Energy Usage (joulse)", importance = Importance.CRITICAL)
  public double getUsedEnergyJoules() {
    return mUsedEnergyJoules;
  }

  /**
   * Register a new current user
   *
   * @param name {@link String} Name of user
   * @param currentMeasurer {@link Supplier} of {@link Double} Stream of user supply current
   */
  public void registerCurrentUser(String name, Supplier<Double> currentMeasurer) {
    mCurrentUsers.put(name, currentMeasurer);
  }

  public void calculateUsage() {
    mUsedPowerWatts = 0.0;
    mUsedCurrentAmps = 0.0;

    for (var user : mCurrentUsers.entrySet()) {
      var amps = Math.abs(user.getValue().get());

      var power = amps * getBatteryVoltage();
      var energy = amps * 0.02;

      mUsedCurrentAmps += amps;
      mUsedPowerWatts += power;
      mUsedEnergyJoules += energy;
    }
  }
}
