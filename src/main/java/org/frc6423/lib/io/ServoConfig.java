// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * Represents a configuration for the {@link ServoIO}
 *
 * @param name {@link String} representing servo nickname
 * @param canBus {@link CANBus} representing CAN bus loop device is in
 * @param canDeviceId {@link Integer} representing the id of CAN device
 * @param talonConfig {@link TalonFXConfiguration} representing the servo config
 * @param rotationalInertia {@link MomentOfInertia} representing the rotational inertia of system
 * @param motorKt {@link Double} representing the servo's kT rating
 * @param voltageGainsSlot {@link Integer} representing the gains slot to use for voltage based
 *     control
 * @param torqueGainsSlot {@link Integer} representing the gains slot to use for torque based
 *     control
 */
public record ServoConfig(
    String name,
    CANBus canBus,
    int canDeviceId,
    TalonFXConfiguration talonConfig,
    MomentOfInertia rotationalInertia,
    double motorKt,
    int voltageGainsSlot,
    int torqueGainsSlot) {

  /**
   * @return {@link Double} representing the motor's kT after going through gearbox of system
   */
  public double systemKt() {
    return talonConfig.Feedback.SensorToMechanismRatio / motorKt();
  }
}
