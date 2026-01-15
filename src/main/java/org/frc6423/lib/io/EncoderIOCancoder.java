// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/**
 * Implementation of {@link EncoderIO} for CANcoder
 *
 * @see https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html
 */
public class EncoderIOCancoder extends EncoderIO {
  protected final CANcoder cancoder;
  protected final CANcoderConfiguration config;

  protected final StatusSignal<Angle> absPositionSignal;

  public EncoderIOCancoder(int canDeviceId, CANBus canbus, CANcoderConfiguration config) {
    this.cancoder = new CANcoder(canDeviceId, canbus);
    this.config = config;

    cancoder.getConfigurator().apply(config);

    absPositionSignal = cancoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    absPositionSignal.refresh();
  }

  @Override
  public Rotation2d getRotation2d() {
    return new Rotation2d(absPositionSignal.getValue());
  }
}
