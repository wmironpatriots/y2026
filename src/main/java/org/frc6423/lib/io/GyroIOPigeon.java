// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

/**
 * Implementation of {@link GyroIO} for Pigeon2
 *
 * @see https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/pigeon2/index.html
 */
public class GyroIOPigeon extends GyroIO {
  protected final Pigeon2 pigeon;
  protected final Pigeon2Configuration config;

  protected final StatusSignal<Angle> pitchSignal, rollSignal, yawSignal;

  public GyroIOPigeon(int canDeviceId, CANBus canbus, Pigeon2Configuration config) {
    this.pigeon = new Pigeon2(canDeviceId, canbus);
    this.config = config;

    this.pigeon.getConfigurator().apply(config);

    pitchSignal = pigeon.getPitch();
    rollSignal = pigeon.getRoll();
    yawSignal = pigeon.getYaw();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(pitchSignal, rollSignal, yawSignal);
  }

  @Override
  public Rotation3d getRotation3d() {
    return new Rotation3d(rollSignal.getValue(), pitchSignal.getValue(), yawSignal.getValue());
  }
}
