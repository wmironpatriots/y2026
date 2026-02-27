// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;

// TODO
public class GyroIOPigeon2 extends GyroIO {
  private final Pigeon2 mPigeon;

  public GyroIOPigeon2(int canDeviceId, CANBus canBus, Pigeon2Configuration config) {
    mPigeon = new Pigeon2(canDeviceId, canBus);
    mPigeon.getConfigurator().apply(config);
  }

  @Override
  public Rotation3d getRotation3d() {
    return mPigeon.getRotation3d();
  }
}
