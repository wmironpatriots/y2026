// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc6423.robot.subsystem.drive.constant.SwerveConstants.GyroConfig;

/** {@link GyroIO} extension for interacting with a {@link Pigeon2} */
public class GyroIOPigeon2 extends GyroIO {
  private final Pigeon2 mPigeon;

  private final BaseStatusSignal mYawSignal;

  public GyroIOPigeon2(GyroConfig config) {
    mPigeon = new Pigeon2(config.deviceId(), config.canBus());
    mPigeon.getConfigurator().apply(config.config());
    mPigeon.getConfigurator().setYaw(0.0);

    mYawSignal = mPigeon.getYaw();
    mYawSignal.setUpdateFrequency(50.0);

    ParentDevice.optimizeBusUtilizationForAll(mPigeon);
  }

  @Override
  public Rotation2d getYawRotation2d() {
    return mPigeon.getRotation2d();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(mYawSignal);
  }
}
