// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.LinearAcceleration;

/** Interface for interacting with {@link Pigeon2} gyro hardware */
public class GyroIOPigeon2 extends GyroIO {
  private final Pigeon2 mPigeon;

  private final StatusSignal<LinearAcceleration> mAccelXSig, mAccelYSig, mAccelZSig;

  public GyroIOPigeon2(int canDeviceId, CANBus canBus, Pigeon2Configuration config) {
    mPigeon = new Pigeon2(canDeviceId, canBus);
    mPigeon.getConfigurator().apply(config);

    mAccelXSig = mPigeon.getAccelerationX();
    mAccelYSig = mPigeon.getAccelerationY();
    mAccelZSig = mPigeon.getAccelerationZ();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAccelXSig, mAccelYSig, mAccelZSig);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(mAccelXSig, mAccelYSig, mAccelZSig);
  }

  @Override
  public Rotation3d getRotation3d() {
    return mPigeon.getRotation3d();
  }

  @Override
  public LinearAcceleration getAccelerationX() {
    return mAccelXSig.getValue();
  }

  @Override
  public LinearAcceleration getAccelerationY() {
    return mAccelYSig.getValue();
  }

  @Override
  public LinearAcceleration getAccelerationZ() {
    return mAccelZSig.getValue();
  }
}
