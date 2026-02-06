// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.GyroConfig;

/**
 * Represents a {@link GyroIO} implementation for a CTRe Pigeon2
 *
 * @see https://ctre.download/files/user-manual/Pigeon2%20User's%20Guide.pdf
 */
public class GyroIOPigeon2 extends GyroIO {
  protected final Pigeon2 mPigeon;

  protected final StatusSignal<Angle> mPitch, mYaw, mRoll;

  /**
   * Create new {@link GyroIOPigeon2}
   *
   * @param config {@link GyroConfig} representing the configuration for system
   */
  public GyroIOPigeon2(GyroConfig config) {
    mPigeon = new Pigeon2(config.deviceId(), config.canBus());

    mPitch = mPigeon.getPitch();
    mYaw = mPigeon.getYaw();
    mRoll = mPigeon.getRoll();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(mPitch, mYaw, mRoll);
  }

  @Override
  public Rotation3d getRotation3d() {
    return new Rotation3d(mRoll.getValue(), mPitch.getValue(), mYaw.getValue());
  }
}
