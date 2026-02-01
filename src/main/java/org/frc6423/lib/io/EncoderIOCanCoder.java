// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;

/**
 * Implementation of {@link EncoderIO} for CANcoder based Encoders
 *
 * @see https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html
 */
public class EncoderIOCanCoder extends EncoderIO {
  private final CANcoder mEncoder;

  private final CANcoderConfiguration mConfig;

  private final StatusSignal<Angle> mAngleSig;

  public EncoderIOCanCoder(int canDeviceId, CANBus canBusId, CANcoderConfiguration config) {
    mEncoder = new CANcoder(canDeviceId, canBusId);

    mEncoder.getConfigurator().apply(config);
    mConfig = config;

    mAngleSig = mEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(mAngleSig);
  }

  @Override
  public void reset(Angle angle) {
    mEncoder.setPosition(angle);
  }

  @Override
  public Angle getAngle() {
    return mAngleSig.getValue();
  }
}
