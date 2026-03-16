// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

/** {@link RollerIO} extension for placeholder hardware (used for simulating roller hardware) */
public class RollerIONone extends RollerIO {
  private double mVoltageOutput;

  @Override
  public void periodic() {}

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public double getAppliedVoltage() {
    return mVoltageOutput;
  }

  @Override
  public void setVoltageOutput(double volts) {
    mVoltageOutput = volts;
  }

  @Override
  public void stop() {
    mVoltageOutput = 0.0;
  }
}
