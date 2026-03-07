// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ServoIONone extends ServoIO {
  protected ServoIONone(String name) {
    super(name, 0, new TalonFXConfiguration());
  }

  @Override
  public void periodic() {}

  @Override
  public double getMotorKtNewtonMetersPerAmps() {
    return 0.0;
  }

  @Override
  public double getAppliedVolts() {
    return 0.0;
  }

  @Override
  public double getSupplyCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getTorqueCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getTemperatureCelsius() {
    return 0.0;
  }

  @Override
  public double getAngularPositionRevs() {
    return 0.0;
  }

  @Override
  public double getAngularVelocityRevsPerSec() {
    return 0.0;
  }

  @Override
  public double getAngularAccelerationRevsPerSecPerSec() {
    return 0.0;
  }

  @Override
  public void setLeader(ServoIO leader, boolean flipped) {}

  @Override
  public void setBrakeModeStatus(boolean brakeEnabled) {}

  @Override
  public void resetRelativeEncoder(double positionRevs) {}

  @Override
  public void setNeutral() {}

  @Override
  public void setTorqueCurrentOutput(double torqueNewtonMeters) {}

  @Override
  public void setPositionSetpoint(double positionRevs) {}

  @Override
  public void setPositionSetpoint(double positionRevs, double feedforward) {}

  @Override
  public void setVelocitySetpoint(double velocityRevsPerSec) {}

  @Override
  public void setVelocitySetpoint(double velocityRevsPerSec, double feedforward) {}

  @Override
  public void setProfiledPositionSetpoint(double positionRevs) {}

  @Override
  public void setProfiledPositionSetpoint(double positionRevs, double feedforward) {}

  @Override
  public void setProfiledVelocitySetpoint(double velocityRevsPerSec) {}

  @Override
  public void setProfiledVelocitySetpoint(double velocityRevsPerSec, double feedforward) {}
}
