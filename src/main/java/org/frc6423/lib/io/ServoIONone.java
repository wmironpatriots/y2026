// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;

/** Blank {@link ServoIO} extension for simulation */
public class ServoIONone extends ServoIO {
  public ServoIONone(String name) {
    super(name, CANBus.roboRIO(), 0, new TalonFXConfiguration());
  }

  @Override
  public void periodic() {}

  @Override
  public Per<TorqueUnit, CurrentUnit> getMotorKt() {
    return NewtonMeters.zero().per(Amps);
  }

  @Override
  public Voltage getSupplyVoltage() {
    return Volts.zero();
  }

  @Override
  public Voltage getStatorVoltage() {
    return Volts.zero();
  }

  @Override
  public Current getSupplyCurrent() {
    return Amps.zero();
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.zero();
  }

  @Override
  public Current getTorqueCurrent() {
    return Amps.zero();
  }

  @Override
  public Angle getRawAngle() {
    return Revolution.zero();
  }

  @Override
  public Angle getAngle() {
    return Revolution.zero();
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return RevolutionsPerSecond.zero();
  }

  @Override
  public AngularAcceleration getAngularAcceleration() {
    return RadiansPerSecondPerSecond.zero();
  }

  @Override
  public Temperature getTemperature() {
    return Celsius.zero();
  }

  @Override
  public void setLeader(ServoIO leader, boolean flipped) {}

  @Override
  public void setGainsSlot(int slot) {}

  @Override
  public void setBrakeStatus(boolean active) {}

  @Override
  public void resetEncoder(Angle angle) {}

  @Override
  public void stop() {}

  @Override
  public void setVoltageSetpoint(Voltage voltage, boolean withFoc) {}

  @Override
  public void setVoltagePositionSetpoint(Angle angle, boolean withFoc) {}

  @Override
  public void setVoltagePositionSetpoint(Angle angle, Voltage feedforward, boolean withFoc) {}

  @Override
  public void setVoltageVelocitySetpoint(AngularVelocity velocity, boolean withFoc) {}

  @Override
  public void setVoltageVelocitySetpoint(
      AngularVelocity velocity, Voltage feedforward, boolean withFoc) {}

  @Override
  public void setVoltageMotionProfiledPositionSetpoint(Angle angle, boolean withFoc) {}

  @Override
  public void setVoltageMotionProfiledPositionSetpoint(
      Angle angle, Voltage feedforward, boolean withFoc) {}

  @Override
  public void setVoltageMotionProfiledVelocitySetpoint(AngularVelocity velocity, boolean withFoc) {}

  @Override
  public void setTorqueCurrentSetpoint(Current current) {}

  @Override
  public void setTorquePositionSetpoint(Angle angle) {}

  @Override
  public void setTorquePositionSetpoint(Angle angle, Torque torque) {}

  @Override
  public void setTorquePositionSetpoint(Angle angle, Current feedforward) {}

  @Override
  public void setTorqueVelocitySetpoint(AngularVelocity velocity) {}

  @Override
  public void setTorqueVelocitySetpoint(AngularVelocity velocity, Torque torque) {}

  @Override
  public void setTorqueVelocitySetpoint(AngularVelocity velocity, Current feedforward) {}

  @Override
  public void setTorqueVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration) {}

  @Override
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle) {}

  @Override
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle, Torque torque) {}

  @Override
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle, Current feedforward) {}

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity) {}

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity, Torque torque) {}

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, Current feedforward) {}

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration) {}
}
