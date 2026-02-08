// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.module;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

/** {@link SwerveModuleIO} extension using the {@link DCMotorSim} WPILib class */
public class SwerveModuleIODcMotorSim extends SwerveModuleIO {
  /**
   * Create new {@link SwerveModuleIODcMotorSim}
   *
   * @param config {@link ModuleConfig} representing the configuration for module
   * @param driveConstants {@link DriveConstants} representing the constants of the overall
   *     drivetrain
   */
  public SwerveModuleIODcMotorSim(ModuleConfig config, DriveConstants driveConstants) {
    super(config, driveConstants);
    // TODO Auto-generated constructor stub
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  protected void setPivotTorqueCurrentFocSetpoint(Current current) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'setPivotTorqueCurrentFocSetpoint'");
  }

  @Override
  protected void setPivotPositionSetpoint(Angle position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPivotPositionSetpoint'");
  }

  @Override
  protected void setDriveVoltageSetpoint(Voltage voltage, boolean focEnabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveVoltageSetpoint'");
  }

  @Override
  protected void setDriveTorqueCurrentFocSetpoint(Current current) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'setDriveTorqueCurrentFocSetpoint'");
  }

  @Override
  protected void setDriveVoltageVelocitySetpoint(AngularVelocity velocity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'setDriveVoltageVelocitySetpoint'");
  }

  @Override
  protected void setDriveTorqueVelocitySetpoint(AngularVelocity velocity, Current wheelForceAmps) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'setDriveTorqueVelocitySetpoint'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public Angle getAbsEncoderAngle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAbsEncoderAngle'");
  }

  @Override
  public Voltage getPivotAppliedVoltage() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotAppliedVoltage'");
  }

  @Override
  public Current getPivotSupplyCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotSupplyCurrent'");
  }

  @Override
  public Current getPivotStatorCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotStatorCurrent'");
  }

  @Override
  public Current getPivotTorqueCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotTorqueCurrent'");
  }

  @Override
  public Angle getPivotAngle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotAngle'");
  }

  @Override
  public Temperature getPivotTemperature() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotTemperature'");
  }

  @Override
  public Voltage getDriveAppliedVoltage() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveAppliedVoltage'");
  }

  @Override
  public Current getDriveSupplyCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveSupplyCurrent'");
  }

  @Override
  public Current getDriveStatorCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveStatorCurrent'");
  }

  @Override
  public Current getDriveTorqueCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveTorqueCurrent'");
  }

  @Override
  public Angle getDriveAngle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveAngle'");
  }

  @Override
  public Temperature getDriveTemperature() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveTemperature'");
  }

  @Override
  public AngularVelocity getDriveAngularVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveAngularVelocity'");
  }

  @Override
  public AngularAcceleration getDriveAngularAcceleration() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveAngularAcceleration'");
  }
}
