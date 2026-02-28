// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

// TODO
public class SwerveModuleIOServo extends SwerveModuleIO {
  @Logged private final ServoIO mPivot, mDrive;
  @Logged private final EncoderIO mEncoder;

  public SwerveModuleIOServo(
      String name,
      EncoderIO encoder,
      ServoIO pivot,
      ServoIO drive,
      ModuleConfig config,
      DriveConstants constants) {
    super(name, config, constants);

    mPivot = pivot;
    mDrive = drive;
    mEncoder = encoder;
  }

  @Override
  public void periodic() {
    mPivot.periodic();
    mDrive.periodic();
    mEncoder.periodic();
  }

  @Override
  protected Angle getEncoderAngle() {
    return mEncoder.getAngle();
  }

  @Override
  protected Angle getPivotAngle() {
    return mPivot.getAngle();
  }

  @Override
  public Temperature getPivotTemperature() {
    return mPivot.getTemperature();
  }

  @Override
  protected Angle getDriveAngle() {
    return mDrive.getAngle();
  }

  @Override
  protected AngularVelocity getDriveAngularVelocity() {
    return mDrive.getAngularVelocity();
  }

  @Override
  protected AngularAcceleration getDriveAngularAcceleration() {
    return mDrive.getAngularAcceleration();
  }

  @Override
  public Temperature getDriveTemperature() {
    return mDrive.getTemperature();
  }

  @Override
  public void setPivotCurrent(Current current) {
    mPivot.setTorqueCurrentSetpoint(current);
  }

  @Override
  protected void setPivotSetpoint(Angle setpoint) {
    mPivot.setTorqueMotionProfiledPositionSetpoint(setpoint);
  }

  @Override
  public void setDriveCurrent(Current current) {
    mDrive.setTorqueCurrentSetpoint(current);
  }

  @Override
  protected void setDriveSetpoint(AngularVelocity velocity, boolean focEnabled) {
    if (focEnabled) mDrive.setTorqueMotionProfiledVelocitySetpoint(velocity);
    else mDrive.setVoltageMotionProfiledVelocitySetpoint(velocity, false);
  }

  @Override
  protected void setDriveSetpoint(AngularVelocity velocity, Torque torque) {
    mDrive.setTorqueMotionProfiledVelocitySetpoint(velocity, torque);
  }

  @Override
  public void resetEncoders(Angle pivotAngle, Angle driveAngle) {
    mPivot.resetEncoder(pivotAngle);
    mDrive.resetEncoder(driveAngle);
  }

  @Override
  public void stop() {
    mPivot.stop();
    mDrive.stop();
  }
}
