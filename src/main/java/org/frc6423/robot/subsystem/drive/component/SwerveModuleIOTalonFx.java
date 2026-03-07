// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import static edu.wpi.first.units.Units.NewtonMeters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

/**
 * Interface for interacting with a swerve module with {@link TalonFX} & {@link CANcoder} hardware
 */
public class SwerveModuleIOTalonFx extends SwerveModuleIO {
  protected final CANcoder mEncoder;
  protected final CANcoderConfiguration mEncoderConfig;

  protected final TalonFX mPivot, mDrive;
  protected final TalonFXConfiguration mPivotConfig, mDriveConfig;
  protected final StatusSignal<Voltage> mPSupplyVoltageSupplier, mPStatorVoltageSupplier;
  protected final StatusSignal<Current> mPSupplyCurrentSupplier,
      mPStatorCurrentSupplier,
      mPTorqueCurrentSupplier;
  protected final StatusSignal<Temperature> mPTemperatureSupplier;
  protected final StatusSignal<Angle> mPAngleSupplier, mPRawAngleSupplier;
  protected final StatusSignal<Voltage> mDSupplyVoltageSupplier, mDStatorVoltageSupplier;
  protected final StatusSignal<Current> mDSupplyCurrentSupplier,
      mDStatorCurrentSupplier,
      mDTorqueCurrentSupplier;
  protected final StatusSignal<Temperature> mDTemperatureSupplier;
  protected final StatusSignal<Angle> mDAngleSupplier;
  protected final StatusSignal<AngularVelocity> mDVelocitySupplier;
  protected final StatusSignal<AngularAcceleration> mDAccelerationSupplier;

  protected final VoltageOut mVoltOut = new VoltageOut(0.0).withEnableFOC(false);
  protected final TorqueCurrentFOC mCurrentOut = new TorqueCurrentFOC(0.0);
  protected final MotionMagicTorqueCurrentFOC mTorquePoseOut = new MotionMagicTorqueCurrentFOC(0.0);
  protected final MotionMagicVelocityTorqueCurrentFOC mTorqueVelOut =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);
  protected final MotionMagicVelocityVoltage mVoltVelOut =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(false);

  public SwerveModuleIOTalonFx(String name, ModuleConfig config, DriveConstants constants) {
    super(name, config, constants);

    // Init hardware
    mEncoder = new CANcoder(config.cancoderId(), config.canBus());
    mPivot = new TalonFX(config.pivotDeviceId(), config.canBus());
    mDrive = new TalonFX(config.driveDeviceId(), config.canBus());

    // Configure Hardware
    mEncoderConfig = config.cancoderConfig();
    mPivotConfig = config.pivotConfig();
    mDriveConfig = config.driveConfig();

    mEncoder.getConfigurator().apply(mEncoderConfig);
    mPivot.getConfigurator().apply(mPivotConfig);
    mDrive.getConfigurator().apply(mDriveConfig);

    // Setup signals
    mPSupplyVoltageSupplier = mPivot.getSupplyVoltage(true);
    mPStatorVoltageSupplier = mPivot.getMotorVoltage(true);

    mPSupplyCurrentSupplier = mPivot.getSupplyCurrent(true);
    mPStatorCurrentSupplier = mPivot.getStatorCurrent(true);
    mPTorqueCurrentSupplier = mPivot.getTorqueCurrent(true);

    mPTemperatureSupplier = mPivot.getDeviceTemp(true);

    mPAngleSupplier = mPivot.getPosition(true);
    mPRawAngleSupplier = mPivot.getRotorPosition(true);

    mDSupplyVoltageSupplier = mDrive.getSupplyVoltage(true);
    mDStatorVoltageSupplier = mDrive.getMotorVoltage(true);

    mDSupplyCurrentSupplier = mDrive.getSupplyCurrent(true);
    mDStatorCurrentSupplier = mDrive.getStatorCurrent(true);
    mDTorqueCurrentSupplier = mDrive.getTorqueCurrent(true);

    mDTemperatureSupplier = mDrive.getDeviceTemp(true);

    mDAngleSupplier = mDrive.getPosition(true);
    mDVelocitySupplier = mDrive.getVelocity(true);
    mDAccelerationSupplier = mDrive.getAcceleration(true);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        mPSupplyVoltageSupplier,
        mPStatorVoltageSupplier,
        mPSupplyCurrentSupplier,
        mPStatorCurrentSupplier,
        mPTorqueCurrentSupplier,
        mPTemperatureSupplier,
        mPAngleSupplier,
        mPRawAngleSupplier,
        mDSupplyVoltageSupplier,
        mDStatorVoltageSupplier,
        mDSupplyCurrentSupplier,
        mDStatorCurrentSupplier,
        mDTorqueCurrentSupplier,
        mDTemperatureSupplier,
        mDAngleSupplier,
        mDVelocitySupplier,
        mDAccelerationSupplier);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        mPSupplyVoltageSupplier,
        mPStatorVoltageSupplier,
        mPSupplyCurrentSupplier,
        mPStatorCurrentSupplier,
        mPTorqueCurrentSupplier,
        mPTemperatureSupplier,
        mPAngleSupplier,
        mPRawAngleSupplier,
        mDSupplyVoltageSupplier,
        mDStatorVoltageSupplier,
        mDSupplyCurrentSupplier,
        mDStatorCurrentSupplier,
        mDTorqueCurrentSupplier,
        mDTemperatureSupplier,
        mDAngleSupplier,
        mDVelocitySupplier,
        mDAccelerationSupplier);
  }

  @Override
  public Voltage getPivotSupplyVoltage() {
    return mPSupplyVoltageSupplier.getValue();
  }

  @Override
  public Voltage getPivotStatorVoltage() {
    return mPStatorVoltageSupplier.getValue();
  }

  @Override
  public Current getPivotSupplyCurrent() {
    return mPSupplyCurrentSupplier.getValue();
  }

  @Override
  public Current getPivotStatorCurrent() {
    return mPStatorCurrentSupplier.getValue();
  }

  @Override
  public Current getPivotTorqueCurrent() {
    return mPTorqueCurrentSupplier.getValue();
  }

  @Override
  public Temperature getPivotTemperature() {
    return mPTemperatureSupplier.getValue();
  }

  @Override
  public Angle getPivotAngle() {
    return mPAngleSupplier.getValue();
  }

  @Override
  public void setPivotVoltage(Voltage voltage) {
    mPivot.setControl(mVoltOut.withOutput(voltage));
  }

  @Override
  public void setPivotCurrent(Current current) {
    mPivot.setControl(mCurrentOut.withOutput(current));
  }

  @Override
  protected void setPivotSetpoint(Angle setpoint) {
    mPivot.setControl(mTorquePoseOut.withPosition(setpoint));
  }

  @Override
  public Voltage getDriveSupplyVoltage() {
    return mDSupplyVoltageSupplier.getValue();
  }

  @Override
  public Voltage getDriveStatorVoltage() {
    return mDStatorVoltageSupplier.getValue();
  }

  @Override
  public Current getDriveSupplyCurrent() {
    return mDSupplyCurrentSupplier.getValue();
  }

  @Override
  public Current getDriveStatorCurrent() {
    return mDStatorCurrentSupplier.getValue();
  }

  @Override
  public Current getDriveTorqueCurrent() {
    return mDTorqueCurrentSupplier.getValue();
  }

  @Override
  public Temperature getDriveTemperature() {
    return mDTemperatureSupplier.getValue();
  }

  @Override
  public Angle getDriveAngle() {
    return mDAngleSupplier.getValue();
  }

  @Override
  public AngularVelocity getDriveAngularVelocity() {
    return mDVelocitySupplier.getValue();
  }

  @Override
  public AngularAcceleration getDriveAngularAcceleration() {
    return mDAccelerationSupplier.getValue();
  }

  @Override
  public void setDriveVoltage(Voltage voltage) {
    mDrive.setControl(mVoltOut.withOutput(voltage));
  }

  @Override
  public void setDriveCurrent(Current current) {
    mDrive.setControl(mCurrentOut.withOutput(current));
  }

  @Override
  protected void setDriveSetpoint(AngularVelocity velocity, boolean focEnabled) {
    if (focEnabled) mDrive.setControl(mTorqueVelOut.withVelocity(velocity).withSlot(0));
    else mDrive.setControl(mVoltVelOut.withVelocity(velocity).withSlot(1));
  }

  @Override
  protected void setDriveSetpoint(AngularVelocity velocity, Torque torque) {
    mDrive.setControl(
        mTorqueVelOut
            .withVelocity(velocity)
            .withFeedForward(mConstants.getDriveGearboxKt() / torque.in(NewtonMeters)));
  }

  @Override
  public void resetEncoders(Angle pivotAngle, Angle driveAngle) {
    mPivot.setPosition(pivotAngle);
    mDrive.setPosition(driveAngle);
  }

  @Override
  public void stop() {
    mPivot.stopMotor();
    mDrive.stopMotor();
  }
}
