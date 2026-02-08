// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Voltage;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

/** {@link SwerveModuleIO} extension for 2x TalonFX + CANcoder */
public class SwerveModuleIOTalonFx extends SwerveModuleIO {
  protected final CANcoder mEncoder;
  protected final TalonFX mPivot, mDrive;
  private final StatusSignal<Angle> mAbsAngle, mPivotAngle, mDriveAngle;
  private final StatusSignal<AngularVelocity> mDriveVelocity;
  private final StatusSignal<AngularAcceleration> mDriveAcceleration;
  private final StatusSignal<Voltage> mPivotVolt, mDriveVolt;
  private final StatusSignal<Current> mPivotSupply,
      mPivotStator,
      mPivotTorque,
      mDriveSupply,
      mDriveStator,
      mDriveTorque;
  private final StatusSignal<Temperature> mPivotTemp, mDriveTemp;

  private final VoltageOut mVoltageOut = new VoltageOut(0.0);
  private final TorqueCurrentFOC mTorqueOut = new TorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC mPoseOut = new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC mFocVelOut =
      new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final MotionMagicVelocityVoltage mVelOut =
      new MotionMagicVelocityVoltage(0.0).withSlot(1);

  /**
   * Create new {@link SwerveModuleIOTalonFx}
   *
   * @param config {@link ModuleConfig} representing the configuration for module
   * @param driveConstants {@link DriveConstants} representing the constants of the overall
   *     drivetrain
   */
  public SwerveModuleIOTalonFx(ModuleConfig config, DriveConstants driveConstants) {
    super(config, driveConstants);

    mEncoder = new CANcoder(mConfig.cancoderId(), mConfig.canBus());
    mPivot = new TalonFX(mConfig.pivotDeviceId(), mConfig.canBus());
    mDrive = new TalonFX(mConfig.driveDeviceId(), mConfig.canBus());

    mEncoder.getConfigurator().apply(mConfig.cancoderConfig());
    mPivot.getConfigurator().apply(mConfig.pivotConfig());
    mDrive.getConfigurator().apply(mConfig.driveConfig());

    mAbsAngle = mEncoder.getAbsolutePosition();
    mPivotAngle = mPivot.getPosition();
    mDriveAngle = mDrive.getPosition();

    mDriveVelocity = mDrive.getVelocity();

    mDriveAcceleration = mDrive.getAcceleration();

    mPivotVolt = mPivot.getMotorVoltage();
    mDriveVolt = mDrive.getMotorVoltage();

    mPivotSupply = mPivot.getSupplyCurrent();
    mPivotStator = mPivot.getStatorCurrent();
    mPivotTorque = mPivot.getTorqueCurrent();
    mDriveSupply = mDrive.getSupplyCurrent();
    mDriveStator = mDrive.getStatorCurrent();
    mDriveTorque = mDrive.getTorqueCurrent();

    mPivotTemp = mPivot.getDeviceTemp();
    mDriveTemp = mDrive.getDeviceTemp();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        mAbsAngle,
        mPivotAngle,
        mDriveVelocity,
        mPivotVolt,
        mDriveVolt,
        mPivotSupply,
        mPivotStator,
        mPivotTorque,
        mDriveSupply,
        mDriveStator,
        mDriveTorque,
        mPivotTemp,
        mDriveTemp);
  }

  @Override
  protected void setPivotTorqueCurrentFocSetpoint(Current current) {
    mPivot.setControl(mTorqueOut.withOutput(current));
  }

  @Override
  protected void setPivotPositionSetpoint(Angle position) {
    mPivot.setControl(mPoseOut.withPosition(position));
  }

  @Override
  protected void setDriveVoltageSetpoint(Voltage voltage, boolean focEnabled) {
    mDrive.setControl(mVoltageOut.withOutput(voltage).withEnableFOC(focEnabled));
  }

  @Override
  protected void setDriveTorqueCurrentFocSetpoint(Current current) {
    mDrive.setControl(mTorqueOut.withOutput(current));
  }

  @Override
  protected void setDriveVoltageVelocitySetpoint(AngularVelocity velocity) {
    mDrive.setControl(mVelOut.withVelocity(velocity));
  }

  @Override
  protected void setDriveTorqueVelocitySetpoint(AngularVelocity velocity, Current wheelForceAmps) {
    mDrive.setControl(mFocVelOut.withVelocity(velocity).withFeedForward(wheelForceAmps));
  }

  @Override
  public void stop() {
    mPivot.stopMotor();
    mDrive.stopMotor();
  }

  @Override
  public Angle getAbsEncoderAngle() {
    return mAbsAngle.getValue();
  }

  @Override
  public Voltage getPivotAppliedVoltage() {
    return mPivotVolt.getValue();
  }

  @Override
  public Current getPivotSupplyCurrent() {
    return mPivotSupply.getValue();
  }

  @Override
  public Current getPivotStatorCurrent() {
    return mPivotStator.getValue();
  }

  @Override
  public Current getPivotTorqueCurrent() {
    return mPivotTorque.getValue();
  }

  @Override
  public Angle getPivotAngle() {
    return mPivotAngle.getValue();
  }

  @Override
  public Temperature getPivotTemperature() {
    return mPivotTemp.getValue();
  }

  @Override
  public Voltage getDriveAppliedVoltage() {
    return mDriveVolt.getValue();
  }

  @Override
  public Current getDriveSupplyCurrent() {
    return mDriveSupply.getValue();
  }

  @Override
  public Current getDriveStatorCurrent() {
    return mDriveStator.getValue();
  }

  @Override
  public Current getDriveTorqueCurrent() {
    return mDriveTorque.getValue();
  }

  @Override
  public Angle getDriveAngle() {
    return mDriveAngle.getValue();
  }

  @Override
  public Temperature getDriveTemperature() {
    return mDriveTemp.getValue();
  }

  @Override
  public AngularVelocity getDriveAngularVelocity() {
    return mDriveVelocity.getValue();
  }

  @Override
  public AngularAcceleration getDriveAngularAcceleration() {
    return mDriveAcceleration.getValue();
  }
}
