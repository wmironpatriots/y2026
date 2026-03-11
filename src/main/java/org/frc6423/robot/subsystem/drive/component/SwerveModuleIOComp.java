// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import org.frc6423.lib.util.PhoneixUtils;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystem.drive.constants.SwerveConstants.ModuleConfig;

public class SwerveModuleIOComp extends SwerveModuleIO {
  protected final CANcoder mEncoder;
  protected final TalonFX mPivot, mDrive;

  private static final TunableNumber kS = new TunableNumber("Drive/kS", 0.0);
  private static final TunableNumber kV = new TunableNumber("Drive/kV", 0.0);
  private static final TunableNumber kA = new TunableNumber("Drive/kA", 0.0);
  private static final TunableNumber kP = new TunableNumber("Drive/kP", 0.0);
  private static final TunableNumber kD = new TunableNumber("Drive/kD", 0.0);

  private final VoltageOut mVoltageReq = new VoltageOut(0.0);
  private final TorqueCurrentFOC mCurrentReq = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC mPositionReq =
      new PositionTorqueCurrentFOC(0.0).withSlot(0);
  private final MotionMagicVelocityVoltage mVelocityReq =
      new MotionMagicVelocityVoltage(0.0).withSlot(1);
  private final MotionMagicVelocityTorqueCurrentFOC mVelocityFocReq =
      new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);

  private final BaseStatusSignal mPivotVolts,
      mPivotSupply,
      mPivotStator,
      mPivotTorque,
      mPivotTemp,
      mPivotAngle,
      mPivotSpeed;
  private final BaseStatusSignal mDriveVolts,
      mDriveSupply,
      mDriveStator,
      mDriveTorque,
      mDriveTemp,
      mDriveAngle,
      mDriveSpeed;

  public SwerveModuleIOComp(ModuleConfig config) {
    super(config);

    mEncoder = new CANcoder(config.cancoderId(), config.canBus());
    mPivot = new TalonFX(config.pivotDeviceId(), config.canBus());
    mDrive = new TalonFX(config.driveDeviceId(), config.canBus());

    mEncoder.getConfigurator().apply(config.cancoderConfig());
    mPivot.getConfigurator().apply(config.pivotConfig());
    mDrive.getConfigurator().apply(config.driveConfig());

    mPivotVolts = mPivot.getMotorVoltage(true);
    mPivotSupply = mPivot.getSupplyCurrent(true);
    mPivotStator = mPivot.getStatorCurrent(true);
    mPivotTorque = mPivot.getTorqueCurrent(true);
    mPivotTemp = mPivot.getDeviceTemp(true);
    mPivotAngle = mPivot.getPosition(true);
    mPivotSpeed = mPivot.getVelocity(true);

    mDriveVolts = mDrive.getMotorVoltage(true);
    mDriveSupply = mDrive.getSupplyCurrent(true);
    mDriveStator = mDrive.getStatorCurrent(true);
    mDriveTorque = mDrive.getTorqueCurrent(true);
    mDriveTemp = mDrive.getDeviceTemp(true);
    mDriveAngle = mDrive.getPosition(true);
    mDriveSpeed = mDrive.getVelocity(true);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        mDriveAngle,
        mDriveSpeed,
        mPivotAngle,
        mPivotSpeed,
        mPivotVolts,
        mPivotSupply,
        mPivotTorque,
        mPivotTemp,
        mPivotAngle,
        mPivotSpeed,
        mDriveVolts,
        mDriveSupply,
        mDriveStator,
        mDriveTorque,
        mDriveTemp,
        mDriveAngle,
        mDriveSpeed);

    ParentDevice.optimizeBusUtilizationForAll(mEncoder, mPivot, mDrive);
  }

  @Override
  public void periodic() {
    if (kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())) {
      new Thread(
              () -> {
                mConfig.driveConfig().Slot0.kS = kS.get();
                mConfig.driveConfig().Slot0.kV = kV.get();
                mConfig.driveConfig().Slot0.kA = kA.get();
                mConfig.driveConfig().Slot0.kP = kP.get();
                mConfig.driveConfig().Slot0.kD = kD.get();

                PhoneixUtils.tryUntilOk(
                    5, () -> mDrive.getConfigurator().apply(mConfig.driveConfig()));
              })
          .start();
    }

    BaseStatusSignal.refreshAll(
        mDriveAngle,
        mDriveSpeed,
        mPivotAngle,
        mPivotSpeed,
        mPivotVolts,
        mPivotSupply,
        mPivotTorque,
        mPivotTemp,
        mPivotAngle,
        mPivotSpeed,
        mDriveVolts,
        mDriveSupply,
        mDriveStator,
        mDriveTorque,
        mDriveTemp,
        mDriveAngle,
        mDriveSpeed);
  }

  @Override
  public double getPivotAppliedVolts() {
    return mPivotVolts.getValueAsDouble();
  }

  @Override
  public double getPivotSupplyCurrentAmps() {
    return mPivotSupply.getValueAsDouble();
  }

  @Override
  public double getPivotStatorCurrentAmps() {
    return mPivotStator.getValueAsDouble();
  }

  @Override
  public double getPivotSupplyTorqueAmps() {
    return mPivotSupply.getValueAsDouble();
  }

  @Override
  public double getPivotTemperatureCelsius() {
    return mPivotTemp.getValueAsDouble();
  }

  @Override
  public double getPivotAngleRevs() {
    return mPivotAngle.getValueAsDouble();
  }

  @Override
  public void runPivotCharacterizationVoltage(double volts) {
    mPivot.setControl(mVoltageReq.withOutput(volts));
  }

  @Override
  public void runPivotCharacterizationCurrent(double amps) {
    mPivot.setControl(mCurrentReq.withOutput(amps));
  }

  @Override
  protected void setPivotAngleSetpoint(double angleRevs) {
    mPivot.setControl(mPositionReq.withPosition(angleRevs).withSlot(0));
  }

  @Override
  public double getDriveAppliedVolts() {
    return mDriveVolts.getValueAsDouble();
  }

  @Override
  public double getDriveSupplyCurrentAmps() {
    return mDriveSupply.getValueAsDouble();
  }

  @Override
  public double getDriveStatorCurrentAmps() {
    return mDriveStator.getValueAsDouble();
  }

  @Override
  public double getDriveSupplyTorqueAmps() {
    return mDriveTorque.getValueAsDouble();
  }

  @Override
  public double getDriveTemperatureCelsius() {
    return mDriveTemp.getValueAsDouble();
  }

  @Override
  public double getDriveAngleRevs() {
    return mDriveAngle.getValueAsDouble();
  }

  @Override
  public double getDriveAngularSpeedRevsPerSec() {
    return mDriveSpeed.getValueAsDouble();
  }

  @Override
  public void runDriveCharacterizationVoltage(double volts) {
    mDrive.setControl(mVoltageReq.withOutput(volts));
  }

  @Override
  public void runDriveCharacterizationCurrent(double amps) {
    mDrive.setControl(mCurrentReq.withOutput(amps));
  }

  @Override
  protected void setDriveSpeedSetpoint(double speedRevsPerSec, boolean focEnabled) {
    if (focEnabled) mDrive.setControl(mVelocityFocReq.withVelocity(speedRevsPerSec).withSlot(0));
    else mDrive.setControl(mVelocityReq.withVelocity(speedRevsPerSec).withSlot(1));
  }

  @Override
  protected void setDriveSpeedSetpoint(double speedRevsPerSec, double torqueNm) {
    mDrive.setControl(
        mVelocityFocReq
            .withVelocity(speedRevsPerSec)
            .withFeedForward(Flags.kDriveConstants.getDriveGearboxKt() / torqueNm)
            .withSlot(0));
  }

  @Override
  public void neutral() {
    mPivot.stopMotor();
    mDrive.stopMotor();
  }
}
