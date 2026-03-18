// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.frc6423.lib.util.PhoneixUtils;

public class HoodIOReal extends HoodIO {
  protected final TalonFX mServo;
  protected final TalonFXConfiguration mConfig;

  protected final BaseStatusSignal mMeasuredVoltage,
      mMeasuredStatorCurrent,
      mMeasuredSupplyCurrent,
      mMeasuredTemperature,
      mMeasuredPosition;

  protected final VoltageOut mVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  protected final MotionMagicTorqueCurrentFOC mPositionRequest =
      new MotionMagicTorqueCurrentFOC(0.0);

  protected boolean mIsConnected = false;

  public HoodIOReal(int canDeviceId, CANBus canBus, TalonFXConfiguration config) {
    mServo = new TalonFX(canDeviceId, canBus);
    mConfig = config;

    mServo.getConfigurator().apply(config);

    mMeasuredVoltage = mServo.getMotorVoltage();
    mMeasuredStatorCurrent = mServo.getStatorCurrent();
    mMeasuredSupplyCurrent = mServo.getSupplyCurrent();
    mMeasuredTemperature = mServo.getDeviceTemp();
    mMeasuredPosition = mServo.getPosition();
  }

  @Override
  public boolean isConnected() {
    return mIsConnected;
  }

  @Override
  public void periodic() {
    mIsConnected =
        BaseStatusSignal.refreshAll(
                mMeasuredVoltage,
                mMeasuredStatorCurrent,
                mMeasuredSupplyCurrent,
                mMeasuredTemperature,
                mMeasuredPosition)
            .isOK();
  }

  @Override
  public double getOutputVoltage() {
    return mMeasuredVoltage.getValueAsDouble();
  }

  @Override
  public double getStatorCurrentAmps() {
    return mMeasuredStatorCurrent.getValueAsDouble();
  }

  @Override
  public double getSupplyCurrentAmps() {
    return mMeasuredSupplyCurrent.getValueAsDouble();
  }

  @Override
  public double getTemperatureCelsius() {
    return mMeasuredTemperature.getValueAsDouble();
  }

  @Override
  public double getPositionRevs() {
    return mMeasuredPosition.getValueAsDouble();
  }

  @Override
  public void setTargetVoltage(double volts) {
    mServo.setControl(mVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setTargetPosition(double positionRevs) {
    mServo.setControl(mPositionRequest.withPosition(positionRevs));
  }

  @Override
  public void stop() {
    mServo.stopMotor();
  }

  @Override
  public void resetEncoder(double positionRevs) {
    mServo.setPosition(positionRevs);
  }

  @Override
  public void setProfilingConstraints(double cruiseVelocity, double acceleration) {
    new Thread(
            () -> {
              mConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
              mConfig.MotionMagic.MotionMagicAcceleration = acceleration;

              PhoneixUtils.tryUntilOk(3, () -> mServo.getConfigurator().apply(mConfig));
            })
        .run();
  }

  @Override
  public void setGains(double kS, double kG, double kV, double kA, double kP, double kD) {
    new Thread(
            () -> {
              mConfig.Slot0.kS = kS;
              mConfig.Slot0.kG = kG;
              mConfig.Slot0.kV = kV;
              mConfig.Slot0.kA = kA;
              mConfig.Slot0.kP = kP;
              mConfig.Slot0.kD = kD;

              PhoneixUtils.tryUntilOk(3, () -> mServo.getConfigurator().apply(mConfig));
            })
        .run();
  }

  @Override
  public void enableBrake(boolean enabled) {
    new Thread(
            () -> {
              mConfig.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

              PhoneixUtils.tryUntilOk(3, () -> mServo.getConfigurator().apply(mConfig));
            })
        .run();
  }
}
