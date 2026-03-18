// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.frc6423.lib.util.PhoneixUtils;

public class FlywheelIOReal extends FlywheelIO {
  protected final TalonFX mLeft, mRight;
  protected final TalonFXConfiguration mConfig;

  protected final BaseStatusSignal mMeasuredVoltage,
      mMeasuredStatorCurrent,
      mMeasuredSupplyCurrent,
      mMeasuredTemperature,
      mMeasuredPosition,
      mMeasuredVelocity,
      mMeasuredAcceleration;

  protected final TorqueCurrentFOC mCurrentRequest = new TorqueCurrentFOC(0.0);
  protected final MotionMagicVelocityTorqueCurrentFOC mVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  protected boolean mIsConnected = false;

  public FlywheelIOReal(
      int leftCanDeviceId, int rightCanDeviceId, CANBus canBus, TalonFXConfiguration config) {
    mLeft = new TalonFX(leftCanDeviceId, canBus);
    mRight = new TalonFX(rightCanDeviceId, canBus);
    mConfig = config;

    mLeft.getConfigurator().apply(config);
    mRight.getConfigurator().apply(config);

    mRight.setControl(new Follower(leftCanDeviceId, MotorAlignmentValue.Opposed));

    mMeasuredVoltage = mLeft.getMotorVoltage();
    mMeasuredStatorCurrent = mLeft.getStatorCurrent();
    mMeasuredSupplyCurrent = mLeft.getSupplyCurrent();
    mMeasuredTemperature = mLeft.getDeviceTemp();
    mMeasuredPosition = mLeft.getPosition();
    mMeasuredVelocity = mLeft.getVelocity();
    mMeasuredAcceleration = mLeft.getAcceleration();
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
                mMeasuredPosition,
                mMeasuredVelocity,
                mMeasuredAcceleration)
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
  public double getVelocityRevsPerSec() {
    return mMeasuredVelocity.getValueAsDouble();
  }

  @Override
  public double getAccelerationRevsPerSecPerSec() {
    return mMeasuredAcceleration.getValueAsDouble();
  }

  @Override
  public void setTargetTorqueCurrent(double amps) {
    mLeft.setControl(mCurrentRequest.withOutput(amps));
  }

  @Override
  public void setTargetVelocity(double revsPerSec) {
    setTargetVelocity(revsPerSec, 0.0);
  }

  @Override
  public void setTargetVelocity(double revsPerSec, double feedforwardAmps) {
    mLeft.setControl(mVelocityRequest.withVelocity(revsPerSec).withFeedForward(feedforwardAmps));
  }

  @Override
  public void stop() {
    mLeft.stopMotor();
  }

  @Override
  public void setProfilingConstraints(double cruiseVelocity, double acceleration) {
    new Thread(
            () -> {
              mConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
              mConfig.MotionMagic.MotionMagicAcceleration = acceleration;

              PhoneixUtils.tryUntilOk(3, () -> mLeft.getConfigurator().apply(mConfig));
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

              PhoneixUtils.tryUntilOk(3, () -> mLeft.getConfigurator().apply(mConfig));
            })
        .run();
  }

  @Override
  public void enableBrake(boolean enabled) {
    new Thread(
            () -> {
              mConfig.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

              PhoneixUtils.tryUntilOk(3, () -> mLeft.getConfigurator().apply(mConfig));
            })
        .run();
  }
}
