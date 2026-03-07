// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import org.frc6423.lib.util.PhoneixUtils;

public class ServoIOTalonFx extends ServoIO {
  protected final TalonFX mServo;
  protected final MotorType mTalonType;

  private int mGainsSlot = 0;

  public final BaseStatusSignal mVoltsSignal,
      mSupplyAmpsSignal,
      mStatorAmpsSignal,
      mTorqueAmpsSignal,
      mCelsiusSignal,
      mRevsSignal,
      mRevsPerSecSignal,
      mRevsPerSecPerSecSignal;

  private final TorqueCurrentFOC mTorqueCurrentRequest = new TorqueCurrentFOC(0.0);

  private final PositionTorqueCurrentFOC mPositionRequest = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC mVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private final MotionMagicTorqueCurrentFOC mProfiledPositionRequest =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC mProfiledVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  protected ServoIOTalonFx(
      String name, MotorType type, CANBus canBus, int canDeviceId, TalonFXConfiguration config) {
    super(name, canDeviceId, config);

    mServo = new TalonFX(canDeviceId, canBus);
    mServo.getConfigurator().apply(mTalonConfig);

    mTalonType = type;

    mVoltsSignal = mServo.getMotorVoltage(true);
    mSupplyAmpsSignal = mServo.getSupplyCurrent(true);
    mStatorAmpsSignal = mServo.getStatorCurrent(true);
    mTorqueAmpsSignal = mServo.getTorqueCurrent(true);
    mCelsiusSignal = mServo.getDeviceTemp(true);
    mRevsSignal = mServo.getPosition(true);
    mRevsPerSecSignal = mServo.getVelocity(true);
    mRevsPerSecPerSecSignal = mServo.getAcceleration(true);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        mVoltsSignal,
        mSupplyAmpsSignal,
        mStatorAmpsSignal,
        mTorqueAmpsSignal,
        mCelsiusSignal,
        mRevsSignal,
        mRevsPerSecSignal,
        mRevsPerSecPerSecSignal);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        mVoltsSignal,
        mSupplyAmpsSignal,
        mStatorAmpsSignal,
        mTorqueAmpsSignal,
        mCelsiusSignal,
        mRevsSignal,
        mRevsPerSecSignal,
        mRevsPerSecPerSecSignal);
  }

  @Override
  public double getMotorKtNewtonMetersPerAmps() {
    if (mTalonType == MotorType.KrakenX44) return DCMotor.getKrakenX44Foc(1).KtNMPerAmp;
    else return DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
  }

  @Override
  public double getAppliedVolts() {
    return mVoltsSignal.getValueAsDouble();
  }

  @Override
  public double getSupplyCurrentAmps() {
    return mSupplyAmpsSignal.getValueAsDouble();
  }

  @Override
  public double getStatorCurrentAmps() {
    return mStatorAmpsSignal.getValueAsDouble();
  }

  @Override
  public double getTorqueCurrentAmps() {
    return mTorqueAmpsSignal.getValueAsDouble();
  }

  @Override
  public double getTemperatureCelsius() {
    return mCelsiusSignal.getValueAsDouble();
  }

  @Override
  public double getAngularPositionRevs() {
    return mRevsSignal.getValueAsDouble();
  }

  @Override
  public double getAngularVelocityRevsPerSec() {
    return mRevsPerSecSignal.getValueAsDouble();
  }

  @Override
  public double getAngularAccelerationRevsPerSecPerSec() {
    return mRevsPerSecPerSecSignal.getValueAsDouble();
  }

  @Override
  public void setLeader(ServoIO leader, boolean flipped) {
    mServo.setControl(
        new Follower(
            leader.mCanDeviceId,
            (flipped) ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  @Override
  public void setBrakeModeStatus(boolean brakeEnabled) {
    if ((brakeEnabled && mTalonConfig.MotorOutput.NeutralMode == NeutralModeValue.Brake)
        || (!brakeEnabled && mTalonConfig.MotorOutput.NeutralMode == NeutralModeValue.Coast)) {
      return;
    }

    new Thread(
            () -> {
              mTalonConfig.MotorOutput.NeutralMode =
                  brakeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

              PhoneixUtils.tryUntilOk(5, () -> mServo.getConfigurator().apply(mTalonConfig));
            })
        .start();
  }

  @Override
  public void resetRelativeEncoder(double positionRevs) {
    mServo.setPosition(positionRevs);
  }

  @Override
  public void setNeutral() {
    mServo.stopMotor();
    ;
  }

  @Override
  public void setTorqueCurrentOutput(double torqueNewtonMeters) {
    mServo.setControl(mTorqueCurrentRequest.withOutput(torqueNewtonMeters));
  }

  @Override
  public void setPositionSetpoint(double positionRevs) {
    mServo.setControl(mPositionRequest.withPosition(positionRevs));
  }

  @Override
  public void setPositionSetpoint(double positionRevs, double feedforward) {
    mServo.setControl(mPositionRequest.withPosition(positionRevs).withFeedForward(feedforward));
  }

  @Override
  public void setVelocitySetpoint(double velocityRevsPerSec) {
    mServo.setControl(mVelocityRequest.withVelocity(velocityRevsPerSec));
  }

  @Override
  public void setVelocitySetpoint(double velocityRevsPerSec, double feedforward) {
    mServo.setControl(
        mVelocityRequest.withVelocity(velocityRevsPerSec).withFeedForward(feedforward));
  }

  @Override
  public void setProfiledPositionSetpoint(double positionRevs) {
    mServo.setControl(mProfiledPositionRequest.withPosition(positionRevs));
  }

  @Override
  public void setProfiledPositionSetpoint(double positionRevs, double feedforward) {
    mServo.setControl(
        mProfiledPositionRequest.withPosition(positionRevs).withFeedForward(feedforward));
  }

  @Override
  public void setProfiledVelocitySetpoint(double velocityRevsPerSec) {
    mServo.setControl(mProfiledVelocityRequest.withVelocity(velocityRevsPerSec));
  }

  @Override
  public void setProfiledVelocitySetpoint(double velocityRevsPerSec, double feedforward) {
    mServo.setControl(
        mProfiledVelocityRequest.withVelocity(velocityRevsPerSec).withFeedForward(feedforward));
  }
}
