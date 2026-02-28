// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.NewtonMeters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import org.frc6423.lib.util.PhoneixUtils;

// TODO Javadoc
public class ServoIOTalonFx extends ServoIO {
  protected final TalonFX mServo;

  public final StatusSignal<Voltage> mSupplyVoltageSignal, mStatorVoltageSignal;
  public final StatusSignal<Current> mSupplyCurrentSignal,
      mStatorCurrentSignal,
      mTorqueCurrentSignal;
  public final StatusSignal<Angle> mPositionSignal;
  public final StatusSignal<AngularVelocity> mVelocitySignal;
  public final StatusSignal<AngularAcceleration> mAccelerationSignal;
  public final StatusSignal<Temperature> mTemperatureSignal;

  protected final VoltageOut mVoltageOut = new VoltageOut(0.0);
  protected final TorqueCurrentFOC mTorqueCurrentOut = new TorqueCurrentFOC(0.0);

  protected final PositionVoltage mVoltagePositionOut = new PositionVoltage(0.0);
  protected final PositionTorqueCurrentFOC mTorquePositionOut = new PositionTorqueCurrentFOC(0.0);

  protected final VelocityVoltage mVoltageVelocityOut = new VelocityVoltage(0.0);
  protected final VelocityTorqueCurrentFOC mTorqueVelocityOut = new VelocityTorqueCurrentFOC(0.0);

  protected final MotionMagicVoltage mVoltageMotionMagicOut = new MotionMagicVoltage(0.0);
  protected final MotionMagicTorqueCurrentFOC mTorqueMotionMagicOut =
      new MotionMagicTorqueCurrentFOC(0.0);

  protected final MotionMagicVelocityVoltage mVoltageMotionMagicVelOut =
      new MotionMagicVelocityVoltage(0.0);
  protected final MotionMagicVelocityTorqueCurrentFOC mTorqueMotionMagicVelOut =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private int mSlot = 0;

  public ServoIOTalonFx(
      String name, CANBus canBus, int deviceId, TalonFXConfiguration talonConfig) {
    super(name, canBus, deviceId, talonConfig);

    mServo = new TalonFX(deviceId, canBus);
    mServo.getConfigurator().apply(mTalonConfig);

    mSupplyVoltageSignal = mServo.getSupplyVoltage();
    mStatorVoltageSignal = mServo.getMotorVoltage();

    mSupplyCurrentSignal = mServo.getSupplyCurrent();
    mStatorCurrentSignal = mServo.getStatorCurrent();
    mTorqueCurrentSignal = mServo.getTorqueCurrent();

    mPositionSignal = mServo.getPosition();

    mVelocitySignal = mServo.getVelocity();
    mAccelerationSignal = mServo.getAcceleration();

    mTemperatureSignal = mServo.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        mSupplyVoltageSignal,
        mStatorVoltageSignal,
        mSupplyCurrentSignal,
        mStatorCurrentSignal,
        mTorqueCurrentSignal,
        mPositionSignal,
        mVelocitySignal,
        mAccelerationSignal,
        mTemperatureSignal);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        mSupplyVoltageSignal,
        mStatorVoltageSignal,
        mSupplyCurrentSignal,
        mStatorCurrentSignal,
        mTorqueCurrentSignal,
        mPositionSignal,
        mVelocitySignal,
        mAccelerationSignal,
        mTemperatureSignal);
  }

  @Override
  public Per<TorqueUnit, CurrentUnit> getMotorKt() {
    return mServo.getMotorKT().getValue();
  }

  @Override
  public Voltage getSupplyVoltage() {
    return mSupplyVoltageSignal.getValue();
  }

  @Override
  public Voltage getStatorVoltage() {
    return mStatorVoltageSignal.getValue();
  }

  @Override
  public Current getSupplyCurrent() {
    return mSupplyCurrentSignal.getValue();
  }

  @Override
  public Current getStatorCurrent() {
    return mStatorCurrentSignal.getValue();
  }

  @Override
  public Current getTorqueCurrent() {
    return mTorqueCurrentSignal.getValue();
  }

  @Override
  public Angle getAngle() {
    return mPositionSignal.getValue();
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return mVelocitySignal.getValue();
  }

  @Override
  public AngularAcceleration getAngularAcceleration() {
    return mAccelerationSignal.getValue();
  }

  @Override
  public Temperature getTemperature() {
    return mTemperatureSignal.getValue();
  }

  @Override
  public void setLeader(ServoIO leader, boolean flipped) {
    mServo.setControl(
        new Follower(
            leader.mCanDeviceId,
            flipped ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  @Override
  public void setGainsSlot(int slot) {
    mSlot = slot;
  }

  @Override
  public void setBrakeStatus(boolean active) {
    if ((active && mTalonConfig.MotorOutput.NeutralMode == NeutralModeValue.Brake)
        || (!active && mTalonConfig.MotorOutput.NeutralMode == NeutralModeValue.Coast)) {
      return;
    }

    new Thread(
            () -> {
              mTalonConfig.MotorOutput.NeutralMode =
                  active ? NeutralModeValue.Brake : NeutralModeValue.Coast;

              PhoneixUtils.tryUntilOk(5, () -> mServo.getConfigurator().apply(mTalonConfig));
            })
        .start();
    ;
  }

  @Override
  public void resetEncoder(Angle angle) {
    mServo.setPosition(angle);
  }

  @Override
  public void stop() {
    mServo.stopMotor();
  }

  @Override
  public void setVoltageSetpoint(Voltage voltage, boolean withFoc) {
    mServo.setControl(mVoltageOut.withOutput(voltage).withEnableFOC(withFoc));
  }

  @Override
  public void setVoltagePositionSetpoint(Angle angle, boolean withFoc) {
    mServo.setControl(
        mVoltagePositionOut.withPosition(angle).withEnableFOC(withFoc).withSlot(mSlot));
  }

  @Override
  public void setVoltagePositionSetpoint(Angle angle, Voltage feedforward, boolean withFoc) {
    mServo.setControl(
        mVoltagePositionOut
            .withPosition(angle)
            .withFeedForward(feedforward)
            .withEnableFOC(withFoc)
            .withSlot(mSlot));
  }

  @Override
  public void setVoltageVelocitySetpoint(AngularVelocity velocity, boolean withFoc) {
    mServo.setControl(
        mVoltageVelocityOut.withVelocity(velocity).withEnableFOC(withFoc).withSlot(mSlot));
  }

  @Override
  public void setVoltageVelocitySetpoint(
      AngularVelocity velocity, Voltage feedforward, boolean withFoc) {
    mServo.setControl(
        mVoltageVelocityOut
            .withVelocity(velocity)
            .withAcceleration(getAngularAcceleration())
            .withFeedForward(feedforward)
            .withEnableFOC(withFoc)
            .withSlot(mSlot));
  }

  @Override
  public void setVoltageMotionProfiledPositionSetpoint(Angle angle, boolean withFoc) {
    mServo.setControl(
        mVoltageMotionMagicOut.withPosition(angle).withEnableFOC(withFoc).withSlot(mSlot));
  }

  @Override
  public void setVoltageMotionProfiledPositionSetpoint(
      Angle angle, Voltage feedforward, boolean withFoc) {
    mServo.setControl(
        mVoltageMotionMagicOut
            .withPosition(angle)
            .withFeedForward(feedforward)
            .withEnableFOC(withFoc)
            .withSlot(mSlot));
  }

  @Override
  public void setVoltageMotionProfiledVelocitySetpoint(AngularVelocity velocity, boolean withFoc) {
    mServo.setControl(
        mVoltageMotionMagicVelOut.withVelocity(velocity).withEnableFOC(withFoc).withSlot(mSlot));
  }

  @Override
  public void setTorqueCurrentSetpoint(Current current) {
    mServo.setControl(mTorqueCurrentOut.withOutput(current));
  }

  @Override
  public void setTorquePositionSetpoint(Angle angle) {
    mServo.setControl(mTorquePositionOut.withPosition(angle).withSlot(mSlot));
  }

  @Override
  public void setTorquePositionSetpoint(Angle angle, Torque torque) {
    mServo.setControl(
        mTorquePositionOut
            .withPosition(angle)
            .withFeedForward(getSystemKt().in(NewtonMeters.per(Amps)) / torque.in(NewtonMeters))
            .withSlot(mSlot));
  }

  @Override
  public void setTorquePositionSetpoint(Angle angle, Current feedforward) {
    mServo.setControl(
        mTorquePositionOut.withPosition(angle).withFeedForward(feedforward).withSlot(mSlot));
  }

  @Override
  public void setTorqueVelocitySetpoint(AngularVelocity velocity) {
    mServo.setControl(mTorqueVelocityOut.withVelocity(velocity).withSlot(mSlot));
  }

  @Override
  public void setTorqueVelocitySetpoint(AngularVelocity velocity, Torque torque) {
    mServo.setControl(
        mTorqueVelocityOut
            .withVelocity(velocity)
            .withFeedForward(getSystemKt().in(NewtonMeters.per(Amps)) / torque.in(NewtonMeters))
            .withSlot(mSlot));
  }

  @Override
  public void setTorqueVelocitySetpoint(AngularVelocity velocity, Current feedforward) {
    mServo.setControl(
        mTorqueVelocityOut.withVelocity(velocity).withFeedForward(feedforward).withSlot(mSlot));
  }

  @Override
  public void setTorqueVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration) {
    mServo.setControl(
        mTorqueVelocityOut.withVelocity(velocity).withAcceleration(acceleration).withSlot(mSlot));
  }

  @Override
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle) {
    mServo.setControl(mTorqueMotionMagicOut.withPosition(angle).withSlot(mSlot));
  }

  @Override
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle, Torque torque) {
    mServo.setControl(
        mTorqueMotionMagicOut
            .withPosition(angle)
            .withFeedForward(getSystemKt().in(NewtonMeters.per(Amps)) / torque.in(NewtonMeters))
            .withSlot(mSlot));
  }

  @Override
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle, Current feedforward) {
    mServo.setControl(
        mTorqueMotionMagicOut.withPosition(angle).withFeedForward(feedforward).withSlot(mSlot));
  }

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity) {
    mServo.setControl(mTorqueMotionMagicVelOut.withVelocity(velocity).withSlot(mSlot));
  }

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity, Torque torque) {
    mServo.setControl(
        mTorqueMotionMagicVelOut
            .withVelocity(velocity)
            .withFeedForward(getSystemKt().in(NewtonMeters.per(Amps)) / torque.in(NewtonMeters))
            .withSlot(mSlot));
  }

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, Current feedforward) {
    mServo.setControl(
        mTorqueMotionMagicVelOut
            .withVelocity(velocity)
            .withFeedForward(feedforward)
            .withSlot(mSlot));
  }

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration) {
    mServo.setControl(
        mTorqueMotionMagicVelOut
            .withVelocity(velocity)
            .withAcceleration(acceleration)
            .withSlot(mSlot));
  }
}
