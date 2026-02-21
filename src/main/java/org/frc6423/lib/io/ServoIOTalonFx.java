// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.frc6423.lib.util.PhoneixUtils;

/**
 * {@link ServoIO} extension for a TalonFX based servo
 *
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/index.html
 */
public class ServoIOTalonFx extends ServoIO {
  protected final TalonFX mServo;
  protected final TalonFXConfiguration mTalonConfig;

  protected final StatusSignal<Voltage> mVoltSignal;
  protected final StatusSignal<Current> mSupplySignal, mStatorSignal, mTorqueSignal;
  protected final StatusSignal<Angle> mAngleSignal;
  protected final StatusSignal<AngularVelocity> mVelocitySignal;
  protected final StatusSignal<AngularAcceleration> mAccelerationSignal;
  protected final StatusSignal<Temperature> mTemperatureSignal;

  protected final VoltageOut mVoltRequest = new VoltageOut(0.0);
  protected final TorqueCurrentFOC mTorqueRequest = new TorqueCurrentFOC(0.0);

  protected final PositionVoltage mVoltPoseRequest = new PositionVoltage(0.0);
  protected final PositionTorqueCurrentFOC mTorquePoseRequest = new PositionTorqueCurrentFOC(0.0);

  protected final VelocityVoltage mVoltVelRequest = new VelocityVoltage(0.0);
  protected final VelocityTorqueCurrentFOC mTorqueVelRequest = new VelocityTorqueCurrentFOC(0.0);

  protected final MotionMagicVoltage mVoltProfiledPoseRequest = new MotionMagicVoltage(0.0);
  protected final MotionMagicTorqueCurrentFOC mTorqueProfiledPoseRequest =
      new MotionMagicTorqueCurrentFOC(0.0);

  protected final MotionMagicVelocityVoltage mVoltProfiledVelRequest =
      new MotionMagicVelocityVoltage(0.0);
  protected final MotionMagicVelocityTorqueCurrentFOC mTorqueProfiledVelRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  /**
   * Create new {@link ServoIOTalonFx}
   *
   * @param name {@link String} representing servo nickname
   * @param canBus {@link CANBus} representing CAN bus loop device is in
   * @param canDeviceId {@link Integer} representing the id of CAN device
   * @param talonConfig {@link TalonFXConfiguration} representing the servo config
   * @param motorKt {@link Double} representing the servo's kT rating
   */
  public ServoIOTalonFx(
      String name, CANBus canBus, int deviceId, TalonFXConfiguration talonConfig) {
    super(name, canBus, deviceId, talonConfig, DCMotor.getKrakenX60Foc(1).KtNMPerAmp);

    mServo = new TalonFX(deviceId, canBus);
    mTalonConfig = talonConfig;
    mServo.getConfigurator().apply(mTalonConfig);

    mVoltSignal = mServo.getMotorVoltage();

    mSupplySignal = mServo.getSupplyCurrent();
    mStatorSignal = mServo.getStatorCurrent();
    mTorqueSignal = mServo.getTorqueCurrent();

    mAngleSignal = mServo.getPosition();

    mVelocitySignal = mServo.getVelocity();
    mAccelerationSignal = mServo.getAcceleration();

    mTemperatureSignal = mServo.getDeviceTemp();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        mVoltSignal,
        mSupplySignal,
        mStatorSignal,
        mTorqueSignal,
        mAngleSignal,
        mVelocitySignal,
        mAccelerationSignal,
        mTemperatureSignal);
  }

  @Override
  public Voltage getAppliedVoltage() {
    return mVoltSignal.getValue();
  }

  @Override
  public Current getSupplyCurrent() {
    return mSupplySignal.getValue();
  }

  @Override
  public Current getStatorCurrent() {
    return mStatorSignal.getValue();
  }

  @Override
  public Current getTorqueCurrent() {
    return mTorqueSignal.getValue();
  }

  @Override
  public Angle getAngle() {
    return mAngleSignal.getValue();
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
            leader.mDeviceId, flipped ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  @Override
  public void setBrakeStatus(boolean active) {
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
    mServo.setControl(mVoltRequest.withOutput(voltage).withEnableFOC(withFoc));
  }

  @Override
  public void setTorqueCurrentSetpoint(Current current) {
    mServo.setControl(mTorqueRequest.withOutput(current));
  }

  @Override
  public void setVoltagePositionSetpoint(Angle angle, Voltage feedforward, boolean withFoc) {
    mServo.setControl(
        mVoltPoseRequest.withPosition(angle).withFeedForward(feedforward).withEnableFOC(withFoc));
  }

  @Override
  public void setTorquePositionSetpoint(Angle angle, Current feedforward) {
    mServo.setControl(mTorquePoseRequest.withPosition(angle).withFeedForward(feedforward));
  }

  @Override
  public void setVoltageVelocitySetpoint(
      AngularVelocity velocity, Voltage feedforward, boolean withFoc) {
    mServo.setControl(
        mVoltVelRequest.withVelocity(velocity).withFeedForward(feedforward).withEnableFOC(withFoc));
  }

  @Override
  public void setTorqueVelocitySetpoint(AngularVelocity velocity, Current feedforward) {
    mServo.setControl(mTorqueVelRequest.withVelocity(velocity).withFeedForward(feedforward));
  }

  @Override
  public void setTorqueVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration) {
    mServo.setControl(mTorqueVelRequest.withVelocity(velocity).withAcceleration(acceleration));
  }

  @Override
  public void setVoltageMotionProfiledPositionSetpoint(
      Angle angle, Voltage feedforward, boolean withFoc) {
    mServo.setControl(
        mVoltProfiledPoseRequest
            .withPosition(angle)
            .withFeedForward(feedforward)
            .withEnableFOC(withFoc));
  }

  @Override
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle, Current feedforward) {
    mServo.setControl(mTorqueProfiledPoseRequest.withPosition(angle).withFeedForward(feedforward));
  }

  @Override
  public void setVoltageMotionProfiledVelocitySetpoint(AngularVelocity velocity, boolean withFoc) {
    mVoltProfiledVelRequest.withVelocity(velocity).withEnableFOC(withFoc);
  }

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity) {
    mTorqueProfiledVelRequest.withVelocity(velocity);
  }

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, Current feedforward) {
    mTorqueProfiledVelRequest.withVelocity(velocity).withFeedForward(feedforward);
  }

  @Override
  public void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration) {
    mTorqueProfiledVelRequest.withVelocity(velocity).withAcceleration(acceleration);
  }
}
