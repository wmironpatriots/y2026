// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/**
 * Implementation of {@link ServoIO} for TalonFX Servos
 *
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/index.html
 */
public class ServoIOTalonFx extends ServoIO {
  // TODO give this class a cleanup :(
  protected final TalonFX mServo;
  protected final TalonFXConfiguration mConfig;

  private final BaseStatusSignal voltageSignal,
      inCurrentSignal,
      outCurrentSignal,
      torqueCurrentSignal,
      angleSignal,
      velocitySignal,
      accelerationSignal,
      temperatureSignal;

  private final VoltageOut mVoltReq = new VoltageOut(0.0).withEnableFOC(false);
  private final TorqueCurrentFOC mTauReq = new TorqueCurrentFOC(0.0);
  private final DutyCycleOut mDutycycleReq = new DutyCycleOut(0.0).withEnableFOC(false);
  private final PositionTorqueCurrentFOC mFocPoseReq = new PositionTorqueCurrentFOC(0.0);
  private final PositionVoltage mPoseReq = new PositionVoltage(0.0).withEnableFOC(false);
  private final VelocityTorqueCurrentFOC mFocVelReq = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityVoltage mVelReq = new VelocityVoltage(0.0).withEnableFOC(false);
  private final MotionMagicTorqueCurrentFOC mFocProfPoseReq = new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage mProfPoseReq = new MotionMagicVoltage(0.0).withEnableFOC(false);
  private final MotionMagicVelocityTorqueCurrentFOC mFocProfVelReq =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);
  private final MotionMagicVelocityVoltage mProfVelReq =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(false);

  private boolean mFocEnabled = true;

  private final BlockingQueue<Runnable> mQueue = new LinkedBlockingQueue<>();
  private final ThreadPoolExecutor mThreadPoolExe =
      new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, mQueue);

  /**
   * Create new {@link ServoIOTalonFx}
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer ID on CAN loop
   * @param canBusId {@link CANBus} representing the CAN loop device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   */
  public ServoIOTalonFx(String name, int canDeviceId, CANBus canBus, TalonFXConfiguration config) {
    super(name, canDeviceId);

    this.mServo = new TalonFX(canDeviceId, canBus);
    this.mConfig = config;

    this.voltageSignal = mServo.getMotorVoltage(true);
    this.inCurrentSignal = mServo.getSupplyCurrent(true);
    this.outCurrentSignal = mServo.getStatorCurrent(true);
    this.torqueCurrentSignal = mServo.getTorqueCurrent(true);

    this.angleSignal = mServo.getPosition(true);
    this.velocitySignal = mServo.getVelocity(true);
    this.accelerationSignal = mServo.getAcceleration(true);
    this.temperatureSignal = mServo.getDeviceTemp(true);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        voltageSignal,
        inCurrentSignal,
        outCurrentSignal,
        torqueCurrentSignal,
        angleSignal,
        velocitySignal,
        accelerationSignal,
        temperatureSignal);
  }

  /**
   * Apply new {@link TalonFXConfiguration}
   *
   * @param configApplier {@link UnaryOperator} that modifies a {@link TalonFXConfiguration}
   */
  protected void applyConfig(UnaryOperator<TalonFXConfiguration> configApplier) {
    mThreadPoolExe.submit(
        () -> {
          var newConfig = configApplier.apply(mConfig);

          if (newConfig != mConfig) {
            for (int i = 0; i < 5; i++) {
              StatusCode result = mServo.getConfigurator().apply(configApplier.apply(mConfig));
              if (result.isOK()) {
                break;
              }
            }
          }
        });
  }

  @Override
  public Voltage getAppliedVoltage() {
    return Volts.of(voltageSignal.getValueAsDouble());
  }

  @Override
  public Current getSupplyCurrent() {
    return Amps.of(inCurrentSignal.getValueAsDouble());
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(outCurrentSignal.getValueAsDouble());
  }

  @Override
  public Current getTorqueCurrent() {
    return Amps.of(torqueCurrentSignal.getValueAsDouble());
  }

  @Override
  public Angle getAngle() {
    return Revolutions.of(angleSignal.getValueAsDouble());
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return RevolutionsPerSecond.of(velocitySignal.getValueAsDouble());
  }

  @Override
  public AngularAcceleration getAngularAcceleration() {
    return RadiansPerSecondPerSecond.of(2 * Math.PI * accelerationSignal.getValueAsDouble());
  }

  @Override
  public Temperature getTemperature() {
    return Celsius.of(temperatureSignal.getValueAsDouble());
  }

  @Override
  public void setBrakeStatus(boolean active) {
    UnaryOperator<TalonFXConfiguration> configApplier =
        (TalonFXConfiguration config) -> {
          config.MotorOutput.NeutralMode = active ? NeutralModeValue.Brake : NeutralModeValue.Coast;

          return config;
        };

    applyConfig(configApplier);
  }

  @Override
  public void setFocStatus(boolean active) {
    mFocEnabled = true;
  }

  @Override
  public void setLeader(ServoIO leader, boolean flipped) {
    mServo.setControl(
        new Follower(
            leader.mCanDeviceId,
            (flipped) ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  @Override
  public void stop() {
    mServo.stopMotor();
  }

  @Override
  protected void setVoltageSetpoint(Voltage voltage) {
    mServo.setControl(mVoltReq.withOutput(voltage).withEnableFOC(mFocEnabled));
  }

  @Override
  protected void setTorqueCurrentSetpoint(Current current) {
    mServo.setControl(mTauReq.withOutput(current));
  }

  @Override
  protected void setDutycycleSetpoint(double dutycycle) {
    mServo.setControl(mDutycycleReq.withOutput(dutycycle).withEnableFOC(mFocEnabled));
  }

  @Override
  protected void setPositionSetpoint(Angle position, int slot) {
    if (mFocEnabled) {
      mFocPoseReq.withPosition(position).withSlot(slot);
      return;
    }

    mPoseReq.withPosition(position).withSlot(slot);
  }

  @Override
  protected void setVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
    if (mFocEnabled) {
      mFocVelReq.withVelocity(velocity);
      return;
    }

    mVelReq.withVelocity(velocity).withAcceleration(acceleration).withSlot(slot);
  }

  @Override
  protected void setPositionProfiledSetpoint(Angle position, int slot) {
    if (mFocEnabled) {
      mFocProfPoseReq.withPosition(position).withSlot(slot);
      return;
    }

    mProfPoseReq.withPosition(position).withSlot(slot);
  }

  @Override
  protected void setVelocityProfiledSetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
    if (mFocEnabled) {
      mFocProfVelReq.withVelocity(velocity).withAcceleration(acceleration).withSlot(slot);
      return;
    }

    mProfVelReq.withVelocity(velocity).withAcceleration(acceleration).withSlot(slot);
  }
}
