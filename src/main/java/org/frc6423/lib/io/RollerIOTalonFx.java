// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

/**
 * {@link RollerIO} extension for {@link TalonFX} Servos
 *
 * <p>@see https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/index.html
 */
public class RollerIOTalonFx extends RollerIO {
  // * ~~~~~~~~ CONSTANTS ~~~~~~~~
  /**
   * Create a generic {@link TalonFXConfiguration} for a roller servo
   *
   * @param counterClockwisePositive {@link Boolean} Whether counterclockwise should be considered
   *     the positive direction or not
   * @return {@link TalonFXConfiguration}
   */
  public static TalonFXConfiguration createGenericRollerConfig(boolean counterClockwisePositive) {
    return new TalonFXConfiguration()
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnBoot(true))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(
                    (counterClockwisePositive
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive)))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true));
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private final TalonFX mServo;
  private final TalonFXConfiguration mConfig;

  private final BaseStatusSignal mMeasuredVoltage, mMeasuredPosition, mMeasuredVelocity;

  private final VoltageOut mVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private boolean mIsConnected = false;

  public RollerIOTalonFx(int deviceId, CANBus canBus, TalonFXConfiguration config) {
    mServo = new TalonFX(deviceId, canBus);
    mConfig = config;

    mIsConnected = mServo.getConfigurator().apply(config).isOK();

    mMeasuredVoltage = mServo.getMotorVoltage();
    mMeasuredPosition = mServo.getPosition();
    mMeasuredVelocity = mServo.getVelocity();
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  @Override
  public void periodic() {
    mIsConnected =
        BaseStatusSignal.refreshAll(mMeasuredVoltage, mMeasuredPosition, mMeasuredVelocity).isOK();
  }

  @Override
  public boolean isConnected() {
    return mIsConnected;
  }

  @Override
  public double getAppliedVoltage() {
    return mMeasuredVoltage.getValueAsDouble();
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  @Override
  public void setVoltageOutput(double volts) {
    mServo.setControl(mVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void stop() {
    mServo.stopMotor();
  }
}
