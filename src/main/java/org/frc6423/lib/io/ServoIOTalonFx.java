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
  private final TalonFX servo;
  private final TalonFXConfiguration config;

  private final BaseStatusSignal voltageSignal,
      inCurrentSignal,
      outCurrentSignal,
      torqueCurrentSignal,
      angleSignal,
      velocitySignal,
      accelerationSignal,
      temperatureSignal;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(false);
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0.0);
  private final DutyCycleOut dutycycleRequest = new DutyCycleOut(0.0).withEnableFOC(false);
  private final PositionTorqueCurrentFOC focPositionRequest = new PositionTorqueCurrentFOC(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);
  private final VelocityTorqueCurrentFOC focVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withEnableFOC(false);
  private final MotionMagicTorqueCurrentFOC focProfiledPositionRequest =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage profiledPositionRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(false);
  private final MotionMagicVelocityTorqueCurrentFOC focProfiledVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);
  private final MotionMagicVelocityVoltage profiledVelocityRequest =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(false);

  private boolean focEnabled = true;

  private final BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
  private final ThreadPoolExecutor threadPoolExecutor =
      new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

  /**
   * Create new {@link ServoIOTalonFx}
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer representing the CAN identification
   * @param canBusId {@link CANBus} representing the CAN bus device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   * @param temperature {@link Temperature} representing the maximum temperature a servo can be at
   *     before shutting down
   */
  public ServoIOTalonFx(
      String name,
      int canDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      Temperature temperature) {
    super(name, temperature);

    this.servo = new TalonFX(canDeviceId, canBus);
    this.config = config;

    this.voltageSignal = servo.getMotorVoltage(true);
    this.inCurrentSignal = servo.getSupplyCurrent(true);
    this.outCurrentSignal = servo.getStatorCurrent(true);
    this.torqueCurrentSignal = servo.getTorqueCurrent(true);

    this.angleSignal = servo.getPosition(true);
    this.velocitySignal = servo.getVelocity(true);
    this.accelerationSignal = servo.getAcceleration(true);
    this.temperatureSignal = servo.getDeviceTemp(true);
  }

  /**
   * Apply new {@link TalonFXConfiguration}
   *
   * @param configApplier {@link UnaryOperator} that modifies a {@link TalonFXConfiguration}
   */
  public void applyConfig(UnaryOperator<TalonFXConfiguration> configApplier) {
    threadPoolExecutor.submit(
        () -> {
          for (int i = 0; i < 5; i++) {
            StatusCode result = servo.getConfigurator().apply(configApplier.apply(config));
            if (result.isOK()) {
              break;
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
  public void enableBrake() {
    UnaryOperator<TalonFXConfiguration> configApplier =
        (TalonFXConfiguration config) -> {
          config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

          return config;
        };

    applyConfig(configApplier);
  }

  @Override
  public void disableBrake() {
    UnaryOperator<TalonFXConfiguration> configApplier =
        (TalonFXConfiguration config) -> {
          config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

          return config;
        };

    applyConfig(configApplier);
  }

  /**
   * Enable Field-Oriented Control (FOC)
   *
   * <p><strong>WARNING</strong>: FOC control calculates a Torque Current system input (in Amps).
   * This means gains that work with voltage based control will never work with FOC based control
   *
   * @see
   *     https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html#field-oriented-control
   */
  public void enableFoc() {
    focEnabled = true;
  }

  /**
   * Disable Field-Oriented Control (FOC)
   *
   * <p><strong>WARNING</strong>: Disabling FOC will use voltage based control for all controll
   * methods and will produce a Voltage system input (in Volts). This means gains that work with FOC
   * based control will never work with voltage based control
   *
   * @see
   *     https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html#field-oriented-control
   */
  public void disableFoc() {
    focEnabled = false;
  }

  @Override
  protected void idle() {
    servo.stopMotor();
  }

  @Override
  protected void setVoltageSetpoint(Voltage voltage) {
    servo.setControl(voltageRequest.withOutput(voltage).withEnableFOC(focEnabled));
  }

  @Override
  protected void setTorqueCurrentSetpoint(Current current) {
    servo.setControl(torqueRequest.withOutput(current));
  }

  @Override
  protected void setDutycycleSetpoint(double dutycycle) {
    servo.setControl(dutycycleRequest.withOutput(dutycycle).withEnableFOC(focEnabled));
  }

  @Override
  protected void setPositionSetpoint(Angle position, int slot) {
    if (focEnabled) {
      focPositionRequest.withPosition(position).withSlot(slot);
      return;
    }

    positionRequest.withPosition(position).withSlot(slot);
  }

  @Override
  protected void setVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
    if (focEnabled) {
      focVelocityRequest.withVelocity(velocity);
      return;
    }

    velocityRequest.withVelocity(velocity).withAcceleration(acceleration).withSlot(slot);
  }

  @Override
  protected void setPositionProfiledSetpoint(Angle position, int slot) {
    if (focEnabled) {
      focProfiledPositionRequest.withPosition(position).withSlot(slot);
      return;
    }

    profiledPositionRequest.withPosition(position).withSlot(slot);
  }

  @Override
  protected void setVelocityProfiledSetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
    if (focEnabled) {
      focProfiledVelocityRequest
          .withVelocity(velocity)
          .withAcceleration(acceleration)
          .withSlot(slot);
      return;
    }

    profiledVelocityRequest.withVelocity(velocity).withAcceleration(acceleration).withSlot(slot);
  }
}
