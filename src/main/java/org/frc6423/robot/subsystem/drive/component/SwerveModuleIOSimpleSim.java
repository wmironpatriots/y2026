// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

public class SwerveModuleIOSimpleSim extends SwerveModuleIO {
  private final DCMotorSim mPivotModel, mDriveModel;

  private final SimpleMotorFeedforward mDriveFf = new SimpleMotorFeedforward(5.0, 0.0, 0.0, 0.02);
  private final PIDController mPivotController = new PIDController(10.0, 0, 0, 0.02);
  private final PIDController mDriveController = new PIDController(0.01, 0, 0, 0.02);

  private double mPivotAppliedOutput, mDriveAppliedOutput;

  public SwerveModuleIOSimpleSim(String name, ModuleConfig config, DriveConstants constants) {
    super(name, config, constants);

    mPivotController.enableContinuousInput(-Math.PI, Math.PI);

    mPivotModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                0.004,
                mConstants.getPivotSensorToMechanismRatio()
                    * mConstants.getPivotRotorToSensorRatio()),
            DCMotor.getKrakenX60Foc(1));

    mDriveModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.025, mConstants.getDriveRotorToMechRatio()),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void periodic() {
    mPivotAppliedOutput = mPivotController.calculate(getPivotAngle().in(Radians));
    mDriveAppliedOutput =
        mDriveFf.calculate(getDriveAngularVelocity().in(RadiansPerSecond))
            + mDriveController.calculate(getDriveAngularVelocity().in(RadiansPerSecond));

    mPivotModel.setInputVoltage(MathUtil.clamp(mPivotAppliedOutput, -12.0, 12.0));
    mDriveModel.setInputVoltage(MathUtil.clamp(mDriveAppliedOutput, -12.0, 12.0));
    mPivotModel.update(0.02);
    mDriveModel.update(0.02);
  }

  @Override
  public Voltage getPivotSupplyVoltage() {
    return Volts.zero();
  }

  @Override
  public Voltage getPivotStatorVoltage() {
    return Volts.of(mPivotAppliedOutput);
  }

  @Override
  public Current getPivotSupplyCurrent() {
    return Amps.of(mPivotModel.getCurrentDrawAmps());
  }

  @Override
  public Current getPivotStatorCurrent() {
    return Amps.zero();
  }

  @Override
  public Current getPivotTorqueCurrent() {
    return Amps.zero();
  }

  @Override
  public Temperature getPivotTemperature() {
    return Celsius.of(0.0);
  }

  @Override
  public Angle getPivotAngle() {
    return Revolutions.of(mPivotModel.getAngularPosition().in(Revolutions));
  }

  @Override
  public void setPivotVoltage(Voltage voltage) {
    mPivotModel.setInputVoltage(voltage.in(Volts));
  }

  @Override
  public void setPivotCurrent(Current current) {}

  @Override
  protected void setPivotSetpoint(Angle setpoint) {
    mPivotController.setSetpoint(setpoint.in(Radians));
  }

  @Override
  public Voltage getDriveSupplyVoltage() {
    return Volts.zero();
  }

  @Override
  public Voltage getDriveStatorVoltage() {
    return Volts.of(mDriveAppliedOutput);
  }

  @Override
  public Current getDriveSupplyCurrent() {
    return Amps.of(mDriveModel.getCurrentDrawAmps());
  }

  @Override
  public Current getDriveStatorCurrent() {
    return Amps.zero();
  }

  @Override
  public Current getDriveTorqueCurrent() {
    return Amps.zero();
  }

  @Override
  public Temperature getDriveTemperature() {
    return Celsius.zero();
  }

  @Override
  public Angle getDriveAngle() {
    return mDriveModel.getAngularPosition();
  }

  @Override
  public AngularVelocity getDriveAngularVelocity() {
    return mDriveModel.getAngularVelocity();
  }

  @Override
  public AngularAcceleration getDriveAngularAcceleration() {
    return mDriveModel.getAngularAcceleration();
  }

  @Override
  public void setDriveVoltage(Voltage voltage) {
    mDriveModel.setInputVoltage(voltage.in(Volts));
  }

  @Override
  public void setDriveCurrent(Current current) {}

  @Override
  protected void setDriveSetpoint(AngularVelocity velocity, boolean focEnabled) {
    mDriveController.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  protected void setDriveSetpoint(AngularVelocity velocity, Torque torque) {
    mDriveController.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void resetEncoders(Angle pivotAngle, Angle driveAngle) {
    mPivotModel.setAngle(pivotAngle.in(Radians));
    mDriveModel.setAngle(driveAngle.in(Radians));
  }

  @Override
  public void stop() {
    mPivotAppliedOutput = 0.0;
    mDriveAppliedOutput = 0.0;
  }
}
