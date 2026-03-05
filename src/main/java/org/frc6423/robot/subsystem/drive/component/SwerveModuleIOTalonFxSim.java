// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

public class SwerveModuleIOTalonFxSim extends SwerveModuleIOTalonFx {
  private final DCMotorSim mPivotModel, mDriveModel;

  private final Rotation2d mPivotInitRotation = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double mPivotAppliedVolts;

  private final PIDController mPivotFeedback = new PIDController(100.0, 0.0, 0.0);

  public SwerveModuleIOTalonFxSim(String name, ModuleConfig config, DriveConstants constants) {
    super(name, config, constants);

    mPivotFeedback.enableContinuousInput(-0.5, 0.5);

    mPivotModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                0.0004,
                mConstants.getPivotSensorToMechanismRatio()
                    * mConstants.getPivotRotorToSensorRatio()),
            DCMotor.getKrakenX60Foc(1));

    mDriveModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.005, mConstants.getDriveRotorToMechRatio()),
            DCMotor.getKrakenX60Foc(1));

    mDrive.getSimState().setMotorType(MotorType.KrakenX60);
    mDrive.getSimState().Orientation =
        mDriveConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;
  }

  @Override
  public void periodic() {
    mPivot.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    mDrive.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    mDriveModel.setInput(mDrive.getSimState().getMotorVoltage());

    mPivotModel.update(0.02);
    mDriveModel.update(0.02);

    mDrive
        .getSimState()
        .setRotorVelocity(
            (mDriveModel.getAngularVelocityRPM() / 60.0) * mConstants.getDriveRotorToMechRatio());
  }

  @Override
  public Angle getPivotAngle() {
    return mPivotModel.getAngularPosition();
  }

  @Override
  protected void setPivotSetpoint(Angle setpoint) {
    mPivotModel.setInputVoltage(
        MathUtil.clamp(
            mPivotFeedback.calculate(
                mPivotModel.getAngularPositionRotations(), setpoint.in(Rotations)),
            -12.0,
            12.0));
  }

  @Override
  public Angle getDriveAngle() {
    return mDriveModel.getAngularPosition();
  }

  @Override
  public AngularVelocity getDriveAngularVelocity() {
    return mDriveModel.getAngularVelocity();
  }

  // @Override
  // protected void setDriveSetpoint(AngularVelocity velocity, Torque torque) {
  //   setDriveSetpoint(velocity, false);
  // }

  // @Override
  // protected void setDriveSetpoint(AngularVelocity velocity, boolean focEnabled) {
  //   mDrive.setControl(mVoltVelOut.withVelocity(velocity).withEnableFOC(false).withSlot(1));
  // }
}
