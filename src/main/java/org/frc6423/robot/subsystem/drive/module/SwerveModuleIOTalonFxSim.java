// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.module;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc6423.lib.sim.MechSim;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

public class SwerveModuleIOTalonFxSim extends SwerveModuleIOTalonFx {
  private final DCMotorSim mPivotModel, mDriveModel;

  private final Notifier mUpdater;

  private double mPreviousTimestamp = 0.0;

  public SwerveModuleIOTalonFxSim(ModuleConfig config, DriveConstants driveConstants) {
    super(config, driveConstants);

    mPivotModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.004, mDriveConstants.getPivotRotorToSensorRatio()),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0);
    mDriveModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.025, mDriveConstants.getDriveSensorToMechRatio()),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0);

    mUpdater = new Notifier(() -> updateSimulation());
    mUpdater.startPeriodic(0.005);
  }

  public void updateSimulation() {
    mPivot.getSimState().setSupplyVoltage(12.0);
    mDrive.getSimState().setSupplyVoltage(12.0);

    mPivotModel.setInputVoltage(
        MechSim.addFriction(mPivot.getSimState().getMotorVoltageMeasure(), Volts.of(0.25))
            .in(Volts));
    mDriveModel.setInputVoltage(
        MechSim.addFriction(mDrive.getSimState().getMotorVoltageMeasure(), Volts.of(0.25))
            .in(Volts));

    mPivotModel.update(Timer.getFPGATimestamp() - mPreviousTimestamp);
    mDriveModel.update(Timer.getFPGATimestamp() - mPreviousTimestamp);
    mPreviousTimestamp = Timer.getFPGATimestamp();

    // Update simulation using system model
    mPivot
        .getSimState()
        .setRawRotorPosition(
            mPivotModel
                .getAngularPosition()
                .times(mConfig.pivotConfig().Feedback.SensorToMechanismRatio));
    mPivot
        .getSimState()
        .setRotorVelocity(
            mPivotModel
                .getAngularVelocity()
                .times(mConfig.pivotConfig().Feedback.SensorToMechanismRatio));
    mDrive
        .getSimState()
        .setRawRotorPosition(
            mDriveModel
                .getAngularPosition()
                .times(mConfig.driveConfig().Feedback.SensorToMechanismRatio));
    mDrive
        .getSimState()
        .setRotorVelocity(
            mDriveModel
                .getAngularVelocity()
                .times(mConfig.driveConfig().Feedback.SensorToMechanismRatio));
  }
}
