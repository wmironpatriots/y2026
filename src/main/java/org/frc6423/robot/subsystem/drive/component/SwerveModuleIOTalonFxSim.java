// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

public class SwerveModuleIOTalonFxSim extends SwerveModuleIOTalonFx {
  private final DCMotorSim mPivotSim, mDriveSim;

  private double mPreviousTimestamp;
  private final Notifier mNotifier;

  public SwerveModuleIOTalonFxSim(String name, ModuleConfig config, DriveConstants constants) {
    super(name, config, constants);

    mPivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.004, mConstants.getPivotSensorToMechanismRatio()),
            DCMotor.getKrakenX60Foc(1));

    mDriveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.025, mConstants.getDriveRotorToMechRatio()),
            DCMotor.getKrakenX60Foc(1));

    mPivot.getSimState().setMotorType(MotorType.KrakenX60);
    mPivot.getSimState().Orientation =
        mPivotConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    mDrive.getSimState().setMotorType(MotorType.KrakenX60);
    mDrive.getSimState().Orientation =
        mDriveConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    mNotifier =
        new Notifier(
            () -> {
              final double timestamp = Timer.getFPGATimestamp();
              final double deltaTime = timestamp - mPreviousTimestamp;
              mPreviousTimestamp = timestamp;

              mPivot.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
              mDrive.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

              mPivotSim.setInputVoltage(mPivot.getSimState().getMotorVoltage());
              mDriveSim.setInputVoltage(mDrive.getSimState().getMotorVoltage());

              mPivot
                  .getSimState()
                  .setRawRotorPosition(
                      mPivotSim.getAngularPosition().in(Rotations)
                          * mConstants.getPivotSensorToMechanismRatio());
              mPivot
                  .getSimState()
                  .setRotorVelocity(
                      mPivotSim.getAngularVelocity().in(RotationsPerSecond)
                          * mConstants.getPivotSensorToMechanismRatio());

              mDrive
                  .getSimState()
                  .setRawRotorPosition(
                      mDriveSim.getAngularPosition().in(Rotations)
                          * mConstants.getDriveRotorToMechRatio());
              mDrive
                  .getSimState()
                  .setRotorVelocity(
                      mDriveSim.getAngularVelocity().in(RotationsPerSecond)
                          * mConstants.getDriveRotorToMechRatio());
            });

    mNotifier.startPeriodic(0.002);
  }
}
