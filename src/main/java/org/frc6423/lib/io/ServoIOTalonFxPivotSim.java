// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** {@link ServoIOTalon} extension for pivot simulation */
public class ServoIOTalonFxPivotSim extends ServoIOTalonFx {
  private final SingleJointedArmSim mPhysicsModel;

  private final double mRatio;

  private double previousTimestamp;
  private final Notifier mNotifier;

  /**
   * Create new {@link ServoIOTalonFxRollerSim}
   *
   * @param name {@link String} representing servo nickname
   * @param canBus {@link CANBus} representing CAN bus loop device is in
   * @param canDeviceId {@link Integer} representing the id of CAN device
   * @param talonConfig {@link TalonFXConfiguration} representing the servo config
   * @param rotationalInertia {@link MomentOfInertia} representing the rotational inertia of system
   * @param motorType {@link MotorType} representing the type of motors used for gearbox input
   * @param gearbox {@link DCMotor} representing the gearbox input
   * @param rotorToMechanismRatio {@link Double} representing the gear ratio between the servo rotor
   *     and the mechanism
   */
  public ServoIOTalonFxPivotSim(
      String name,
      CANBus canBus,
      int deviceId,
      TalonFXConfiguration talonConfig,
      MomentOfInertia rotationalInertia,
      Distance armLength,
      Angle minAngle,
      Angle maxAngle,
      Angle startingAngle,
      boolean simulateGravity,
      MotorType motorType,
      DCMotor gearbox,
      double rotorToMechanismRatio) {
    super(name, canBus, deviceId, talonConfig);

    mPhysicsModel =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                gearbox, rotationalInertia.in(KilogramSquareMeters), rotorToMechanismRatio),
            gearbox,
            rotorToMechanismRatio,
            armLength.in(Meters),
            minAngle.in(Radians),
            maxAngle.in(Radians),
            simulateGravity,
            startingAngle.in(Radians));

    mTalonConfig =
        mTalonConfig.withFeedback(
            new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    mRatio = rotorToMechanismRatio;

    mServo.getConfigurator().apply(mTalonConfig);

    mServo.getSimState().setMotorType(motorType);
    mServo.getSimState().Orientation =
        mTalonConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    mNotifier =
        new Notifier(
            () -> {
              final double timestamp = Timer.getFPGATimestamp();
              final double deltaTime = timestamp - previousTimestamp;
              previousTimestamp = timestamp;

              mServo.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

              mPhysicsModel.setInputVoltage(mServo.getSimState().getMotorVoltage());
              mPhysicsModel.update(deltaTime);

              mServo
                  .getSimState()
                  .setRawRotorPosition(
                      (mPhysicsModel.getAngleRads() / (Math.PI * 2)) * rotorToMechanismRatio);
              mServo
                  .getSimState()
                  .setRotorVelocity(
                      (mPhysicsModel.getVelocityRadPerSec() / (Math.PI * 2))
                          * rotorToMechanismRatio);
            });

    mNotifier.startPeriodic(0.002);
  }

  @Override
  public Angle getAngle() {
    return mRawPositionSignal.getValue().div(mRatio);
  }
}
