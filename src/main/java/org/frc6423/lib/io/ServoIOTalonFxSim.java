// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Optional;

/** Simulated extension of {@link ServoIOTalonFx} */
public class ServoIOTalonFxSim extends ServoIOTalonFx {
  private final ChassisReference positiveDirection;

  private final Optional<DCMotorSim> dcMotorSim;
  private final Optional<FlywheelSim> flywheelSim;
  private final Optional<SingleJointedArmSim> armSim;
  private final Optional<ElevatorSim> elevatorSim;

  /**
   * Create new {@link ServoIOTalonFxSim} with a {@link DCMotorSim} backend
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer representing the CAN identification
   * @param canBusId {@link CANBus} representing the CAN bus device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   * @param positiveDirection {@link ChassisReference} representing the positive angular direction
   *     of the servo
   * @param dcMotorSim {@link DCMotorSim} representing the physics model the simulation should use
   *     as backend
   */
  public ServoIOTalonFxSim(
      String name,
      int canDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      ChassisReference positiveDirection,
      DCMotorSim dcMotorSim) {
    super(name, canDeviceId, canBus, config, Celsius.of(999));

    this.positiveDirection = positiveDirection;

    this.dcMotorSim = Optional.of(dcMotorSim);
    this.flywheelSim = Optional.empty();
    this.armSim = Optional.empty();
    this.elevatorSim = Optional.empty();
  }

  /**
   * Create new {@link ServoIOTalonFxSim} with a {@link FlywheelSim} backend
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer representing the CAN identification
   * @param canBusId {@link CANBus} representing the CAN bus device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   * @param positiveDirection {@link ChassisReference} representing the positive angular direction
   *     of the servo
   * @param flywheelSim {@link FlywheelSim} representing the physics model the simulation should use
   *     as backend
   */
  public ServoIOTalonFxSim(
      String name,
      int canDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      ChassisReference positiveDirection,
      FlywheelSim flywheelSim) {
    super(name, canDeviceId, canBus, config, Celsius.of(999));

    this.positiveDirection = positiveDirection;

    this.dcMotorSim = Optional.empty();
    this.flywheelSim = Optional.of(flywheelSim);
    this.armSim = Optional.empty();
    this.elevatorSim = Optional.empty();
  }

  /**
   * Create new {@link ServoIOTalonFxSim} with a {@link SingleJointedArmSim} backend
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer representing the CAN identification
   * @param canBusId {@link CANBus} representing the CAN bus device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   * @param positiveDirection {@link ChassisReference} representing the positive angular direction
   *     of the servo
   * @param armSim {@link SingleJointedArmSim} representing the physics model the simulation should
   *     use as backend
   */
  public ServoIOTalonFxSim(
      String name,
      int canDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      ChassisReference positiveDirection,
      SingleJointedArmSim armSim) {
    super(name, canDeviceId, canBus, config, Celsius.of(999));

    this.positiveDirection = positiveDirection;

    this.dcMotorSim = Optional.empty();
    this.flywheelSim = Optional.empty();
    this.armSim = Optional.of(armSim);
    this.elevatorSim = Optional.empty();
  }

  /**
   * Create new {@link ServoIOTalonFxSim} with a {@link ElevatorSim} backend
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer representing the CAN identification
   * @param canBusId {@link CANBus} representing the CAN bus device is on
   * @param config {@link TalonFXConfiguration} representing servo config
   * @param positiveDirection {@link ChassisReference} representing the positive angular direction
   *     of the servo
   * @param elevatorSim {@link ElevatorSim} representing the physics model the simulation should use
   *     as backend
   */
  public ServoIOTalonFxSim(
      String name,
      int canDeviceId,
      CANBus canBus,
      TalonFXConfiguration config,
      ChassisReference positiveDirection,
      ElevatorSim elevatorSim) {
    super(name, canDeviceId, canBus, config, Celsius.of(999));

    this.positiveDirection = positiveDirection;

    this.dcMotorSim = Optional.empty();
    this.flywheelSim = Optional.empty();
    this.armSim = Optional.empty();
    this.elevatorSim = Optional.of(elevatorSim);
  }

  @Override
  public void periodic() {
    super.periodic();

    var simState = getTalonFXSimState();
  }

  /**
   * @return {@link TalonFXSimState} for servo
   */
  // TODO finish writing simulation periodic logic
  public TalonFXSimState getTalonFXSimState() {
    var state = servo.getSimState();

    state.Orientation = positiveDirection;

    return state;
  }
}
