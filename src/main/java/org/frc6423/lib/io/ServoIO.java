// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;

// TODO cleanup JavaDoc
/**
 * A Hardware Interface for controlling a servo
 *
 * <p>A {@link ServoIO} instance <strong>must</strong> have its <strong>periodic method called every
 * robot loop for values to be properly logged</strong>
 */
public abstract class ServoIO {
  public final String mName;
  public final int mDeviceId;

  public final TalonFXConfiguration mTalonConfig;

  public final double mMotorKt;

  /**
   * Create new {@link ServoIO}
   *
   * @param name {@link String} representing servo nickname
   * @param canBus {@link CANBus} representing CAN bus loop device is in
   * @param canDeviceId {@link Integer} representing the id of CAN device
   * @param talonConfig {@link TalonFXConfiguration} representing the servo config
   * @param motorKt {@link Double} representing the servo's kT rating
   */
  protected ServoIO(
      String name, CANBus canBus, int canDeviceId, TalonFXConfiguration config, double motorKt) {
    mName = name;
    mDeviceId = canDeviceId;

    mTalonConfig = config;

    mMotorKt = motorKt;
  }

  /** Update all logged values */
  public abstract void periodic();

  /**
   * @return {@link Double} representing the motor's kT after going through the gearbox of system
   */
  public double getSystemKt() {
    return mTalonConfig.Feedback.SensorToMechanismRatio / mMotorKt;
  }

  /**
   * @return {@link String} representing the "nickname" for servo
   */
  @Logged(name = "Servo Name", importance = Importance.INFO)
  public String getName() {
    return mName;
  }

  /**
   * @return {@link Voltage} representing the applied voltage of servo
   */
  @Logged(name = "Applied Voltage", importance = Importance.INFO)
  public abstract Voltage getAppliedVoltage();

  /**
   * @return {@link Current} representing the input supply current of servo
   */
  @Logged(name = "Supply Current", importance = Importance.INFO)
  public abstract Current getSupplyCurrent();

  /**
   * @return {@link Current} representing the output stator current of servo
   */
  @Logged(name = "Stator Current", importance = Importance.INFO)
  public abstract Current getStatorCurrent();

  /**
   * @return {@link Current} representing the torque current output of servo
   */
  @Logged(name = "Torque Current", importance = Importance.INFO)
  public abstract Current getTorqueCurrent();

  /**
   * @return {@link Angle} representing the angular position of servo
   */
  @Logged(name = "Angle", importance = Importance.INFO)
  public abstract Angle getAngle();

  /**
   * @return {@link AngularVelocity} representing the angular velocity of servo
   */
  @Logged(name = "Angular Velocity", importance = Importance.INFO)
  public abstract AngularVelocity getAngularVelocity();

  /**
   * @return {@link AngularAcceleration} representing the angular acceleration of servo
   */
  @Logged(name = "Angular Acceleration", importance = Importance.INFO)
  public abstract AngularAcceleration getAngularAcceleration();

  /**
   * @return {@link Temperature} representing the temperature of servo
   */
  @Logged(name = "Temperature", importance = Importance.INFO)
  public abstract Temperature getTemperature();

  /**
   * Setup servo for follower mode When in follower mode, servo will mimic the servo set as its
   * leader
   *
   * @param leader {@link ServoIO} to mimic
   * @param flipped when true follower will mimic actions in the opposite direction
   */
  public abstract void setLeader(ServoIO leader, boolean flipped);

  /**
   * Set the status of motor brake
   *
   * @param active when true motor will apply brake when idling, else motor will coast
   */
  public abstract void setBrakeStatus(boolean active);

  /**
   * Reset internal relative encoder to specified angular position
   *
   * @param angle {@link Angle} representing angular position to reset to
   */
  public abstract void resetEncoder(Angle angle);

  /** Stop servo completely */
  public abstract void stop();

  // * OPEN-LOOP

  /**
   * Set Voltage Setpoint
   *
   * @param voltage {@link Voltage} representing desired voltage output
   * @param withFoc when true, FOC will be enabled
   */
  public abstract void setVoltageSetpoint(Voltage voltage, boolean withFoc);

  /**
   * Set Torque Current Setpoint
   *
   * @param current {@link Current} representing desired current output
   */
  public abstract void setTorqueCurrentSetpoint(Current current);

  // * POSITION

  /**
   * Set Voltage based Position Setpoint
   *
   * @param angle {@link Angle} representing desired angular position
   * @param withFoc when true, FOC will be enabled
   */
  public void setVoltagePositionSetpoint(Angle angle, boolean withFoc) {
    setVoltagePositionSetpoint(angle, Volts.zero(), true);
  }

  /**
   * Set Voltage based Position Setpoint /w feedfoward voltage
   *
   * @param angle {@link Angle} representing desired angular position
   * @param feedforward {@link Voltage} representing feedforward voltage to apply
   * @param withFoc when true, FOC will be enabled
   */
  public abstract void setVoltagePositionSetpoint(
      Angle angle, Voltage feedforward, boolean withFoc);

  /**
   * Set Torque based Position Setpoint /w feedforward current
   *
   * @param angle {@link Angle} representing desired angular position
   */
  public void setTorquePositionSetpoint(Angle angle) {
    setTorquePositionSetpoint(angle, Amps.zero());
  }

  /**
   * Set Torque based Position Setpoint /w specified output torque
   *
   * @param angle {@link Angle} representing desired angular position
   * @param torque {@link Torque} representing desired torque output of system
   */
  public void setTorquePositionSetpoint(Angle angle, Torque torque) {
    setTorquePositionSetpoint(angle, Amps.of(getSystemKt() / torque.in(NewtonMeters)));
  }

  /**
   * Set Torque based Position Setpoint /w feedforward current
   *
   * @param angle {@link Angle} representing desired angular position
   * @param feedforward {@link Current} representing feedforward current to apply
   */
  public abstract void setTorquePositionSetpoint(Angle angle, Current feedforward);

  /**
   * Set Voltage based Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param withFoc when true, FOC will be enabled
   */
  public void setVoltageVelocitySetpoint(AngularVelocity velocity, boolean withFoc) {
    setVoltageVelocitySetpoint(velocity, Volts.zero(), withFoc);
  }

  /**
   * Set Voltage based Velocity Setpoint /w feedforward voltage
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param feedforward {@link Voltage} representing feedforward voltage to apply
   * @param withFoc when true, FOC will be enabled
   */
  public abstract void setVoltageVelocitySetpoint(
      AngularVelocity velocity, Voltage feedforward, boolean withFoc);

  /**
   * Set Torque based Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   */
  public void setTorqueVelocitySetpoint(AngularVelocity velocity) {
    setTorqueVelocitySetpoint(velocity, Amps.zero());
  }

  /**
   * Set Torque based Velocity Setpoint /w specified output torque
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param torque {@link torque} representing desired torque output of system
   */
  public void setTorqueVelocitySetpoint(AngularVelocity velocity, Torque torque) {
    setTorqueVelocitySetpoint(velocity, Amps.of(getSystemKt() / torque.in(NewtonMeters)));
  }

  /**
   * Set Torque based Velocity Setpoint /w feedforward current
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param feedforward {@link Current} representing feedforward current to apply
   */
  public abstract void setTorqueVelocitySetpoint(AngularVelocity velocity, Current feedforward);

  /**
   * Set Torque based Velocity Setpoint /w specified acceleration
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param acceleration {@link AngularAcceleration} representing desired angular acceleration
   */
  public abstract void setTorqueVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration);

  /**
   * Set Voltage based Motion Profiled Position Setpoint
   *
   * @param angle {@link Angle} representing desired angular position
   * @param withFoc when true, FOC will be enabled
   */
  public void setVoltageMotionProfiledPositionSetpoint(Angle angle, boolean withFoc) {
    setVoltageMotionProfiledPositionSetpoint(angle, Volts.zero(), withFoc);
  }

  /**
   * Set Voltage based Motion Profiled Position Setpoint /w feedforward voltage
   *
   * @param angle {@link Angle} representing desired angular position
   * @param feedforward
   * @param withFoc when true, FOC will be enabled
   */
  public abstract void setVoltageMotionProfiledPositionSetpoint(
      Angle angle, Voltage feedforward, boolean withFoc);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint
   *
   * @param angle {@link Angle} representing desired angular velocity
   */
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle) {
    setTorqueMotionProfiledPositionSetpoint(angle, Amps.zero());
  }

  /**
   * Set Torque based Motion Profiled Velocity Setpoint /w specified output torque
   *
   * @param angle {@link Angle} representing desired angular velocity
   * @param torque {@link torque} representing desired torque output of system
   */
  public void setTorqueMotionProfiledPositionSetpoint(Angle angle, Torque torque) {
    setTorqueMotionProfiledPositionSetpoint(
        angle, Amps.of(getSystemKt() / torque.in(NewtonMeters)));
  }

  /**
   * Set Torque based Motion Profiled Velocity Setpoint
   *
   * @param angle {@link Angle} representing desired angular velocity
   * @param feedforward {@link Current} representing feedforward current toa apply
   */
  public abstract void setTorqueMotionProfiledPositionSetpoint(Angle angle, Current feedforward);

  /**
   * Set Voltage based Motion Profiled Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param withFoc when true, FOC will be enabled
   */
  public abstract void setVoltageMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, boolean withFoc);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   */
  public abstract void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint /w specified angular acceleration
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param torque {@link torque} representing desired torque output of system
   */
  public void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity, Torque torque) {
    setTorqueMotionProfiledVelocitySetpoint(
        velocity, Amps.of(getSystemKt() / torque.in(NewtonMeters)));
  }

  /**
   * Set Torque based Motion Profiled Velocity Setpoint /w feedforward current
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param feedforward {@link Current} representing feedforward current to apply
   */
  public abstract void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, Current feedforward);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint /w specified angular acceleration
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param acceleration {@link AngularAcceleration} representing desired angular acceleration
   */
  public abstract void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration);
}
