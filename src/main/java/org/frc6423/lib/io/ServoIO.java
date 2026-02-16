// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

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

  /**
   * Set Voltage based Position Setpoint
   *
   * @param angle {@link Angle} representing desired angular position
   * @param withFoc when true, FOC will be enabled
   * @param slot the gains slot to use
   */
  public abstract void setVoltagePositionSetpoint(Angle angle, boolean withFoc, int slot);

  /**
   * Set Torque based Position Setpoint
   *
   * @param angle {@link Angle} representing desired angular position
   * @param slot the gains slot to use
   */
  public abstract void setTorquePositionSetpoint(Angle angle, int slot);

  /**
   * Set Torque based Position Setpoint /w specified output torque
   *
   * @param angle {@link Angle} representing desired angular position
   * @param torque {@link Torque} representing desired torque output of system
   * @param slot the gains slot to use
   */
  public abstract void setTorquePositionSetpoint(Angle angle, Torque torque, int slot);

  /**
   * Set Voltage based Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param withFoc when true, FOC will be enabled
   * @param slot the gains slot to use
   */
  public abstract void setVoltageVelocitySetpoint(
      AngularVelocity velocity, boolean withFoc, int slot);

  /**
   * Set Torque based Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param slot the gains slot to use
   */
  public abstract void setTorqueVelocitySetpoint(AngularVelocity velocity, int slot);

  /**
   * Set Torque based Velocity Setpoint /w specified acceleration
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param acceleration {@link AngularAcceleration} representing desired angular acceleration
   * @param slot the gains slot to use
   */
  public abstract void setTorqueVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot);

  /**
   * Set Torque based Velocity Setpoint /w specified output torque
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param torque {@link torque} representing desired torque output of system
   * @param slot the gains slot to use
   */
  public abstract void setTorqueVelocitySetpoint(AngularVelocity velocity, Torque torque, int slot);

  /**
   * Set Voltage based Motion Profiled Position Setpoint
   *
   * @param angle {@link Angle} representing desired angular position
   * @param withFoc when true, FOC will be enabled
   * @param slot the gains slot to use
   */
  public abstract void setVoltageMotionProfiledPositionSetpoint(
      Angle angle, boolean withFoc, int slot);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint
   *
   * @param angle {@link Angle} representing desired angular velocity
   * @param slot the gains slot to use
   */
  public abstract void setTorqueMotionProfiledPositionSetpoint(Angle angle, int slot);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint /w specified output torque
   *
   * @param angle {@link Angle} representing desired angular velocity
   * @param torque {@link torque} representing desired torque output of system
   * @param slot the gains slot to use
   */
  public abstract void setTorqueMotionProfiledPositionSetpoint(
      Angle angle, Torque torque, int slot);

  /**
   * Set Voltage based Motion Profiled Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param withFoc when true, FOC will be enabled
   * @param slot the gains slot to use
   */
  public abstract void setVoltageMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, boolean withFoc, int slot);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param slot the gains slot to use
   */
  public abstract void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity, int slot);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint /w specified angular acceleration
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param acceleration {@link AngularAcceleration} representing desired angular acceleration
   * @param slot the gains slot to use
   */
  public abstract void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot);

  /**
   * Set Torque based Motion Profiled Velocity Setpoint /w specified angular acceleration
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param torque {@link torque} representing desired torque output of system
   * @param slot the gains slot to use
   */
  public abstract void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, Torque torque, int slot);
}
