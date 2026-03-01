// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.NewtonMeters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;

// TODO javadoc :skull:
// TODO use request status signal returns to return booleans for successes
/** Base Hardware Interface for interacting with a Servo */
public abstract class ServoIO {
  public final String mName;
  public final int mCanDeviceId;

  public final TalonFXConfiguration mTalonConfig;

  /**
   * Create new {@link ServoIO}
   *
   * @param name {@link String} A friendly nickname for servo
   * @param canBus {@link CANBus} CAN bus servo is on
   * @param canDeviceId {@link Integer} CAN ID of servo
   * @param config {@link TalonFXConfiguration} Hardware Config of Servo
   */
  protected ServoIO(String name, CANBus canBus, int canDeviceId, TalonFXConfiguration config) {
    mName = name;
    mCanDeviceId = canDeviceId;

    mTalonConfig = config;
  }

  public abstract void periodic();

  // * GETTERS
  /**
   * Get Torque Constant of Servo
   *
   * <p>@see https://en.wikipedia.org/wiki/Motor_constants
   *
   * @return {@link TorqueUnit} {@link Per} {@link CurrentUnit}
   */
  public abstract Per<TorqueUnit, CurrentUnit> getMotorKt();

  /**
   * Get Torque Constant of Servo through gearbox
   *
   * <p>@see https://en.wikipedia.org/wiki/Motor_constants
   *
   * @return {@link TorqueUnit} {@link Per} {@link CurrentUnit}
   */
  public Per<TorqueUnit, CurrentUnit> getSystemKt() {
    return NewtonMeters.of(
            mTalonConfig.Feedback.SensorToMechanismRatio / getMotorKt().in(NewtonMeters.per(Amps)))
        .per(Amps);
  }

  /**
   * Get friendly nickname of servo
   *
   * @return {@link String}
   */
  @Logged(name = "Servo Name", importance = Importance.DEBUG)
  public String getName() {
    return mName;
  }

  /**
   * Get supply voltage of servo (voltage going in)
   *
   * @return {@link Voltage}
   */
  @Logged(name = "Supply Voltage (volts)", importance = Importance.DEBUG)
  public abstract Voltage getSupplyVoltage();

  /**
   * Get stator voltage of servo (voltage going out; voltage output)
   *
   * @return {@link Voltage}
   */
  @Logged(name = "Stator Voltage (volts)", importance = Importance.DEBUG)
  public abstract Voltage getStatorVoltage();

  /**
   * Get supply current of servo (current going in)
   *
   * @return {@link Current}
   */
  @Logged(name = "Supply Current (amps)", importance = Importance.DEBUG)
  public abstract Current getSupplyCurrent();

  /**
   * Get stator current of servo (current going in)
   *
   * @return {@link Current}
   */
  @Logged(name = "Stator Current (amps)", importance = Importance.DEBUG)
  public abstract Current getStatorCurrent();

  /**
   * Get torque current of servo
   *
   * @return {@link Current}
   */
  @Logged(name = "Torque Current (amps)", importance = Importance.DEBUG)
  public abstract Current getTorqueCurrent();

  /**
   * Get raw rotor angular position of servo
   *
   * @return {@link Angle}
   */
  @Logged(name = "Raw Angle (rads)", importance = Importance.DEBUG)
  public abstract Angle getRawAngle();

  /**
   * Get angular position of servo
   *
   * <p>Note that this output will depend on talon feedback configs
   *
   * <p>Note that gear ratios specified in hardware config will automatically be applied
   *
   * @return {@link Angle}
   */
  @Logged(name = "Angle (rads)", importance = Importance.DEBUG)
  public abstract Angle getAngle();

  /**
   * Get angular velocity if servo
   *
   * <p>Note that gear ratios specified in hardware config will automatically be applied
   *
   * @return {@link AngularVelocity}
   */
  @Logged(name = "Angular Velocity (rads per second)", importance = Importance.DEBUG)
  public abstract AngularVelocity getAngularVelocity();

  /**
   * Get angular acceleration of servo
   *
   * <p>Note that gear ratios specified in hardware config will automatically be applied
   *
   * @return {@link AngularAcceleration}
   */
  @Logged(name = "Angular Acceleration (rads per second per second)", importance = Importance.DEBUG)
  public abstract AngularAcceleration getAngularAcceleration();

  /**
   * Get servo temperature
   *
   * @return {@link Temperature}
   */
  @Logged(name = "Temperature (celsius)", importance = Importance.CRITICAL)
  public abstract Temperature getTemperature();

  // * CONFIG SETTERS
  /**
   * Set a 'leader' servo for 'follower' servo to copy
   *
   * @param leader {@link ServoIO} 'leader' servo
   * @param flipped {@link Boolean} whether 'follower' should copy actions in the opposite direction
   */
  public abstract void setLeader(ServoIO leader, boolean flipped);

  /**
   * Set the gains slot subsystem should use for position/velocity control
   *
   * @param slot {@link Integer} slot to use
   */
  public abstract void setGainsSlot(int slot);

  /**
   * Set neutral mode brake status
   *
   * <p><strong>IMPORTANT</strong>: Disabling the neutral mode brake means that the servo will not
   * put effort into deaccelerating. This can be extremely dangerous with subsystems moving large
   * masses. Make sure you use this feature safely
   *
   * @param active {@link Boolean} status of brake expressed as a boolean
   */
  public abstract void setBrakeStatus(boolean active);

  /**
   * Reset internal relative encoder to specified angle
   *
   * @param angle {@link Angle} angular position to reset to
   */
  public abstract void resetEncoder(Angle angle);

  // * CONTROL SETTERS
  /** Request servo to go into neutral mode */
  public abstract void stop();

  // * CONTROL SETTERS (VOLTS BASED)
  public abstract void setVoltageSetpoint(Voltage voltage, boolean withFoc);

  public abstract void setVoltagePositionSetpoint(Angle angle, boolean withFoc);

  public abstract void setVoltagePositionSetpoint(
      Angle angle, Voltage feedforward, boolean withFoc);

  public abstract void setVoltageVelocitySetpoint(AngularVelocity velocity, boolean withFoc);

  public abstract void setVoltageVelocitySetpoint(
      AngularVelocity velocity, Voltage feedforward, boolean withFoc);

  public abstract void setVoltageMotionProfiledPositionSetpoint(Angle angle, boolean withFoc);

  public abstract void setVoltageMotionProfiledPositionSetpoint(
      Angle angle, Voltage feedforward, boolean withFoc);

  public abstract void setVoltageMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, boolean withFoc);

  // * CONTROL SETTERS (TORQUE AMPS BASED)
  public abstract void setTorqueCurrentSetpoint(Current current);

  public abstract void setTorquePositionSetpoint(Angle angle);

  public abstract void setTorquePositionSetpoint(Angle angle, Torque torque);

  public abstract void setTorquePositionSetpoint(Angle angle, Current feedforward);

  public abstract void setTorqueVelocitySetpoint(AngularVelocity velocity);

  public abstract void setTorqueVelocitySetpoint(AngularVelocity velocity, Torque torque);

  public abstract void setTorqueVelocitySetpoint(AngularVelocity velocity, Current feedforward);

  public abstract void setTorqueVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration);

  public abstract void setTorqueMotionProfiledPositionSetpoint(Angle angle);

  public abstract void setTorqueMotionProfiledPositionSetpoint(Angle angle, Torque torque);

  public abstract void setTorqueMotionProfiledPositionSetpoint(Angle angle, Current feedforward);

  public abstract void setTorqueMotionProfiledVelocitySetpoint(AngularVelocity velocity);

  public abstract void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, Torque torque);

  public abstract void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, Current feedforward);

  public abstract void setTorqueMotionProfiledVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration);
}
