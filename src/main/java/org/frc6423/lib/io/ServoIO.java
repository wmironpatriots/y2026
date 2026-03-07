// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

/** Hardware Interface for interacting with a Servo */
public abstract class ServoIO {
  public final String mName;
  public final int mCanDeviceId;

  public TalonFXConfiguration mTalonConfig;

  public SetpointType mSetpointType;
  public double mSetpointValue;

  protected ServoIO(String name, int canDeviceId, TalonFXConfiguration config) {
    mName = name;
    mCanDeviceId = canDeviceId;

    mTalonConfig = config;
  }

  /** Update hardware loggers */
  public abstract void periodic();

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Get Torque Constant of Servo through gearbox
   *
   * @return {@link Double}
   */
  public double getSystemKtNewtonMetersPerAmps() {
    return mTalonConfig.Feedback.SensorToMechanismRatio / getMotorKtNewtonMetersPerAmps();
  }

  /**
   * Get Torque Constant of Servo
   *
   * @return {@link Double}
   */
  public abstract double getMotorKtNewtonMetersPerAmps();

  /**
   * Get friendly nickname of servo
   *
   * @return {@link String}
   */
  public String getName() {
    return mName;
  }

  @Logged(name = "Applied Voltage (volts)", importance = Importance.DEBUG)
  public abstract double getAppliedVolts();

  @Logged(name = "Supply Current (amps)", importance = Importance.DEBUG)
  public abstract double getSupplyCurrentAmps();

  @Logged(name = "Stator Current (amps)", importance = Importance.DEBUG)
  public abstract double getStatorCurrentAmps();

  @Logged(name = "Torque Current (amps)", importance = Importance.DEBUG)
  public abstract double getTorqueCurrentAmps();

  @Logged(name = "Temperature (celsius)", importance = Importance.DEBUG)
  public abstract double getTemperatureCelsius();

  @Logged(name = "Angular Position (revs)", importance = Importance.DEBUG)
  public abstract double getAngularPositionRevs();

  @Logged(name = "Angular Velocity (revs per second)", importance = Importance.DEBUG)
  public abstract double getAngularVelocityRevsPerSec();

  @Logged(name = "Angular Acceleration (revs per second per second)", importance = Importance.DEBUG)
  public abstract double getAngularAccelerationRevsPerSecPerSec();

  @Logged(name = "Setpoint Value", importance = Importance.DEBUG)
  public double getSetpointValue() {
    return mSetpointValue;
  }

  @Logged(name = "Setpoint Type", importance = Importance.DEBUG)
  public SetpointType getSetpointType() {
    return mSetpointType;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  public abstract void setLeader(ServoIO leader, boolean flipped);

  public abstract void setBrakeModeStatus(boolean brakeEnabled);

  public void resetRelativeEncoder() {
    resetRelativeEncoder(0.0);
  }

  public abstract void resetRelativeEncoder(double positionRevs);

  public abstract void setNeutral();

  public abstract void setTorqueCurrentOutput(double torqueNewtonMeters);

  public abstract void setPositionSetpoint(double positionRevs);

  public abstract void setPositionSetpoint(double positionRevs, double feedforward);

  public abstract void setVelocitySetpoint(double velocityRevsPerSec);

  public abstract void setVelocitySetpoint(double velocityRevsPerSec, double feedforward);

  public abstract void setProfiledPositionSetpoint(double positionRevs);

  public abstract void setProfiledPositionSetpoint(double positionRevs, double feedforward);

  public abstract void setProfiledVelocitySetpoint(double velocityRevsPerSec);

  public abstract void setProfiledVelocitySetpoint(double velocityRevsPerSec, double feedforward);

  /** Type of {@link ServoIO} setpoint */
  public enum SetpointType {
    NEUTRAL,
    TORQUE_CURRENT,
    POSITION,
    VELOCITY,
    PROFILED_POSITION,
    PROFILED_VELOCITY
  }
}
