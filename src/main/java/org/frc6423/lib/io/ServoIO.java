// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.UnaryOperator;

/**
 * A Hardware Interface for controlling a servo
 *
 * <p>A {@link ServoIO} instance can be given a {@link Setpoint} by using the applySetpoint method
 *
 * <p>A {@link ServoIO} instance <strong>must</strong> have its <strong>periodic method called every
 * robot loop for values to be properly logged</strong>
 *
 * <p>It's recommended to utilize {@link MotorSubsystem} or its extensions to create a servo based
 * subsystem instead of utilizing this class directly
 */
public abstract class ServoIO {
  public final String mName;
  public final int mCanDeviceId;

  private Setpoint mCurrentSetpoint = Setpoint.stop();

  /**
   * Create new {@link ServoIO}
   *
   * @param name friendly "nickname" for servo
   * @param canDeviceId integer ID on CAN loop
   */
  protected ServoIO(String name, int canDeviceId) {
    mName = name;
    mCanDeviceId = canDeviceId;
  }

  /** Update all logged values */
  public abstract void periodic();

  /**
   * @return {@link String} representing the "nickname" for servo
   */
  @Logged(name = "Servo Name", importance = Importance.INFO)
  public String getName() {
    return mName;
  }

  /**
   * @return {@link Setpoint} representing the goal of servo
   */
  @Logged(name = "Setpoint", importance = Importance.INFO)
  public Setpoint getSetpoint() {
    return mCurrentSetpoint;
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
   * Set new {@link Setpoint}
   *
   * @param setpoint {@link Setpoint} representing desired goal
   */
  public void applySetpoint(Setpoint setpoint) {
    this.mCurrentSetpoint = setpoint;

    setpoint.applySetpoint(this);
  }

  /**
   * Set the status of motor brake
   *
   * @param active when true motor will apply brake when idling, else motor will coast
   */
  public abstract void setBrakeStatus(boolean active);

  /**
   * Set the status of Field-Oriented Control (FOC)
   *
   * <p><strong>WARNING</strong>: When FOC based control is active, the control output calculated
   * will be a Torque Current (Amps) in contrast to the control output with foc disabled (Volts).
   * This means gains that work with voltage based control will never work with FOC based control
   *
   * @param active when true motor will utilize FOC based control methods
   * @see
   *     https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html#field-oriented-control
   */
  public abstract void setFocStatus(boolean active);

  /**
   * Setup servo for follower mode When in follower mode, servo will mimic the servo set as its
   * leader
   *
   * @param leader {@link ServoIO} to mimic
   * @param flipped when true follower will mimic actions in the opposite direction
   */
  public abstract void setLeader(ServoIO leader, boolean flipped);

  /** Stop all servo control */
  public abstract void stop();

  /**
   * Set voltage setpoint
   *
   * @param volts {@link Voltage} representing desired voltage
   */
  protected abstract void setVoltageSetpoint(Voltage voltage);

  /**
   * Set Torque Current Setpoint Only works for FOC supported servos; Will not run on non-FOC
   * supported servos
   *
   * @param current {@link Current} representing desired torque current
   */
  protected abstract void setTorqueCurrentSetpoint(Current current);

  /**
   * Set dutycycle setpoint
   *
   * @param dutycycle desired dutycycle output
   */
  protected abstract void setDutycycleSetpoint(double dutycycle);

  /**
   * Set position setpoint
   *
   * @param position {@link Angle} representing desired angular position
   * @param slot control gains slot to use
   */
  protected abstract void setPositionSetpoint(Angle position, int slot);

  /**
   * Set velocity setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param acceleration {@link AngularAcceleration} representing desired angular acceleration
   * @param slot control gains slot to use
   */
  protected abstract void setVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot);

  /**
   * Set position motion profiled setpoint
   *
   * @param position {@link Angle} representing desired angular position
   * @param slot control gains slot to use
   */
  protected abstract void setPositionProfiledSetpoint(Angle position, int slot);

  /**
   * Set velocity motion profiled setpoint
   *
   * @param velocity {@link AngularVelocity} representing desired angular velocity
   * @param acceleration {@link AngularAcceleration} representing desired angular acceleration
   * @param slot control gains slot to use
   */
  protected abstract void setVelocityProfiledSetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot);

  /**
   * Represents a goal state of a {@link ServoIO} instance
   *
   * <p>A Setpoint has two components: its {@link ControlType} & its value
   */
  public static class Setpoint implements Sendable {
    /** Represents a method of Motor Control */
    public static enum ControlType {
      /** Idle */
      IDLE,
      /** Open loop voltage control */
      VOLTAGE,
      /** Open loop torque current control (FOC only) */
      TORQUE_CURRENT,
      /** Open loop dutycycle control */
      DUTYCYCLE,
      /** Closed loop position control */
      POSITION,
      /** Closed loop velocity control */
      VELOCITY,
      /** Motion Profiled closed loop position control */
      POSITION_PROFILED,
      /** Motion Profiled closed loop velocity control */
      VELOCITY_PROFILED;

      /**
       * @return true if {@link ControlType} selected is a form of open loop control
       */
      public boolean isOpenLoopControl() {
        if (this == VOLTAGE || this == TORQUE_CURRENT || this == DUTYCYCLE) {
          return true;
        }

        return false;
      }

      /**
       * @return true if {@link ControlType} selected is a form of closed loop control
       */
      public boolean isClosedLoopControl() {
        if (!isOpenLoopControl() && this == IDLE) {
          return true;
        }

        return false;
      }

      /**
       * @return true if {@link ControlType} selected is a form of closed loop position control
       */
      public boolean isPositionControl() {
        if (this == POSITION || this == POSITION_PROFILED) {
          return true;
        }

        return false;
      }

      /**
       * @return true if {@link ControlType} selected is a form of closed loop velocity control
       */
      public boolean isVelocityControl() {
        if (this == POSITION || this == POSITION_PROFILED) {
          return true;
        }

        return false;
      }
    }

    private final double value;
    private final ControlType type;
    private final UnaryOperator<ServoIO> applier;

    private Setpoint(double value, ControlType type, UnaryOperator<ServoIO> applier) {
      this.value = value;
      this.type = type;
      this.applier = applier;
    }

    /**
     * @return get {@link Setpoint} value in BaseUnits
     */
    public double getValue() {
      return value;
    }

    /**
     * @return get {@link Setpoint} {@link ControlType}
     */
    public ControlType getControlType() {
      return type;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addStringProperty("Type", () -> getControlType().toString(), null);
      builder.addDoubleProperty("Value", this::getValue, null);
    }

    /**
     * Create new idle {@link Setpoint}
     *
     * @return {@link Setpoint}
     */
    public static Setpoint stop() {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.stop();
            return io;
          };

      return new Setpoint(0, ControlType.IDLE, applier);
    }

    /**
     * Create new Voltage {@link Setpoint}
     *
     * @param voltage {@link Voltage} representing desired voltage
     * @return {@link Setpoint}
     */
    public static Setpoint createVoltageSetpoint(Voltage voltage) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setVoltageSetpoint(voltage);
            return io;
          };

      return new Setpoint(voltage.in(Volts), ControlType.VOLTAGE, applier);
    }

    /**
     * Create new Torque Current {@link Setpoint}
     *
     * @param torqueCurrent {@link Current} representing desired torque current
     * @return {@link Setpoint}
     */
    public static Setpoint createTorqueCurrentSetpoint(Current torqueCurrent) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setTorqueCurrentSetpoint(torqueCurrent);
            return io;
          };

      return new Setpoint(torqueCurrent.in(Amps), ControlType.VOLTAGE, applier);
    }

    /**
     * Create new Dutycycle {@link Setpoint}
     *
     * @param dutycycle desired dutycycle
     * @return {@link Setpoint}
     */
    public static Setpoint createDutycycleSetpoint(double dutycycle) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setDutycycleSetpoint(dutycycle);
            return io;
          };

      return new Setpoint(dutycycle, ControlType.VOLTAGE, applier);
    }

    /**
     * Create new Position {@link Setpoint}
     *
     * @param angle {@link Angle} representing desired position
     * @return {@link Setpoint}
     */
    public static Setpoint createPositionSetpoint(Angle angle) {
      return createPositionSetpoint(angle, 0);
    }

    /**
     * Create new Position {@link Setpoint}
     *
     * @param angle {@link Angle} representing desired position
     * @param slot integer representing gains slot to use
     * @return {@link Setpoint}
     */
    public static Setpoint createPositionSetpoint(Angle angle, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setPositionSetpoint(angle, slot);
            return io;
          };

      return new Setpoint(angle.in(Revolutions), ControlType.POSITION, applier);
    }

    /**
     * Create new Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @return {@link Setpoint}
     */
    public static Setpoint createVelocitySetpoint(AngularVelocity velocity) {
      return createVelocitySetpoint(velocity, RadiansPerSecondPerSecond.of(0.0), 0);
    }

    /**
     * Create new Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @param acceleration {@link AngularAcceleration} representing the acceleration limit
     * @return {@link Setpoint}
     */
    public static Setpoint createVelocitySetpoint(
        AngularVelocity velocity, AngularAcceleration acceleration) {
      return createVelocitySetpoint(velocity, acceleration, 0);
    }

    /**
     * Create new Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @param slot integer representing gains slot to use
     * @return {@link Setpoint}
     */
    public static Setpoint createVelocitySetpoint(AngularVelocity velocity, int slot) {
      return createVelocitySetpoint(velocity, RadiansPerSecondPerSecond.of(0.0), slot);
    }

    /**
     * Create new Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @param acceleration {@link AngularAcceleration} representing the acceleration limit
     * @param slot integer representing gains slot to use
     * @return {@link Setpoint}
     */
    public static Setpoint createVelocitySetpoint(
        AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setVelocitySetpoint(velocity, acceleration, slot);
            return io;
          };

      return new Setpoint(velocity.in(RevolutionsPerSecond), ControlType.VELOCITY, applier);
    }

    /**
     * Create new Motion Profiled Position {@link Setpoint}
     *
     * @param angle {@link Angle} representing desired position
     * @return {@link Setpoint}
     */
    public static Setpoint createProfiledPositionSetpoint(Angle angle) {
      return createProfiledPositionSetpoint(angle, 0);
    }

    /**
     * Create new Motion Profiled Position {@link Setpoint}
     *
     * @param angle {@link Angle} representing desired position
     * @param slot integer representing gains slot to use
     * @return {@link Setpoint}
     */
    public static Setpoint createProfiledPositionSetpoint(Angle angle, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setPositionProfiledSetpoint(angle, slot);
            return io;
          };

      return new Setpoint(angle.in(Revolutions), ControlType.POSITION, applier);
    }

    /**
     * Create new Motion Profiled Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @return {@link Setpoint}
     */
    public static Setpoint createProfiledVelocitySetpoint(AngularVelocity velocity) {
      return createVelocitySetpoint(velocity, RadiansPerSecondPerSecond.of(0.0), 0);
    }

    /**
     * Create new Motion Profiled Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @param acceleration {@link AngularAcceleration} representing the acceleration limit
     * @return {@link Setpoint}
     */
    public static Setpoint createProfiledVelocitySetpoint(
        AngularVelocity velocity, AngularAcceleration acceleration) {
      return createVelocitySetpoint(velocity, acceleration, 0);
    }

    /**
     * Create new Motion Profiled Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @param slot integer representing gains slot to use
     * @return {@link Setpoint}
     */
    public static Setpoint createProfiledVelocitySetpoint(AngularVelocity velocity, int slot) {
      return createVelocitySetpoint(velocity, RadiansPerSecondPerSecond.of(0.0), slot);
    }

    /**
     * Create new Motion Profiled Velocity {@link Setpoint}
     *
     * @param velocity {@link AngularVelocity} representing desired velocity
     * @param acceleration {@link AngularAcceleration} representing the acceleration limit
     * @param slot integer representing gains slot to use
     * @return {@link Setpoint}
     */
    public static Setpoint createProfiledVelocitySetpoint(
        AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setVelocityProfiledSetpoint(velocity, acceleration, slot);
            return io;
          };

      return new Setpoint(velocity.in(RevolutionsPerSecond), ControlType.VELOCITY, applier);
    }

    /**
     * Apply setpoint request to {@link ServoIO}
     *
     * @param io {@link ServoIO} to run {@link Setpoint}
     */
    void applySetpoint(ServoIO io) {
      applier.apply(io);
    }
  }
}
