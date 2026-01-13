// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.function.UnaryOperator;

/** A I/O interface for controlling a servo */
public abstract class ServoIO {
  public static final Temperature DEFAULT_SHUTDOWN_TEMPERATURE = Celsius.of(75);

  public final String name;

  private final Temperature maxTemp;
  private Alert shutdownAlert =
      new Alert(getName() + " is overheating! SHUTTING DOWN to prevent damage!", AlertType.kError);

  private Setpoint currentSetpoint = Setpoint.stop();
  private boolean isOverheated = false;

  /**
   * Create new {@link ServoIO}
   *
   * @param name friendly "nickname" for servo
   */
  protected ServoIO(String name) {
    this(name, DEFAULT_SHUTDOWN_TEMPERATURE);
  }

  /**
   * Create new {@link ServoIO}
   *
   * @param name friendly "nickname" for servo
   * @param shutdownTemperature {@link Temperature} representing the maximum temperature a servo can
   *     be at before shutting down
   */
  protected ServoIO(String name, Temperature shutdownTemperature) {
    this.name = name;
    this.maxTemp = shutdownTemperature;
  }

  /** Periodic Logic Should be called every robot loop */
  public void periodic() {
    // TODO put overheating motors in coast
    if (getTemperature().gt(maxTemp)) {
      shutdownAlert.set(true);

      isOverheated = true;
      idle();
    } else if (shutdownAlert.get()) {
      shutdownAlert.set(false);

      isOverheated = false;
      applySetpoint(currentSetpoint);
    }
  }

  /**
   * @return friendly nickname for servo
   */
  @Logged(name = "Servo Name", importance = Importance.INFO)
  public String getName() {
    return name;
  }

  /**
   * @return true if servo has been disabled due to overheating
   */
  @Logged(name = "Overheated", importance = Importance.INFO)
  public boolean isOverheated() {
    return isOverheated;
  }

  /**
   * @return current {@link Setpoint}
   */
  @Logged(name = "Setpoint", importance = Importance.INFO)
  public Setpoint getCurrentSetpoint() {
    return currentSetpoint;
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
   * Run {@link Setpoint}
   *
   * @param setpoint {@link Setpoint} representing desired setpoint
   */
  public void applySetpoint(Setpoint setpoint) {
    this.currentSetpoint = setpoint;

    if (!isOverheated) {
      setpoint.applySetpoint(this);
    }
  }

  /** Enable brake mode */
  public abstract void enableBrake();

  /** Disable brake mode */
  public abstract void disableBrake();

  /** Stop all servo control */
  protected abstract void idle();

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
            io.idle();
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
     * Apply setpoint request to {@link ServoIO}
     *
     * @param io {@link ServoIO} to run {@link Setpoint}
     */
    void applySetpoint(ServoIO io) {
      applier.apply(io);
    }
  }
}
