// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

/** A hardware interface for controlling the hardware of a generic swerve module */
public abstract class SwerveModuleIO {
  /** Represents a method of control for swerve module drive */
  public static enum ControlMode {
    /** {@link ControlMode} representing motion profiled closed-loop control /w Torque FOC */
    CLOSED_LOOP_TORQUE_FOC,
    /** {@link ControlMode} representing motion profiled closed-loop control /w Voltage */
    CLOSED_LOOP_VOLT,
    /** {@link ControlMode} representing open-loop control /w Voltage FOC */
    OPEN_LOOP_VOLT_FOC,
    /** {@link ControlMode} representing open-loop control /w Voltage */
    OPEN_LOOP_VOLT
  }

  @Logged public final String mName;
  protected final ModuleConfig mConfig;
  protected final DriveConstants mDriveConstants;

  /**
   * Create new {@link SwerveModuleIO}
   *
   * @param name {@link String} representing a "nickname" for module
   * @param config {@link ModuleConfig} representing the configuration for module
   * @param driveConstants {@link DriveConstants} representing the constants of the overall
   *     drivetrain
   */
  public SwerveModuleIO(String name, ModuleConfig config, DriveConstants driveConstants) {
    mName = name;
    mConfig = config;
    mDriveConstants = driveConstants;
  }

  /** Update logged signals */
  public abstract void periodic();

  /**
   * @return {@link SwerveModulePosition} representing the displacement vector of module
   */
  @Logged(name = "SwerveModulePosition", importance = Importance.INFO)
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDistance(), getRotation2d());
  }

  /**
   * @return {@link SwerveModuleState} representing the velocity vector of module
   */
  @Logged(name = "SwerveModuleState", importance = Importance.INFO)
  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getVelocity(), getRotation2d());
  }

  /**
   * @return {@link Distance} representing the distance module has traveled
   */
  public Distance getDistance() {
    return mDriveConstants.getWheelRadius().times(getDriveAngle().in(Radians));
  }

  /**
   * @return {@link LinearVelocity} representing the velocity of the module
   */
  public LinearVelocity getVelocity() {
    return FeetPerSecond.of(
        mDriveConstants
            .getWheelRadius()
            .times(getDriveAngularVelocity().in(RadiansPerSecond))
            .in(Feet));
  }

  /**
   * @return {@link LinearAcceleration} representing the acceleration of the module
   */
  @Logged(name = "Linear Acceleration", importance = Importance.INFO)
  public LinearAcceleration getAcceleration() {
    return FeetPerSecondPerSecond.of(
        mDriveConstants
            .getWheelRadius()
            .times(getDriveAngularAcceleration().in(RadiansPerSecondPerSecond))
            .in(Feet));
  }

  /**
   * @return {@link Rotation2d} representing the angle of the module
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(getAbsEncoderAngle());
  }

  /**
   * Set a velocity vector setpoint to optimize and run
   *
   * @param setpoint {@link SwerveModuleState} representing the desired velocity vector
   * @param controlMode {@link ControlMode} representing the desired control method to use
   * @return {@link SwerveModuleState} representing setpoint post optimization
   */
  public SwerveModuleState setSetpoint(SwerveModuleState setpoint, ControlMode controlMode) {
    // Optimizations
    setpoint.optimize(getRotation2d());
    setpoint.cosineScale(getRotation2d());

    setPivotPositionSetpoint(setpoint.angle.getMeasure());
    var velocity = MetersPerSecond.of(setpoint.speedMetersPerSecond);

    // Voltage for open loop is calculated with the following equation:
    // (supplyVoltage) * (setpointVelocity/maxVelocity)
    var velocityVolts =
        Volts.of(
            velocity
                .times(12.0)
                .div(mDriveConstants.getMaxLinearVelocity().in(velocity.baseUnit()))
                .baseUnitMagnitude());

    // Apply drive setpoint
    switch (controlMode) {
      case CLOSED_LOOP_TORQUE_FOC:
        setDriveVelocitySetpoint(velocity, true);
        break;
      case CLOSED_LOOP_VOLT:
        setDriveVelocitySetpoint(velocity, false);
        break;
      case OPEN_LOOP_VOLT_FOC:
        setDriveVoltageSetpoint(velocityVolts, true);
        break;
      case OPEN_LOOP_VOLT:
        setDriveVoltageSetpoint(velocityVolts, false);
        break;
    }

    return setpoint;
  }

  /**
   * Set FOC Torque Current setpoint for pivot servo (open-loop control)
   *
   * @param current {@link Current} representing the desired torque output
   */
  protected abstract void setPivotTorqueCurrentFocSetpoint(Current current);

  /**
   * Set Motion Profiled Position setpoint for pivot servo (closed-loop control)
   *
   * @param position {@link Angle} representing the desired angular position
   */
  protected abstract void setPivotPositionSetpoint(Angle position);

  /**
   * Set Voltage setpoint for drive servo (open-loop control)
   *
   * @param voltage {@link Voltage} representing the desired voltage output
   * @param focEnabled when true, FOC control will be used
   */
  protected abstract void setDriveVoltageSetpoint(Voltage voltage, boolean focEnabled);

  /**
   * Set FOC Torque Current setpoint for drive servo (open-loop control)
   *
   * @param current {@link Current} representing the desired torque output
   */
  protected abstract void setDriveTorqueCurrentFocSetpoint(Current current);

  /**
   * Set Motion Profiled Velocity setpoint for drive servo (closed-loop control)
   *
   * @param velocity {@link LinearVelocity} representing the desired velocity output
   * @param focEnabled when true, FOC control will be used
   */
  protected abstract void setDriveVelocitySetpoint(LinearVelocity velocity, boolean focEnabled);

  /**
   * @return {@link Angle} representing the absolute angular position of module measured by an
   *     encoder
   */
  @Logged(name = "Encoder Angle", importance = Importance.INFO)
  public abstract Angle getAbsEncoderAngle();

  /**
   * @return {@link Voltage} representing the applied voltage of pivot servo
   */
  @Logged(name = "Applied Voltage", importance = Importance.INFO)
  public abstract Voltage getPivotAppliedVoltage();

  /**
   * @return {@link Current} representing the input supply current of pivot servo
   */
  @Logged(name = "Supply Current", importance = Importance.INFO)
  public abstract Current getPivotSupplyCurrent();

  /**
   * @return {@link Current} representing the output stator current of pivot servo
   */
  @Logged(name = "Stator Current", importance = Importance.INFO)
  public abstract Current getPivotStatorCurrent();

  /**
   * @return {@link Current} representing the torque current output of pivot servo
   */
  @Logged(name = "Torque Current", importance = Importance.INFO)
  public abstract Current getPivotTorqueCurrent();

  /**
   * @return {@link Angle} representing the angular position of pivot servo
   */
  @Logged(name = "Angle", importance = Importance.INFO)
  public abstract Angle getPivotAngle();

  /**
   * @return {@link Temperature} representing the temperature of pivot servo
   */
  @Logged(name = "Temperature", importance = Importance.INFO)
  public abstract Temperature getPivotTemperature();

  /**
   * @return {@link Voltage} representing the applied voltage of drive servo
   */
  @Logged(name = "Applied Voltage", importance = Importance.INFO)
  public abstract Voltage getDriveAppliedVoltage();

  /**
   * @return {@link Current} representing the input supply current of drive servo
   */
  @Logged(name = "Supply Current", importance = Importance.INFO)
  public abstract Current getDriveSupplyCurrent();

  /**
   * @return {@link Current} representing the output stator current of drive servo
   */
  @Logged(name = "Stator Current", importance = Importance.INFO)
  public abstract Current getDriveStatorCurrent();

  /**
   * @return {@link Current} representing the torque current output of drive servo
   */
  @Logged(name = "Torque Current", importance = Importance.INFO)
  public abstract Current getDriveTorqueCurrent();

  /**
   * @return {@link Angle} representing the angular position of drive servo
   */
  @Logged(name = "Angle", importance = Importance.INFO)
  public abstract Angle getDriveAngle();

  /**
   * @return {@link Temperature} representing the temperature of drive servo
   */
  @Logged(name = "Temperature", importance = Importance.INFO)
  public abstract Temperature getDriveTemperature();

  /**
   * @return {@link AngularVelocity} representing the angular velocity of drive servo
   */
  @Logged(name = "Angular Velocity", importance = Importance.INFO)
  public abstract AngularVelocity getDriveAngularVelocity();

  /**
   * @return {@link AngularAcceleration} representing the angular acceleration of drive servo
   */
  @Logged(name = "Angular Acceleration", importance = Importance.INFO)
  public abstract AngularAcceleration getDriveAngularAcceleration();
}
