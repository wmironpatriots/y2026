// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.frc6423.robot.subsystem.drive.constants.SwerveConstants.ModuleConfig;

// TODO acceleration?
/** Interface for interacting with gyro hardware */
public abstract class SwerveModuleIO {
  protected final ModuleConfig mConfig;

  protected SwerveModuleIO(ModuleConfig config) {
    mConfig = config;
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Get wheel position of module on field
   *
   * @return {@link SwerveModulePosition}
   */
  public SwerveModulePosition getWheelPosition() {
    return new SwerveModulePosition(getDriveDistanceMeters(), getRotation2d());
  }

  /**
   * Get wheel state (velocity vector) of module on field
   *
   * @return {@link SwerveModuleState}
   */
  public SwerveModuleState getWheelState() {
    return new SwerveModuleState(getDriveSpeedMetersPerSec(), getRotation2d());
  }

  /**
   * Get rotation of wheel
   *
   * @return {@link Rotation2d}
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(getPivotAngleRevs());
  }

  /**
   * Get distance driven by wheel
   *
   * @return {@link Double}
   */
  public double getDriveDistanceMeters() {
    return mConfig.wheelRadiusMeters() * getDriveAngleRevs() * 2 * Math.PI;
  }

  /**
   * Get speed wheel is traveling at
   *
   * @return {@link Double}
   */
  public double getDriveSpeedMetersPerSec() {
    return mConfig.wheelRadiusMeters() * getDriveAngularSpeedRevsPerSec() * 2 * Math.PI;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  /**
   * Set a wheel state setpoint for module to achive
   *
   * <p>This method will automatically optimize setpoints before applying
   *
   * <p>This method will return the optimized setpoint
   *
   * @param setpoint {@link SwerveModuleState} Desired wheel state
   * @param focEnabled {@link Boolean} Whether Field-Oriented Control should be used to reach
   *     setpoint
   * @return {@link SwerveModuleState}
   */
  public SwerveModuleState setSetpoint(SwerveModuleState setpoint, boolean focEnabled) {
    // Mutate setpoint
    setpoint.optimize(getRotation2d());
    setpoint.cosineScale(getRotation2d());

    // Send Pivot Setpoint
    setPivotAngleSetpoint(setpoint.angle.getRotations());

    // Send Drive Setpoint
    var speed = setpoint.speedMetersPerSecond / mConfig.wheelRadiusMeters() / (2 * Math.PI);

    if (focEnabled) setDriveSpeedSetpoint(speed, true);
    else setDriveSpeedSetpoint(speed, false);

    // Returns optimized
    return setpoint;
  }

  /**
   * Set a wheel state setpoint for module to achive
   *
   * <p>This method will automatically optimize setpoints before applying
   *
   * <p>This method will return the optimized setpoint
   *
   * @param setpoint {@link SwerveModuleState} Desired wheel state
   * @param torqueNm (@link Double) Desired wheel torque in Newton-Meters
   * @return {@link SwerveModuleState}
   */
  public SwerveModuleState setSetpoint(SwerveModuleState setpoint, double torqueNm) {
    // Mutate setpoint
    setpoint.optimize(getRotation2d());
    setpoint.cosineScale(getRotation2d());

    // Send Pivot Setpoint
    setPivotAngleSetpoint(setpoint.angle.getRotations());

    // Send Drive Setpoint
    var speed = setpoint.speedMetersPerSecond / mConfig.wheelRadiusMeters() / (2 * Math.PI);
    setDriveSpeedSetpoint(speed, torqueNm);

    // Returns optimized
    return setpoint;
  }

  // * ~~~~~~~~ ABSTRACT ~~~~~~~~

  @Logged(name = "Pivot Applied Voltage (volts)", importance = Importance.DEBUG)
  public abstract double getPivotAppliedVolts();

  @Logged(name = "Pivot Supply Current (amps)", importance = Importance.DEBUG)
  public abstract double getPivotSupplyCurrentAmps();

  @Logged(name = "Pivot Stator Current (amps)", importance = Importance.DEBUG)
  public abstract double getPivotStatorCurrentAmps();

  @Logged(name = "Pivot Torque Current (amps)", importance = Importance.DEBUG)
  public abstract double getPivotSupplyTorqueAmps();

  @Logged(name = "Pivot Temperature (celsius)", importance = Importance.INFO)
  public abstract double getPivotTemperatureCelsius();

  public abstract double getPivotAngleRevs();

  public abstract void runPivotCharacterizationVoltage(double volts);

  public abstract void runPivotCharacterizationCurrent(double amps);

  protected abstract void setPivotAngleSetpoint(double angleRevs);

  @Logged(name = "Drive Applied Voltage (volts)", importance = Importance.DEBUG)
  public abstract double getDriveAppliedVolts();

  @Logged(name = "Drive Supply Current (amps)", importance = Importance.DEBUG)
  public abstract double getDriveSupplyCurrentAmps();

  @Logged(name = "Drive Stator Current (amps)", importance = Importance.DEBUG)
  public abstract double getDriveStatorCurrentAmps();

  @Logged(name = "Drive Torque Current (amps)", importance = Importance.DEBUG)
  public abstract double getDriveSupplyTorqueAmps();

  @Logged(name = "Drive Temperature (celsius)", importance = Importance.INFO)
  public abstract double getDriveTemperatureCelsius();

  public abstract double getDriveAngleRevs();

  public abstract double getDriveAngularSpeedRevsPerSec();

  public abstract void runDriveCharacterizationVoltage(double volts);

  public abstract void runDriveCharacterizationCurrent(double amps);

  protected abstract void setDriveSpeedSetpoint(double speedRevsPerSec, boolean focEnabled);

  protected abstract void setDriveSpeedSetpoint(double speedRevsPerSec, double torqueNm);

  /** Set all servos to neutral mode */
  public abstract void neutral();

  public abstract SwerveModulePosition[] getWheelPositions();

  public abstract double[] getPositions();

  public abstract double[] getRotations();

  public abstract double[] getTimestamps();
}
