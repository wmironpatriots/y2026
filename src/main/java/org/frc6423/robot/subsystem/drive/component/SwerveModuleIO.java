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
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.Drive;
import org.frc6423.robot.subsystem.drive.constants.SwerveConstants.ModuleConfig;

/** Interface for interacting with gyro hardware */
public abstract class SwerveModuleIO {

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  public static final TunableNumber kPivotKs =
      new TunableNumber(Drive.kTunablesPrefix + "/module/Pivot kS");
  public static final TunableNumber kPivotKv =
      new TunableNumber(Drive.kTunablesPrefix + "/module/Pivot kV");
  public static final TunableNumber kPivotKa =
      new TunableNumber(Drive.kTunablesPrefix + "/module/Pivot kA");
  public static final TunableNumber kPivotKp =
      new TunableNumber(Drive.kTunablesPrefix + "/module/Pivot kP");
  public static final TunableNumber kPivotKd =
      new TunableNumber(Drive.kTunablesPrefix + "/module/Pivot kD");

  public static final TunableNumber kDriveTorqueKs =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Torque kS");
  public static final TunableNumber kDriveTorqueKv =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Torque kV");
  public static final TunableNumber kDriveTorqueKa =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Torque kA");
  public static final TunableNumber kDriveTorqueKp =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Torque kP");
  public static final TunableNumber kDriveTorqueKd =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Torque kD");

  public static final TunableNumber kDriveVoltKs =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Volt kS");
  public static final TunableNumber kDriveVoltKv =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Volt kV");
  public static final TunableNumber kDriveVoltKa =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Volt kA");
  public static final TunableNumber kDriveVoltKp =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Volt kP");
  public static final TunableNumber kDriveVoltKd =
      new TunableNumber(Drive.kTunablesPrefix + "/modules/Drive Volt kD");

  static {
    if (Robot.isReal()) {
      kPivotKs.initDefault(0.014);
      kPivotKv.initDefault(10.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(600.0);
      kPivotKd.initDefault(50.0);

      kDriveTorqueKs.initDefault(0.2);
      kDriveTorqueKv.initDefault(0.0);
      kDriveTorqueKa.initDefault(0.0);
      kDriveTorqueKp.initDefault(12.0);
      kDriveTorqueKd.initDefault(0.0);

      kDriveVoltKs.initDefault(10.0);
      kDriveVoltKv.initDefault(5.0);
      kDriveVoltKa.initDefault(0.0);
      kDriveVoltKp.initDefault(300.0);
      kDriveVoltKd.initDefault(0.0);
    } else {
      kPivotKs.initDefault(0.0);
      kPivotKv.initDefault(0.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(10.0);
      kPivotKd.initDefault(0.0);

      kDriveTorqueKs.initDefault(0.014);
      kDriveTorqueKv.initDefault(0.134);
      kDriveTorqueKa.initDefault(0.0);
      kDriveTorqueKp.initDefault(0.01);
      kDriveTorqueKd.initDefault(0.02);

      kDriveVoltKs.initDefault(0.0);
      kDriveVoltKv.initDefault(0.0);
      kDriveVoltKa.initDefault(0.0);
      kDriveVoltKp.initDefault(0.0);
      kDriveVoltKd.initDefault(0.0);
    }
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

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

  /** Update all status signals */
  public void periodic() {
    if (kPivotKs.hasChanged(hashCode())
        || kPivotKv.hasChanged(hashCode())
        || kPivotKa.hasChanged(hashCode())
        || kPivotKp.hasChanged(hashCode())
        || kPivotKd.hasChanged(hashCode())) {
      setPivotGains(kPivotKs.get(), kPivotKv.get(), kPivotKa.get(), kPivotKp.get(), kPivotKd.get());
    }

    if (kDriveTorqueKs.hasChanged(hashCode())
        || kDriveTorqueKv.hasChanged(hashCode())
        || kDriveTorqueKa.hasChanged(hashCode())
        || kDriveTorqueKp.hasChanged(hashCode())
        || kDriveTorqueKd.hasChanged(hashCode())) {
      setDriveTorqueGains(
          kDriveTorqueKs.get(),
          kDriveTorqueKv.get(),
          kDriveTorqueKa.get(),
          kDriveTorqueKp.get(),
          kDriveTorqueKd.get());
    }

    if (kDriveVoltKs.hasChanged(hashCode())
        || kDriveVoltKv.hasChanged(hashCode())
        || kDriveVoltKa.hasChanged(hashCode())
        || kDriveVoltKp.hasChanged(hashCode())
        || kDriveVoltKd.hasChanged(hashCode())) {
      setDriveVoltGains(
          kDriveVoltKs.get(),
          kDriveVoltKv.get(),
          kDriveVoltKa.get(),
          kDriveVoltKp.get(),
          kDriveVoltKd.get());
    }
  }

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

  protected abstract void setPivotGains(double kS, double kV, double kA, double kP, double kD);

  protected abstract void setDriveTorqueGains(
      double kS, double kV, double kA, double kP, double kD);

  protected abstract void setDriveVoltGains(double kS, double kV, double kA, double kP, double kD);

  /** Set all servos to neutral mode */
  public abstract void neutral();
}
