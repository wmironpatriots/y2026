// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Revolution;

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
import edu.wpi.first.units.measure.Torque;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

/** Interface for interacting with a swerve module hardware */
public abstract class SwerveModuleIO {
  public final String mName;

  protected final ModuleConfig mConfig;
  protected final DriveConstants mConstants;

  /**
   * Create new {@link SwerveModuleIO}
   *
   * @param name {@link String} Nickname for module
   * @param config {@link ModuleConfig} Configuration for module
   * @param constants {@link DriveConstants} Constants of drivetrain module is a part of
   */
  public SwerveModuleIO(String name, ModuleConfig config, DriveConstants constants) {
    mName = name;

    mConfig = config;
    mConstants = constants;
  }

  /** Updated logged values & hardware */
  public abstract void periodic();

  /**
   * Get the current position of the module
   *
   * @return {@link SwerveModulePosition}
   */
  @Logged(name = "SwerveModulePosition", importance = Importance.INFO)
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDriveDistance(), getRotation2d());
  }

  /**
   * Get the current state of the module
   *
   * @return {@link SwerveModuleState}
   */
  @Logged(name = "SwerveModuleState", importance = Importance.INFO)
  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getVelocity(), getRotation2d());
  }

  /**
   * Get angular position of module wheel
   *
   * @return {@link Rotation2d}
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(getPivotAngle());
  }

  /**
   * Get drive distance of module wheel
   *
   * @return {@link Distnace}
   */
  public Distance getDriveDistance() {
    return mConstants.getWheelRadius().times(getPivotAngle().in(Radians));
  }

  /**
   * Get linear velocity of module wheel
   *
   * @return {@link LinearVelocity}
   */
  public LinearVelocity getVelocity() {
    return FeetPerSecond.of(
        mConstants.getWheelRadius().times(getDriveAngularVelocity().in(RadiansPerSecond)).in(Feet));
  }

  /**
   * Get linear accleration of module wheel
   *
   * @return {@link LinearAcceleration}
   */
  @Logged(name = "Acceleration (feet per second)", importance = Importance.INFO)
  public LinearAcceleration getAcceleration() {
    // TODO create swervemodulestate but for accelerations, return that, and log it instead
    return FeetPerSecondPerSecond.of(
        mConstants
            .getWheelRadius()
            .times(getDriveAngularAcceleration().in(RadiansPerSecondPerSecond))
            .in(Feet));
  }

  /**
   * Set a {@link SwerveModuleState} setpoint and run/return an optimized mutate
   *
   * @param setpoint {@link SwerveModuleState} Desired module state
   * @param focEnabled {@link Boolean} Whether or not FOC based control should be used
   * @return {@link SwerveModuleState}
   */
  public SwerveModuleState setSetpointState(SwerveModuleState setpoint, boolean focEnabled) {
    // Mutate setpoint
    setpoint.optimize(getRotation2d());
    setpoint.cosineScale(getRotation2d());

    // Send Pivot Setpoint
    setPivotSetpoint(setpoint.angle.getMeasure());

    // Send Drive Setpoint
    var speed =
        RadiansPerSecond.of(
            MetersPerSecond.of(setpoint.speedMetersPerSecond)
                .div(mConstants.getWheelRadius().in(Meters))
                .baseUnitMagnitude());

    if (focEnabled) setDriveSetpoint(speed, true);
    else setDriveSetpoint(speed, false);

    // Returns optimized
    return setpoint;
  }

  /**
   * Set a {@link SwerveModuleState} setpoint & torque setpoint and run/return an optimized mutate
   *
   * @param setpoint {@link SwerveModuleState} Desired module state
   * @param torque {@link Torque} Desired wheel torque
   * @return {@link SwerveModuleState}
   */
  public SwerveModuleState setSetpointState(SwerveModuleState setpoint, Torque torque) {
    // Mutate setpoint
    setpoint.optimize(getRotation2d());
    setpoint.cosineScale(getRotation2d());

    // Send Pivot Setpoint
    setPivotSetpoint(setpoint.angle.getMeasure());

    // Send Drive Setpoint
    var speed =
        RadiansPerSecond.of(
            MetersPerSecond.of(setpoint.speedMetersPerSecond)
                .div(mConstants.getWheelRadius().in(Meters))
                .baseUnitMagnitude());
    setDriveSetpoint(speed, torque);

    // Returns optimized
    return setpoint;
  }

  // * ABSTRACT
  /**
   * Get absolute angular position of encoder
   *
   * @return {@link Angle}
   */
  protected abstract Angle getEncoderAngle();

  /**
   * Get angular position of pivot servo
   *
   * @return {@link Angle}
   */
  protected abstract Angle getPivotAngle();

  /**
   * Get temperature of pivot servo
   *
   * @return
   */
  @Logged(name = "Pivot Servo Temperature (Celsius)", importance = Importance.INFO)
  public abstract Temperature getPivotTemperature();

  /**
   * Get angular position of drive servo
   *
   * @return {@link Angle}
   */
  protected abstract Angle getDriveAngle();

  /**
   * Get angular velocity of drive servo
   *
   * @return {@link AngularVelocity}
   */
  protected abstract AngularVelocity getDriveAngularVelocity();

  /**
   * Get angular acceleration of drive servo
   *
   * @return {@link AngularAcceleration}
   */
  protected abstract AngularAcceleration getDriveAngularAcceleration();

  /**
   * Get temperature of drive servo
   *
   * @return {@link Temperature}
   */
  @Logged(name = "Drive Servo Temperature (Celsius)", importance = Importance.INFO)
  public abstract Temperature getDriveTemperature();

  /**
   * Run Open-Loop current output for pivot
   *
   * @param current {@link Current} Desired current output
   */
  public abstract void setPivotCurrent(Current current);

  /**
   * Set setpoint pivot angular position
   *
   * @param setpoint {@link Angle} Desired angular position
   */
  protected abstract void setPivotSetpoint(Angle setpoint);

  /**
   * Run Open-Loop current output for drive
   *
   * @param current {@link Current} Desired current ouput
   */
  public abstract void setDriveCurrent(Current current);

  /**
   * Set setpoint drive velocity
   *
   * @param velocity {@link AngularVelocity} Desired angular velocity
   * @param focEnabled {@link Boolean} Whether or not FOC based control should be used
   */
  protected abstract void setDriveSetpoint(AngularVelocity velocity, boolean focEnabled);

  /**
   * Set setpoint drive velocity & wheel torque
   *
   * @param velocity {@link AngularVelocity} Desired angular velocity
   * @param torque {@link Torque} Desired wheel torque
   */
  protected abstract void setDriveSetpoint(AngularVelocity velocity, Torque torque);

  /** Reset drive & pivot relative encoder angular positions */
  public void resetEncoders() {
    resetEncoders(Revolution.zero(), Revolution.zero());
  }

  /**
   * Reset drive & pivot relative encoder angular position to specified positions
   *
   * @param pivotAngle {@link Angle} Angle for pivot relative encoder to reset to
   * @param driveAngle {@link Angle} Angle for drive relative encoder to reset to
   */
  public abstract void resetEncoders(Angle pivotAngle, Angle driveAngle);

  /** Stop module completely */
  public abstract void stop();
}
