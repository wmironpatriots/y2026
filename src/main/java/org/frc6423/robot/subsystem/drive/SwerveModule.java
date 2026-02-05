// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxSim;
import org.frc6423.lib.sim.FlywheelSim;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

/** Represents a Swerve Module for {@link Drive} Subsystem */
public class SwerveModule extends SubsystemBase {
  private final ModuleConfig mConfig;
  private final DriveConstants mDriveConstants;

  @Logged private final ServoIO mPivot, mDrive;
  @Logged private final EncoderIO mEncoder;

  public SwerveModule(ModuleConfig config, DriveConstants constants) {
    mConfig = config;
    mDriveConstants = constants;

    if (Robot.isReal()) {
      mPivot =
          new ServoIOTalonFx(
              config.name(), config.pivotDeviceId(), config.canBus(), config.pivotConfig());
      mDrive =
          new ServoIOTalonFx(
              config.name(), config.driveDeviceId(), config.canBus(), config.driveConfig());
      mEncoder =
          new EncoderIOCanCoder(config.cancoderId(), config.canBus(), config.cancoderConfig());
    } else {
      mPivot =
          new ServoIOTalonFxSim(
              config.name() + "Pivot",
              config.pivotDeviceId(),
              config.canBus(),
              config.pivotConfig(),
              new FlywheelSim(
                  new FlywheelSim.Config(
                      DCMotor.getKrakenX60Foc(2), 0.005, KilogramSquareMeters.of(0.004))));
      mDrive =
          new ServoIOTalonFxSim(
              config.name() + "Drive",
              config.driveDeviceId(),
              config.canBus(),
              config.driveConfig(),
              new FlywheelSim(
                  new FlywheelSim.Config(
                      DCMotor.getKrakenX60Foc(2), 0.005, KilogramSquareMeters.of(0.025))));
      mEncoder =
          new EncoderIOCanCoder(config.cancoderId(), config.canBus(), config.cancoderConfig());
    }
  }

  @Override
  public void periodic() {
    mPivot.periodic();
    mDrive.periodic();
    mEncoder.periodic();
  }

  /**
   * @return {@link String} representing "nickname" of module
   */
  @Logged(name = "Module Name", importance = Importance.INFO)
  public String getName() {
    return mConfig.name();
  }

  /**
   * @return {@link SwerveModuleState} representing the velocity vector of module wheel
   */
  @Logged(name = "Module State", importance = Importance.INFO)
  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getLinearVelocity(), getRotation2d());
  }

  /**
   * @return {@link SwerveModulePosition} representing the position of module wheel
   */
  @Logged(name = "Module Pose", importance = Importance.INFO)
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getLinearDistance(), getRotation2d());
  }

  /**
   * @return {@link Rotation2d} representing the angle of module wheel
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(getAngle());
  }

  /**
   * @return {@link Angle} representing the angle of module wheel
   */
  public Angle getAngle() {
    return Robot.isReal() ? mEncoder.getAngle() : mPivot.getAngle();
  }

  /**
   * @return {@link Distance} representing the distance traveled by module wheel
   */
  public Distance getLinearDistance() {
    return mDriveConstants.getWheelRadius().times(mDrive.getAngle().in(Radians));
  }

  /**
   * @return {@link LinearVelocity} representing the velocity of module wheel
   */
  public LinearVelocity getLinearVelocity() {
    // I think this should work
    return mDriveConstants
        .getWheelRadius()
        .times(mDrive.getAngularVelocity().in(RadiansPerSecond))
        .per(Seconds);
  }

  /**
   * @return {@link LinearAcceleration} representing the acceleration of module wheel
   */
  public LinearAcceleration getLinearAcceleration() {
    // deja vu
    return mDriveConstants
        .getWheelRadius()
        .times(mDrive.getAngularAcceleration().in(RadiansPerSecondPerSecond))
        .per(Seconds)
        .per(Seconds);
  }

  /**
   * Run a voltage setpoint for drive servo
   *
   * @param voltage {@link Voltage} representing voltage output
   * @param focEnabled when true, drive motor will run voltage setpoint with foc enabled
   */
  public void setDriveAppliedVoltage(Voltage voltage, boolean focEnabled) {
    mDrive.setFocStatus(focEnabled);
    mDrive.applySetpoint(Setpoint.createVoltageSetpoint(voltage));
  }

  /**
   * Optimizes and sets a {@link SwerveModuleState} setpoint
   *
   * @param setpoint {@link SwerveModuleState} representing unoptimized setpoint
   * @param controlMode {@link ControlMode} representing the control mode module should use to
   *     achieve setpoint
   * @return {@link SwerveModuleState} representing optimized setpoint
   */
  public SwerveModuleState setSwerveModuleStateSetpoint(
      SwerveModuleState setpoint, ControlMode controlMode) {
    setpoint.optimize(getRotation2d());
    setpoint.cosineScale(getRotation2d());

    setPivotSetpoint(getRotation2d());

    var speed = MetersPerSecond.of(setpoint.speedMetersPerSecond);

    // Open-loop voltage is calculated by multiplying the supply voltage by setpointSpeed / maxSpeed
    // (percentage of max speed requested)
    var driveVolts =
        Volts.of(
            speed
                .times(RobotController.getBatteryVoltage())
                .div(mDriveConstants.getMaxLinearVelocity().in(MetersPerSecond))
                .baseUnitMagnitude());

    switch (controlMode) {
      case CLOSED_LOOP_TORQUE_FOC:
        setDriveSetpoint(speed, true);
        break;
      case CLOSED_LOOP_VOLTAGE:
        setDriveSetpoint(speed, false);
        break;
      case OPEN_LOOP_VOLTAGE_FOC:
        setDriveAppliedVoltage(driveVolts, true);
        break;
      case OPEN_LOOP_VOLTAGE:
        setDriveAppliedVoltage(driveVolts, false);
        break;
    }

    return setpoint;
  }

  /**
   * Set module pivot motor angle setpoint
   *
   * @param setpoint {@link Rotation2d} representing setpoint angle
   */
  public void setPivotSetpoint(Rotation2d setpoint) {
    mPivot.applySetpoint(Setpoint.createProfiledPositionSetpoint(setpoint.getMeasure()));
  }

  /**
   * Set module drive motor velocity setpoint
   *
   * @param setpoint {@link LinearVelocity} representing setpoint velocity
   * @param focEnabled when true, drive motor will run setpoint with foc enabled
   */
  public void setDriveSetpoint(LinearVelocity setpoint, boolean focEnabled) {
    mDrive.setFocStatus(focEnabled);
    var angularSetpoint =
        RadiansPerSecond.of(
            setpoint.div(mDriveConstants.getWheelRadius().in(Meters)).baseUnitMagnitude());

    mDrive.applySetpoint(Setpoint.createProfiledVelocitySetpoint(angularSetpoint));
  }

  /** Represents a servo control method module can use to reach setpoint */
  public static enum ControlMode {
    CLOSED_LOOP_TORQUE_FOC,
    CLOSED_LOOP_VOLTAGE,
    OPEN_LOOP_VOLTAGE_FOC,
    OPEN_LOOP_VOLTAGE
  }
}
