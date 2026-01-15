// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;

/**
 * Represents a Swerve Drive Module
 *
 * <p>@see {@link Drive.java}
 */
public class SwerveModule {
  private final SwerveModuleConfig config;

  private final ServoIO drive, pivot;
  private final EncoderIO encoder;

  private final Distance wheelRadius;

  private SwerveModuleState currentSetpoint;

  public SwerveModule(SwerveModuleConfig config) {
    this.config = config;

    this.pivot = config.pivot;
    this.drive = config.drive;
    this.encoder = config.encoder;

    this.wheelRadius = config.wheelRadius;
  }

  /** Update hardware */
  public void periodic() {
    drive.periodic();
    pivot.periodic();
  }

  /**
   * @return String representing the name of module
   */
  @Logged(name = "ID", importance = Importance.INFO)
  public String getId() {
    return config.id;
  }

  /**
   * @return {@link Rotation2d} representing the yaw of module
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return new Rotation2d(pivot.getAngle());
  }

  /**
   * @return {@link Distance} representing the distance driven by module
   */
  @Logged(name = "Distance", importance = Importance.INFO)
  public Distance getDistance() {
    return Meters.of(drive.getAngle().in(Radians) * wheelRadius.in(Meters));
  }

  /**
   * @return {@link LinearVelocity} representing the linear velocity of module
   */
  @Logged(name = "Linear Velocity", importance = Importance.INFO)
  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(
        drive.getAngularVelocity().in(RadiansPerSecond) * wheelRadius.in(Meters));
  }

  /**
   * @return {@link SwerveModulePosition} representing the positional state of module
   */
  @Logged(name = "SwerveModulePosition", importance = Importance.INFO)
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDistance(), getRotation2d());
  }

  /**
   * @return {@link SwerveModuleState} representing the setpoint velocity state of module
   */
  public SwerveModuleState getSetpointSwerveModuleState() {
    return currentSetpoint;
  }

  /**
   * @return {@link SwerveModuleState} representing the velocity state of module
   */
  @Logged(name = "SwerveModuleState", importance = Importance.INFO)
  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getLinearVelocity(), getRotation2d());
  }

  /**
   * Run module with a specified {@link SwerveModuleState} setpoint
   *
   * <p>setpoint will be mutated for optimization
   *
   * @param state {@link SwerveModuleState} setpoint to optimize and apply
   * @return {@link SwerveModuleState} (optimized)
   */
  public SwerveModuleState runSwerveModuleState(SwerveModuleState state, boolean focEnabled) {
    state.optimize(getRotation2d());
    state.cosineScale(getRotation2d());

    // TODO
    if (focEnabled) {
      drive.enableFoc();
    } else {
      drive.disableFoc();
    }

    var driveSetpoint =
        Setpoint.createProfiledVelocitySetpoint(
            RadiansPerSecond.of(state.speedMetersPerSecond / wheelRadius.in(Meters)));
    drive.applySetpoint(driveSetpoint);

    var pivotSetpoint = Setpoint.createProfiledPositionSetpoint(state.angle.getMeasure());
    pivot.applySetpoint(pivotSetpoint);

    currentSetpoint = state;

    return state;
  }

  /** Stop Module */
  public void stop() {
    pivot.stop();
  }

  /** Enable servo brakes */
  public void brake() {
    pivot.enableBrake();
    drive.enableBrake();
  }

  /** Enable servo coasting */
  public void coast() {
    pivot.disableBrake();
    drive.disableBrake();
  }

  /**
   * Represents a configuration for a {@link SwerveModule}
   *
   * @param id friendly "nickname" for module
   * @param pivot {@link ServoIO} representing pivot motor of module
   * @param drive {@link ServoIO} representing drive motor of module
   * @param encoder {@link EncoderIO} representing abs pivot encoder of module
   * @param wheelRadius {@link Distance} representing the wheel radius of module
   * @param displacementWrtCenter {@link Translation2d} representing displacement of module relative
   *     to chassis's center of rotation
   */
  public static final record SwerveModuleConfig(
      String id,
      ServoIO pivot,
      ServoIO drive,
      EncoderIO encoder,
      Distance wheelRadius,
      Translation2d displacementWrtCenter) {}
}
