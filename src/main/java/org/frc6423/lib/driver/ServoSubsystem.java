// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.driver;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;

/**
 * {@link MotorSubsystem} extension for a subsystem requiring precise positional control
 *
 * <p>A {@link ServoSubsystem} instance <strong>CANNOT</strong> run a Velocity {@link Setpoint}
 */
public class ServoSubsystem extends MotorSubsystem {
  public static final Angle kDefaultEpsilon = Degrees.of(0.4);

  public final Angle mEpsilon;

  /**
   * Create new {@link ServoSubsystem}
   *
   * @param servo {@link ServoIO} representing servo driving subsystem
   * @param epsilon {@link Angle} representing largest allowed error for angular position control
   */
  public ServoSubsystem(ServoIO servo, Angle epsilon) {
    super(servo);

    mEpsilon = epsilon;
  }

  /**
   * Create new coupled {@link ServoSubsystem}
   *
   * <p>A coupled servo subsystem is where one or more follower servos copy the actions of one
   * leader servo
   *
   * @param leader {@link ServoIO} representing leader servo
   * @param followers {@link ServoIO} array representing follower servos
   * @param followersFlipped {@link boolean} array storing the follow direction of each follower
   * @param epsilon {@link Angle} representing largest allowed error for angular position control
   */
  public ServoSubsystem(
      ServoIO leader, ServoIO[] followers, boolean[] followersFlipped, Angle epsilon) {
    super(leader, followers, followersFlipped);

    mEpsilon = epsilon;
  }

  /**
   * @return {@link Angle} representing the setpoint angular position of subsystem
   */
  public Angle getSetpointAngle() {
    return Rotations.of(getSetpoint().getValue());
  }

  /**
   * @return {@link Rotation2d} representing the angular position of subsystem
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(getAngle());
  }

  /**
   * @return true if error between subsystem angle and setpoint is less than epsilon of subsystem
   */
  public boolean isNearSetpoint() {
    return (!getSetpoint().getControlType().isVelocityControl())
        ? getSetpointAngle().isNear(getAngle(), mEpsilon)
        : false;
  }

  @Override
  protected Command followSetpointCmd(Supplier<Setpoint> setpointSupplier) {
    return super.followSetpointCmd(setpointSupplier)
        .onlyIf(() -> (!getSetpoint().getControlType().isVelocityControl()));
  }

  @Override
  protected Command setSetpointCmd(Setpoint setpoint) {
    return super.setSetpointCmd(setpoint)
        .onlyIf(() -> (!getSetpoint().getControlType().isVelocityControl()));
  }
}
