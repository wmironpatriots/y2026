// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.driver;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;

/** {@link MotorSubsystem} extension for a subsystem requiring precise velocity control */
public class FlywheelSubsystem extends MotorSubsystem {
  public static final double kDefaultEpsilonPercentage = 0.03;

  public final double mEpsilon;

  /**
   * Create new {@link FlywheelSubsystem}
   *
   * @param servo {@link ServoIO} representing servo driving subsystem
   * @param epsilon largest allowed percent error for angular velocity control
   */
  public FlywheelSubsystem(ServoIO servo, double epsilon) {
    super(servo);

    mEpsilon = epsilon;
  }

  /**
   * Create new coupled {@link FlywheelSubsystem}
   *
   * <p>A coupled servo subsystem is where one or more follower servos copy the actions of one
   * leader servo
   *
   * @param leader {@link ServoIO} representing leader servo
   * @param followers {@link ServoIO} array representing follower servos
   * @param followersFlipped {@link boolean} array storing the follow direction of each follower
   * @param epsilon largest allowed percent error for angular velocity control
   */
  public FlywheelSubsystem(
      ServoIO leader, ServoIO[] followers, boolean[] followersFlipped, double epsilon) {
    super(leader, followers, followersFlipped);

    mEpsilon = epsilon;
  }

  /**
   * @return {@link AngularVelocity} representing the setpoint angular velocity of subsystem
   */
  public AngularVelocity getSetpointAngularVelocity() {
    return RevolutionsPerSecond.of(getSetpoint().getValue());
  }

  /**
   * @return {@link AngularVelocity} representing the error between the setpoint angular velocity
   *     and the angular velocity of subsystem
   */
  public AngularVelocity getError() {
    // This feels wrong for some reason . . .
    return getSetpointAngularVelocity().gt(getAngularVelocity())
        ? getSetpointAngularVelocity().minus(getAngularVelocity())
        : getAngularVelocity().minus(getSetpointAngularVelocity());
  }

  /**
   * @return true if percent error between subsystem angular velocity and setpoint angular velocity
   *     is less than epsilon of subsystem
   */
  public boolean isNearSetpoint() {
    return (!getSetpoint().getControlType().isPositionControl())
        ? getSetpointAngularVelocity().times(mEpsilon).lte(getError())
        : false;
  }

  @Override
  protected Command followSetpointCmd(Supplier<Setpoint> setpointSupplier) {
    return super.followSetpointCmd(setpointSupplier)
        .onlyIf(() -> (!getSetpoint().getControlType().isPositionControl()));
  }

  @Override
  protected Command setSetpointCmd(Setpoint setpoint) {
    return super.setSetpointCmd(setpoint)
        .onlyIf(() -> (!getSetpoint().getControlType().isPositionControl()));
  }
}
