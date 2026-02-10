// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.flywheel;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;

/**
 * {@link SubsystemBase} extension representing the flywheel subsystem
 *
 * <p>The {@link Flywheel} has two servos that drive it
 */
public class Flywheel extends SubsystemBase {
  private final ServoIO mLeft, mRight;

  private State mState = State.COASTING;

  private AngularVelocity mVelocitySetpoint = RevolutionsPerSecond.zero();
  private final double mEpsilon;

  /**
   * Create new {@link Flywheel}
   *
   * @param left {@link ServoIO} representing the left servo spinning flywheel
   * @param right {@link ServoIO} representing the right servo spinning flywheel
   * @param epsilon {@link Double} representing the largest acceptable amount of percent error of
   *     angular velocity from setpoint velocity
   */
  public Flywheel(ServoIO left, ServoIO right, double epsilon) {
    mLeft = left;
    mRight = right;
    mEpsilon = epsilon;

    mRight.setLeader(mLeft, true);
  }

  @Override
  public void periodic() {
    mLeft.periodic();
    mRight.periodic();

    // TODO state machine
  }

  /**
   * @return {@link State} representing the current mode of being subsystem is in
   */
  @Logged(name = "State", importance = Importance.INFO)
  public State getState() {
    return mState;
  }

  /**
   * @return {@link AngularVelocity} representing the angular velocity of flywheel
   */
  @Logged(name = "Angular Velocity", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return null;
  }

  /**
   * @return {@link AngularVelocity} representing the setpoint angular velocity of flywheel
   */
  @Logged(name = "Setpoint Angular Velocity", importance = Importance.INFO)
  public AngularVelocity getAngularVelocitySetpoint() {
    return mVelocitySetpoint;
  }

  /**
   * @return true when the percent error from the setpoint angular velocity is less than the epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return false;
  }

  /**
   * Attempt to coast flywheel
   *
   * @return {@link Command}
   */
  public Command coast() {
    return Commands.none();
  }

  /**
   * Attempt to accelerate flywheel to specified angular velocity continuously
   *
   * @param velocity {@link AngularVelocity} representing setpoint angular velocity to accelerate to
   * @return {@link Command}
   */
  public Command accelerateTo(AngularVelocity velocity) {
    return accelerateTo(() -> velocity);
  }

  /**
   * Attempt to accelerate flywheel to specified angular velocity continuously
   *
   * @param velocity {@link Supplier} of {@link AngularVelocity} representing setpoint angular
   *     velocity to accelerate to
   * @return {@link Command}
   */
  public Command accelerateTo(Supplier<AngularVelocity> velocity) {
    return Commands.none();
  }

  /** Represents a mode of being the {@link Flywheel} subsystem can be in */
  public static enum State {
    /** {@link State} where the {@link Flywheel} is freely spinning with no forces applied */
    COASTING,
    /** {@link State} where the {@link Flywheel} is accelerating to a specified setpoint */
    ACCELERATING,
    /** {@link State} where the {@link Flywheel} is spinning at a specified setpoint */
    CRUISING
  }
}
