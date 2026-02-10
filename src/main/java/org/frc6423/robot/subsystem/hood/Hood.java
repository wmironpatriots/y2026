// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.hood;

import static edu.wpi.first.units.Units.Revolutions;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;

/**
 * {@link SubsystemBase} extension representing the hood subsystem
 *
 * <p>The {@link Hood} has a single pivoting component that folds and unfolds the hood to launch
 * fuel at different angles
 *
 * <p>The {@link Hood} moving to a setpoint angle is referred as "adjusting"
 */
public class Hood extends SubsystemBase {
  private final ServoIO mServo;

  private State mState = State.STOWED;

  private Angle mSetpointAngle = Revolutions.zero();
  private final Angle mEpsilon;

  /**
   * Create new {@link Hood}
   *
   * @param servo {@link ServoIO} representing the servo pivoting hood
   * @param epsilon {@link Angle} representing the largest acceptable amount of angular position
   *     error
   */
  public Hood(ServoIO servo, Angle epsilon) {
    mServo = servo;
    mEpsilon = epsilon;
  }

  @Override
  public void periodic() {
    mServo.periodic();

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
   * @return {@link Angle} representing the angular position of hood
   */
  @Logged(name = "Angle", importance = Importance.INFO)
  public Angle getAngle() {
    return null;
  }

  /**
   * @return {@link Angle} representing the setpoint angular position
   */
  @Logged(name = "Setpoint", importance = Importance.INFO)
  public Angle getSetpointAngle() {
    return mSetpointAngle;
  }

  /**
   * @return true when the angular position error of hood is less than epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return false;
  }

  /**
   * Attempt to stow hood
   *
   * @return {@link Command}
   */
  public Command stow() {
    return Commands.none();
  }

  /**
   * Attempt to adjust hood to specified angle
   *
   * @param angle {@link Angle} representing setpoint angle to adjust to
   * @return {@link Command}
   */
  public Command adjustToAngle(Angle angle) {
    return adjustToAngle(() -> angle);
  }

  /**
   * Attempt to adjust hood to specified angle continuously
   *
   * @param angle {@link Supplier} of {@link Angle} representing setpoint angle to adjust to
   * @return {@link Command}
   */
  public Command adjustToAngle(Supplier<Angle> angle) {
    return Commands.none();
  }

  /** Represents a mode of being the {@link Hood} subsystem can be in */
  public static enum State {
    /** {@link State} where the {@link Hood} is completely folded */
    STOWED,
    /** {@link State} where the {@link Hood} is unfolding to a specified setpoint */
    ADJUSTING,
    /** {@link State} where the {@link Hood} is unfolded at a specified setpoint */
    ANGLED,
    /** {@link State} where the {@link Hood} is folding into a STOWED state */
    STOWING
  }
}
