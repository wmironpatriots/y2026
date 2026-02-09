// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.ServoIO;

/**
 * {@link SubsystemBase} extension representing the intake subsystem
 *
 * <p>AThe{@link Intake} has 3 components: the rollers, the pivot holding the rollers, and the
 * passive kicker mechanism that assists with intaking
 *
 * <p>The kicker starts in a stowed position at the start of the match and must be hit outwards by
 * the pivot; This action is set to automatically happen
 */
public class Intake extends SubsystemBase {
  @Logged private final EncoderIO mEncoder;
  @Logged private final ServoIO mPivot, mRoller;

  private State mState = State.FULLY_STOWED;
  private State mPreviousState = State.FULLY_STOWED;

  private final Angle mEpsilon;

  /**
   * Create new {@link Intake}
   *
   * @param pivot {@link ServoIO} representing pivot servo
   * @param encoder {@link EncoderIO} representing pivot abs encoder
   * @param roller {@link ServoIO} representing roller servo
   * @param epsilon {@link Angle} representing the largest acceptable amount of angular position
   *     error
   */
  public Intake(ServoIO pivot, EncoderIO encoder, ServoIO roller, Angle epsilon) {
    mPivot = pivot;
    mEncoder = encoder;
    mRoller = roller;
    mEpsilon = epsilon;
  }

  @Override
  public void periodic() {
    mEncoder.periodic();
    mPivot.periodic();
    mRoller.periodic();

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
   * @return {@link Angle} representing the angular position of pivot
   */
  @Logged(name = "Pivot Angle", importance = Importance.INFO)
  public Angle getAngle() {
    return null;
  }

  /**
   * @return true when the angular position error of pivot is less than epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return false;
  }

  /**
   * Attempt to stow intake
   *
   * @return {@link Command}
   */
  public Command stow() {
    return Commands.none();
  }

  /**
   * Attempt to deploy and start intaking
   *
   * @return {@link Command}
   */
  public Command intake() {
    return Commands.none();
  }

  /**
   * Attempt to deploy and start outaking
   *
   * @return {@link Command}
   */
  public Command outake() {
    return Commands.none();
  }

  /** Represents a mode of being the {@link Intake} subsystem can be in */
  public static enum State {
    /**
     * {@link State} where the {@link Intake} is folded and stopped while the kicker hasn't been
     * deployed
     */
    FULLY_STOWED,
    /** {@link State} where the {@link Intake} is hitting the kicker to passively deploy it */
    DEPLOYING_KICKER,
    /**
     * {@link State} where the {@link Intake} is folded & stopped while the kicker is passively
     * deployed
     */
    STOWED,
    /** {@link State} where the {@link Intake} is unfolding and starting */
    DEPLOYING,
    /** {@link State} where the {@link Intake} is unfolded and running */
    DEPLOYED_INTAKING,
    /** {@link State} where the {@link Intake} is unfolded and running inverse */
    DEPLOYED_OUTAKING,
    /** {@link State} where the {@link Intake} is attempting to stop and fold */
    STOWING
  }
}
