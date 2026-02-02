// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.superstructure.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the Intake subsystem of the robot, responsible for collecting and feeding fuel to
 * indexer
 *
 * <p>{@link Intake} has two components: the {@link IntakePivot} & {@link IntakeRoller}
 *
 * <p>{@link Intake} has five {@link State}s: STOWING, STOWED, DEPLOYING, DEPLOYED, UNJAMMING
 */
public class Intake extends SubsystemBase {
  @Logged private final IntakePivot mPivot;
  @Logged private final IntakeRoller mRoller;

  private State mState = State.STOWED;

  /**
   * Create new {@link Intake}
   *
   * @param pivot {@link IntakePivot} representing the pivoting component of subsystem
   * @param roller {@link IntakeRoller} representing the roller component of subsystem
   */
  public Intake(IntakePivot pivot, IntakeRoller roller) {
    mPivot = pivot;
    mRoller = roller;
  }

  /**
   * @return {@link State} representing the current action the subsystem is performing
   */
  @Logged(name = "State", importance = Importance.CRITICAL)
  public State getState() {
    return mState;
  }

  /**
   * Request subsystem to stop rolling and fold inwards
   *
   * @return {@link Command}
   */
  public Command stow() {
    return Commands.sequence(
        setState(State.STOWING),
        mRoller.stop(),
        mPivot
            .stow()
            .andThen(
                Commands.waitUntil(() -> mPivot.getState() == IntakePivot.State.STOWED)
                    .andThen(setState(State.DEPLOYED))));
  }

  /**
   * Request subsystem to start rolling and fold outwards
   *
   * @return {@link Command}
   */
  public Command deploy() {
    return Commands.sequence(
        setState(State.DEPLOYING),
        mRoller.startIntaking(),
        mPivot
            .stow()
            .andThen(
                Commands.waitUntil(() -> mPivot.getState() == IntakePivot.State.DEPLOYED)
                    .andThen(setState(State.DEPLOYED))));
  }

  /**
   * Request subsystem to run rollers opposite and rise slightly from a deployed state
   *
   * @return {@link Command}
   */
  public Command unjam() {
    return Commands.sequence(setState(State.UNJAMMING), mRoller.startOutaking(), mPivot.rise());
  }

  /**
   * Set current state value
   *
   * @param state {@link State}
   * @return {@link Command}
   */
  private Command setState(State state) {
    return this.runOnce(() -> mState = state);
  }

  /** Represents an action an {@link Intake} subsystem can do */
  public static enum State {
    /** {@link Intake} is stopping rollers and folding inwards */
    STOWING,
    /** {@link Intake} isn't rolling and fully folded inwards */
    STOWED,
    /** {@link Intake} is starting rollers and unfolding outwards */
    DEPLOYING,
    /** {@link Intake} is rolling and fully unfolded outwards */
    DEPLOYED,
    /** {@link Intake} is rolling opposite and slightly raised from deployed position */
    UNJAMMING
  }
}
