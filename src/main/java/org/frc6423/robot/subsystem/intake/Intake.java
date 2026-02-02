// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

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

  public Command stowCmd() {
    return Commands.sequence(
        setStateCmd(State.STOWING),
        mRoller.idleCmd(),
        mPivot
            .stowCmd()
            .andThen(
                Commands.waitUntil(() -> mPivot.getState() == IntakePivot.State.STOWED)
                    .andThen(setStateCmd(State.DEPLOYED))));
  }

  public Command deployCmd() {
    return Commands.sequence(
        setStateCmd(State.DEPLOYING),
        mRoller.startIntakingCmd(),
        mPivot
            .stowCmd()
            .andThen(
                Commands.waitUntil(() -> mPivot.getState() == IntakePivot.State.DEPLOYED)
                    .andThen(setStateCmd(State.DEPLOYED))));
  }

  public Command unjamCmd() {
    return Commands.sequence(
        setStateCmd(State.UNJAMMING), mRoller.startOutakingCmd(), mPivot.riseCmd());
  }

  /**
   * Set current state value
   *
   * @param state {@link State}
   * @return {@link Command}
   */
  private Command setStateCmd(State state) {
    return this.runOnce(() -> mState = state);
  }

  /** Represents an action an {@link Intake} subsystem can do */
  public static enum State {
    STOWING,
    STOWED,
    DEPLOYING,
    DEPLOYED,
    UNJAMMING
  }
}
