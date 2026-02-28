// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.frc6423.robot.Rebuilt;
import org.frc6423.robot.RobotState;

/** WIP */
public class Superstructure extends SubsystemBase {
  /** Constants for the {@link Superstructure} */
  public class Constants {}

  private final RobotState mRobotState = RobotState.getInstance();

  // * STATE MEMBERS
  private State mState = State.IDLE_NEUTRAL_ZONE;
  private Timer mTimer = new Timer();

  // * TRIGGERS
  @Logged
  private final Trigger mInNeutralZone =
      new Trigger(() -> Rebuilt.kNeutralZone.contains(mRobotState.getTranslation2d()));

  @Logged
  private final Trigger mInTrenchZone =
      new Trigger(() -> Rebuilt.kTrenchZone.contains(mRobotState.getTranslation2d()));

  @Logged
  private final Trigger mInAllianceZone =
      new Trigger(() -> Rebuilt.kRobotAllianceZone.contains(mRobotState.getTranslation2d()));

  // * DRIVER INPUT TRIGGERS
  @Logged
  private final Trigger mIntakeTrigger, mLeftAlign, mRightAlign, mSpinupTrigger, mFireTrigger;

  @Logged private final DoubleSupplier mXVelocitySupplier;
  @Logged private final DoubleSupplier mYVelocitySupplier;
  @Logged private final DoubleSupplier mOmegaSupplier;

  // * STATE MAP
  private final Map<State, Trigger> mStateMap = new HashMap<>();

  public Superstructure(
      Trigger intakeTrigger,
      Trigger leftAlignTrigger,
      Trigger rightAlignTrigger,
      Trigger spinupTrigger,
      Trigger fireTrigger,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier omegaSupplier) {
    // Define driver input triggers
    mIntakeTrigger = intakeTrigger;
    mLeftAlign = leftAlignTrigger;
    mRightAlign = rightAlignTrigger;
    mSpinupTrigger = spinupTrigger;
    mFireTrigger = fireTrigger;

    mXVelocitySupplier = xVelocitySupplier;
    mYVelocitySupplier = yVelocitySupplier;
    mOmegaSupplier = omegaSupplier;

    // Init state map
    for (var state : State.values()) {
      mStateMap.put(state, new Trigger(() -> mState == state));
    }

    // Define state machine behavior
    defineStateBehavior();
    defineStateTransitions();

    // Start internal timer
    mTimer.start();
  }

  /**
   * Get active state
   *
   * @return {@link State}
   */
  @Logged(name = "State", importance = Importance.INFO)
  public State getState() {
    return mState;
  }

  /** Define behavior during different states */
  private void defineStateBehavior() {}

  /** Define transitions between two states */
  private void defineStateTransitions() {
    // NEUTRAL ZONE TRANSITIONS
    createTransition(State.IDLE_NEUTRAL_ZONE, State.IDLE_ALLIANCE_ZONE, mInAllianceZone);

    createTransition(State.IDLE_NEUTRAL_ZONE, State.INTAKING, mIntakeTrigger);

    createTransition(State.IDLE_NEUTRAL_ZONE, State.PREPPING_FERRY, mSpinupTrigger);

    // FERRYING TRANSITIONS
    createTransition(
        State.PREPPING_FERRY,
        State.FERRYING,
        mFireTrigger.and(() -> false)); // TODO check for shooter ready

    createTransition(State.PREPPING_FERRY, State.IDLE_NEUTRAL_ZONE, mSpinupTrigger.negate());

    createTransition(
        State.FERRYING,
        State.PREPPING_FERRY,
        new Trigger(() -> false)); // TODO check for shooter not ready

    createTransition(
        State.FERRYING, State.IDLE_NEUTRAL_ZONE, mFireTrigger.negate().and(mInNeutralZone));

    createTransition(
        State.FERRYING, State.IDLE_ALLIANCE_ZONE, mFireTrigger.negate().and(mInAllianceZone));

    // ALLIANCE ZONE TRANSITIONS
    createTransition(State.IDLE_ALLIANCE_ZONE, State.IDLE_NEUTRAL_ZONE, mInNeutralZone);

    createTransition(State.IDLE_ALLIANCE_ZONE, State.INTAKING, mIntakeTrigger);

    createTransition(State.IDLE_ALLIANCE_ZONE, State.PREPPING_SCORE, mSpinupTrigger);

    // SCORING TRANSITIONS
    createTransition(
        State.PREPPING_SCORE,
        State.SCORING,
        mFireTrigger.and(() -> false)); // TODO check for shooter ready

    createTransition(State.PREPPING_SCORE, State.IDLE_NEUTRAL_ZONE, mSpinupTrigger.negate());

    createTransition(
        State.SCORING,
        State.PREPPING_FERRY,
        new Trigger(() -> false)); // TODO check for shooter not ready

    createTransition(
        State.SCORING, State.IDLE_NEUTRAL_ZONE, mFireTrigger.negate().and(mInNeutralZone));

    createTransition(
        State.SCORING, State.IDLE_ALLIANCE_ZONE, mFireTrigger.negate().and(mInAllianceZone));

    // INTAKING TRANSITIONS
    createTransition(
        State.INTAKING, State.IDLE_NEUTRAL_ZONE, mIntakeTrigger.negate().and(mInNeutralZone));

    createTransition(
        State.INTAKING, State.IDLE_ALLIANCE_ZONE, mIntakeTrigger.negate().and(mInAllianceZone));
  }

  /**
   * Create a transition between two states
   *
   * @param startingState {@link State} Starting state
   * @param endingState {@link State} Ending state
   * @param requirement {@link Trigger} Requirement for the state transition to happen
   */
  private void createTransition(State startingState, State endingState, Trigger requirement) {
    requirement.and(mStateMap.get(startingState)).onTrue(setState(endingState));
  }

  // * COMMANDS
  /**
   * Hard set current state to specified
   *
   * @param state {@link State} Desired state
   * @return {@link Command}
   */
  public Command setState(State state) {
    return Commands.runOnce(
            () -> {
              System.out.println("Transitioning " + mState.toString() + " to " + state.toString());
              mTimer.reset();
              mState = state;
            })
        .ignoringDisable(true)
        .withName("State Change");
  }

  /** A state of being a {@link Superstructure} can be in */
  public static enum State {
    /** {@link State} Superstructure is idle in neutral zone */
    IDLE_NEUTRAL_ZONE,
    /** {@link State} Superstructure is idle in alliance zone */
    IDLE_ALLIANCE_ZONE,
    /** {@link State} Superstucture is intaking */
    INTAKING,
    /** {@link State} Superstructure is spinning up for ferry */
    PREPPING_FERRY,
    /** {@link State} Superstructure is ferrying into alliance zone */
    FERRYING,
    /** {@link State} Superstructure is spinning up for scoring */
    PREPPING_SCORE,
    /** {@link State} Superstructure is firing into hub */
    SCORING,
  }
}
