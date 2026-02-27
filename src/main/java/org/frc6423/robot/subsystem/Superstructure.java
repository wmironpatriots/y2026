// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.subsystem.drive.Drive;
import org.frc6423.robot.subsystem.feeder.Feeder;
import org.frc6423.robot.subsystem.flywheel.Flywheel;
import org.frc6423.robot.subsystem.hood.Hood;
import org.frc6423.robot.subsystem.indexer.Indexer;
import org.frc6423.robot.subsystem.intake.Intake;

/** {@link SubsystemBase} Manager Subsystem for controlling all other subsystem in sequence */
public class Superstructure extends SubsystemBase {
  private final RobotState mRobotState = new RobotState();

  // * SUBSYSTEM MEMBERS
  private final Drive mDrive = new Drive(mRobotState);
  private final Intake mIntake = Intake.create();
  private final Indexer mIndexer = Indexer.create();
  private final Feeder mFeeder = Feeder.create();
  private final Hood mHood = Hood.create();
  private final Flywheel mFlywheel = Flywheel.create();

  // * STATE MEMBERS
  private State mState = State.IDLE_NEUTRAL_ZONE;
  private State mPreviouState = mState;

  // * DRIVER TRIGGERS
  private final Trigger mIntakeTrigger;
  private final Trigger mAlignTrigger;
  private final Trigger mSpinupTrigger;
  private final Trigger mFireTrigger;

  private final DoubleSupplier mXVelSupplier;
  private final DoubleSupplier mYVelSupplier;
  private final DoubleSupplier mOmegaSupplier;

  // * STATE MAP
  private final Map<State, Trigger> mStateMap = new HashMap<>();

  /**
   * Create new {@link Superstructure}
   *
   * @param intakeTrigger {@link Trigger} Trigger for intaking
   * @param actionTrigger {@link Trigger} Trigger for turning on intake align
   * @param spinupTrigger {@link Trigger} Trigger for spinning up flywheel
   * @param fireTrigger {@link Trigger} Trigger for firing fuel
   * @param xVelSupplier {@link DoubleSupplier} Stream of requested drive x velocity magnitudes
   * @param yVelSupplier {@link DoubleSupplier} Stream of requested drive y velocity magnitudes
   * @param omegaSupplier {@link DoubleSupplier} Stream of request drive omega magnitudes
   */
  public Superstructure(
      Trigger intakeTrigger,
      Trigger alignTrigger,
      Trigger spinupTrigger,
      Trigger fireTrigger,
      DoubleSupplier xVelSupplier,
      DoubleSupplier yVelSupplier,
      DoubleSupplier omegaSupplier) {
    mIntakeTrigger = intakeTrigger;
    mAlignTrigger = alignTrigger;
    mSpinupTrigger = spinupTrigger;
    mFireTrigger = fireTrigger;

    mXVelSupplier = xVelSupplier;
    mYVelSupplier = yVelSupplier;
    mOmegaSupplier = omegaSupplier;

    for (var state : State.values()) {
      mStateMap.put(state, new Trigger(() -> mState == state));
    }

    defineStateBehavior();
  }

  /** Define behavior during different states */
  // TODO
  private void defineStateBehavior() {
    // * IDLE NEUTRAL ZONE
    mStateMap
        .get(State.IDLE_NEUTRAL_ZONE)
        .whileTrue(
            Commands.parallel(
                mIntake.stow(), mIndexer.stop(), mFeeder.stop(), mHood.stow(), mFlywheel.coast()));

    // * IDLE ALLIANCE ZONE
    mStateMap
        .get(State.IDLE_NEUTRAL_ZONE)
        .whileTrue(
            Commands.parallel(
                mIntake.stow(), mIndexer.stop(), mFeeder.stop(), mHood.stow(), mFlywheel.coast()));
  }

  /** A state of being a {@link Superstructure} can be in */
  // TODO
  public static enum State {
    IDLE_NEUTRAL_ZONE,
    IDLE_ALLIANC_ZONE,
  }
}
