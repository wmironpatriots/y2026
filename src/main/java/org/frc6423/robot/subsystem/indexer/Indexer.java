// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.indexer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;

/**
 * {@link SubsystemBase} extension representing the indexer subsystem
 *
 * <p>This subsystem's only component is a roller
 *
 * <p>The roller will only spin towards the shooter/feeder
 */
public class Indexer extends SubsystemBase {
  @Logged private final ServoIO mServo;

  private State mState = State.STOPPED;

  /**
   * Create new {@link Indexer}
   *
   * @param servo {@link ServoIO} representing roller servo
   */
  public Indexer(ServoIO servo) {
    mServo = servo;
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
   * Attempt to stop roller
   *
   * @return {@link Command}
   */
  public Command stop() {
    return Commands.none();
  }

  /**
   * Attempt to run roller
   *
   * @return {@link Command}
   */
  public Command run() {
    return Commands.none();
  }

  /**
   * Attempt to start pulsing roller
   *
   * @return {@link Command}
   */
  public Command pulse() {
    return Commands.none();
  }

  /** Represents a mode of being the {@link Indexer} subsystem can be in */
  public static enum State {
    /** {@link State} where the {@link Indexer} is not running */
    STOPPED,
    /** {@link State} where the {@link Indexer} is running */
    RUNNING,
    /** {@link State} where the {@link Indexer} is periodically pulsing */
    PULSING
  }
}
