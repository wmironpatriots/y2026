// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.robot.RobotState;
import org.frc6423.robot.subsystem.drive.Drive;
import org.frc6423.robot.subsystem.flywheel.Flywheel;
import org.frc6423.robot.subsystem.hood.Hood;
import org.frc6423.robot.subsystem.indexer.Indexer;
import org.frc6423.robot.subsystem.intake.Intake;
import org.frc6423.robot.subsystem.led.LED;

/** {@link SubsystemBase} extension for control all subsystems */
public class Superstructure extends SubsystemBase {
  /** Represents a state of being robot can be in */
  public static enum State {
    PACKED,
    UNPACKING,
    IDLE
  }

  private final RobotState mRobotState = RobotState.getInstance();

  private final Drive mDrive;

  private final Intake mIntake;
  private final Indexer mIndexer;
  // TODO feeder
  private final Flywheel mFlywheel;
  private final Hood mHood;

  private final LED mLed;

  private State mState;
  private State mPreviousState;

  /**
   * Create new {@link Superstructure}
   *
   * @param drive {@link Drive} representing drive subsystem
   * @param intake {@link Intake} representing intake subsystem
   * @param indexer {@link Indexer} representing indexer subsystem
   * @param flywheel {@link Flywheel} representing flywheel subsystem
   * @param hood {@link Hood} representing hood subsystem
   * @param led {@link Led} representing LED subsystem
   */
  public Superstructure(
      Drive drive, Intake intake, Indexer indexer, Flywheel flywheel, Hood hood, LED led) {
    mDrive = drive;
    mIntake = intake;
    mIndexer = indexer;
    // TODO feeder
    mFlywheel = flywheel;
    mHood = hood;
    mLed = led;
  }

  @Override
  public void periodic() {}

  /**
   * @return {@link State} representing current action robot is doing
   */
  @Logged(name = "State", importance = Importance.INFO)
  public State getState() {
    return mState;
  }
}
