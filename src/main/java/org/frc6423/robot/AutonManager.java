// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * TODO
 *
 * <p>Replace Placeholders commands
 *
 * <p>Command Routines
 *
 * <p>Drive code integration (must merge first)
 *
 * <p>Javadocs
 */
public class AutonManager {
  private final SendableChooser<Routines> mChooser = new SendableChooser<>();

  /** Create new {@link AutonManager} */
  public AutonManager() {
    mChooser.setDefaultOption("Do Nothing", Routines.NOTHING);
    mChooser.addOption("DepotShot", Routines.DEPOT_SHOT);
    mChooser.addOption("NeutralShot", Routines.NEUTRAL_SHOT);
    mChooser.addOption("SourceShot", Routines.SOURCE_SHOT);

    SmartDashboard.putData(mChooser);
  }

  /**
   * @return {@link Command} representing the selected autonomous routine
   */
  public Command getAutonomousCommand() {
    if (mChooser.getSelected() == null) {
      return Commands.none();
    }

    switch (mChooser.getSelected()) {
      case DEPOT_SHOT:
        return getDepotShotRoutineCommand();
      case NEUTRAL_SHOT:
        return getNeutralShotRoutineCommand();
      case SOURCE_SHOT:
        return getSourceShotRoutineCommand();
      default:
        return Commands.none();
    }
  }

  /**
   * @return {@link Command} representing Depot Shot (placeholder until drive integration)
   */
  public Command getDepotShotRoutineCommand() {
    return Commands.none();
  }

  /**
   * @return {@link Command} representing Neutral Shot (placeholder until drive integration)
   */
  public Command getNeutralShotRoutineCommand() {
    return Commands.none();
  }

  /**
   * @return {@link Command} representing Source Shot (placeholder until drive integration)
   */
  public Command getSourceShotRoutineCommand() {
    return Commands.none();
  }

  /** Represents a possible routine during autonomous */
  public static enum Routines {
    NOTHING,
    DEPOT_SHOT,
    NEUTRAL_SHOT,
    SOURCE_SHOT
  }
}
