// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
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
  private final AutoFactory mAutoFactory;
  private final AutoChooser mAutoChooser;

  /** Create new {@link AutonManager} */
  public AutonManager() {
    mAutoChooser = new AutoChooser();
    mAutoFactory = new AutoFactory(null, null, null, false, null);

    mAutoChooser.addCmd("Do Nothing", this::doNothing);
    mAutoChooser.addCmd("DepotShot", this::depotShot);
    mAutoChooser.addCmd("NeutralShot", this::neutralShot);
    mAutoChooser.addCmd("SourceShot", this::sourceShot);

    SmartDashboard.putData(mAutoChooser);
  }

  /**
   * @return {@link Command} representing the selected autonomous routine
   */
  public Command getAutonomousCommand() {
    return mAutoChooser.selectedCommandScheduler();
  }

  private Command doNothing() {
    return Commands.none();
  }

  private Command depotShot() {
    return Commands.sequence(
        mAutoFactory.resetOdometry("DepotShot"), mAutoFactory.trajectoryCmd("DepotShot"));
  }

  private Command neutralShot() {
    return Commands.sequence(
        mAutoFactory.resetOdometry("NeutralShot"), mAutoFactory.trajectoryCmd("NeutralShot"));
  }

  private Command sourceShot() {
    return Commands.sequence(
        mAutoFactory.resetOdometry("SourceShot"), mAutoFactory.trajectoryCmd("SourceShot"));
  }

  /** Represents a possible routine during autonomous */
  public static enum Routines {
    NOTHING,
    DEPOT_SHOT,
    NEUTRAL_SHOT,
    SOURCE_SHOT
  }
}
