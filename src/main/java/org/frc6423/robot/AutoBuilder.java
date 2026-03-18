// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc6423.robot.fcs.FireControlSystem;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;
import org.frc6423.robot.subsystem.feeder.FeederSubsystem;
import org.frc6423.robot.subsystem.indexer.IndexerSubsystem;
import org.frc6423.robot.subsystem.intake.IntakeSubsystem;
import org.frc6423.robot.subsystem.shooter.ShooterSubsystem;

public class AutoBuilder {
  // * ~~~~~~~~ COMMON ~~~~~~~~

  private static Command empty(
      AutoRoutine routine,
      DriveSubsystem drive,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      ShooterSubsystem shooter) {
    return Commands.parallel(
            drive.driveTeleoperatedFacingTarget(
                () -> 0.0, () -> 0.0, () -> FireControlSystem.getVirtualTarget(), true),
            shooter.runSetpoint(
                () ->
                    FireControlSystem.calculateParameters(
                        drive.getPose2d(), drive.getChassisSpeedsWrtField())),
            Commands.waitUntil(shooter::isHoldingSetpoint)
                .andThen(indexer.index().alongWith(feeder.feed())))
        .until(routine.active().negate());
  }

  // * ~~~~~~~~ SUBSYSTEMS ~~~~~~~~

  private final DriveSubsystem mDrive;
  private final IntakeSubsystem mIntake;
  private final IndexerSubsystem mIndexer;
  private final FeederSubsystem mFeeder;
  private final ShooterSubsystem mShooter;

  // * ~~~~~~~~ CHOREO ~~~~~~~~

  private final AutoFactory mFactory;
  private final AutoChooser mChooser;

  public AutoBuilder(
      DriveSubsystem drive,
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      ShooterSubsystem shooter) {
    mDrive = drive;
    mIntake = intake;
    mIndexer = indexer;
    mFeeder = feeder;
    mShooter = shooter;

    mFactory =
        new AutoFactory(
            mDrive::getPose2d, mDrive::reset, mDrive.getChoreoSwerveSampleConsumer(), true, drive);
    mChooser = new AutoChooser();

    mChooser.addRoutine("Mid Empty", this::getMidEmptyRoutine);
    mChooser.addRoutine("Neutral Rush 2", () -> getNeutralRoutine(2));

    SmartDashboard.putData("Auto Chooser", mChooser);
  }

  public AutoRoutine getMidEmptyRoutine() {
    var routine = mFactory.newRoutine("Mid Empty");
    var trajectory = routine.trajectory("S2_F1");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                trajectory.resetOdometry(),
                trajectory.cmd().until(trajectory.done()),
                empty(routine, mDrive, mIndexer, mFeeder, mShooter)));

    return routine;
  }

  public AutoRoutine getNeutralRoutine(int length) {
    var routine = mFactory.newRoutine("Mid Empty");
    var trajectory = routine.trajectory("S1_N" + length);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                trajectory.resetOdometry(),
                trajectory.cmd().until(trajectory.done()),
                empty(routine, mDrive, mIndexer, mFeeder, mShooter)));

    trajectory.atTime("intake_start").whileTrue(mIntake.intake());
    trajectory.atTime("intake_end").whileTrue(mIntake.stow());

    return routine;
  }

  public Command getSelectedAuton() {
    return mChooser.selectedCommand();
  }
}
