// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.command;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;

import org.frc6423.robot.Rebuilt;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;
import org.frc6423.robot.subsystem.feeder.Feeder;
import org.frc6423.robot.subsystem.indexer.Indexer;
import org.frc6423.robot.subsystem.intake.Intake;
import org.frc6423.robot.subsystem.shooter.ShooterSubsystem;

public class Auto {
  private static final RobotState kRobotState = RobotState.getInstance();

  public static enum Node {
    S1(new Pose2d(4.50, 0.65, Rotation2d.fromRadians(1 / 4 * Math.PI))),
    S2(new Pose2d(3.55, 4.00, Rotation2d.fromRadians(Math.PI))),
    S3(new Pose2d(3.6513702869415283, 7.2504496574401855, Rotation2d.fromRadians(Math.PI))),
    S4(new Pose2d(3.55, 0.70, Rotation2d.fromRadians(Math.PI))),
    N2(new Pose2d(8.00, 2.9, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    N3(new Pose2d(7.90, 4.00, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    N4(new Pose2d(7.90, 5.20, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    N5(new Pose2d(7.90, 6.00, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    A1(new Pose2d(0.65, 5.95, Rotation2d.fromRadians(1 / 2 * Math.PI))),
    F(new Pose2d(2.33, 4.00, Rotation2d.fromRadians(Math.PI)));
    private final Pose2d mPose2d;

    private Node(Pose2d pose) {
      mPose2d = pose;
    }

    public Pose2d getPose2d() {
      return mPose2d;
    }
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private final DriveSubsystem mDrive;
  private final AutoFactory mFactory;
  private final AutoChooser mAutoChooser;
  private final ShooterSubsystem mShooter;
  private final Feeder mFeeder;
  private final Indexer mIndexer;
  private final Intake mIntake;

  public Auto(DriveSubsystem drive, ShooterSubsystem shooter, Feeder feeder, Indexer indexer, Intake intake) {
    this.mDrive = drive;
    this.mShooter = shooter;
    this.mFeeder = feeder;
    this.mIndexer = indexer;
    this.mIntake = intake;

    // Init Auto Factory
    mFactory =
        new AutoFactory(
            kRobotState::getEstimatedPosition,
            kRobotState::resetPose,
            mDrive.getChoreoSwerveSampleConsumer(),
            true,
            drive);
    // Init Auto Chooser
    mAutoChooser = new AutoChooser();
    addAuto("S1 N2 cycle", "S1", "N2", "S1", "F");
    addAuto("S1 N3 cycle", "S1", "N3", "S1", "F");
    addAuto("S1 N4 cycle", "S1", "N4", "S1", "F");
    addAuto("S1 N5 cycle", "S1", "N5", "S1", "F");

    addAuto("S1 N2 F1 cycle", "S1", "N2", "F1");
    addAuto("S1 N3 F1 cycle", "S1", "N3", "F1");
    addAuto("S1 N4 F1 cycle", "S1", "N4", "F1");
    addAuto("S1 N5 F1 cycle", "S1", "N5", "F1");

    addAuto("Depot Shot S4", "S4", "A1", "F");
    addAuto("Depot Shot S3", "S3", "A1", "F");
    addAuto("Depot Shot S2", "S2", "A1", "F");

    SmartDashboard.putData("Auto Chooser", mAutoChooser);
  }

  private void addAuto(String name, String... stops) {
    mAutoChooser.addRoutine(name, () -> buildRoutine(name, stops));
  }

  // Give current selected auto
  public Command getSelectedAuto() {
    return mAutoChooser.selectedCommand();
  }

  private AutoRoutine buildRoutine(String routineName, String... stops) {

    final var routine = mFactory.newRoutine(routineName);
    HashMap<String, AutoTrajectory> steps = new HashMap<>();

    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "_" + stops[i + 1];
      steps.put(name, routine.trajectory(name));
      registerCommand(steps.get(name));
    }

    String first = stops[0] + "_" + stops[1];

    routine
        .active()
        .onTrue(Commands.sequence(steps.get(first).resetOdometry(), steps.get(first).cmd()));
    for (int i = 0; i < stops.length - 2; i++) {

      String current = stops[i] + "_" + stops[i + 1];
      String next = stops[i + 1] + "_" + stops[i + 2];

      routine.observe(steps.get(current).done()).onTrue(steps.get(next).cmd());
    }

    return routine;
  }

  private void registerCommand(AutoTrajectory traj) {
    traj.atTime("intake_start").onTrue(mIntake.intake());
    traj.atTime("intake_end").onTrue(mIntake.stow());
    traj.atTime("score").onTrue(mShooter.runSetpoint(()-> ShooterSubsystem.kHubShotMap, ()-> Rebuilt.kHubPose2d));
    traj.atTime("score")
        .and(mShooter::isHoldingSetpoint)
        .whileTrue(Commands.parallel(mIndexer.index(), mFeeder.feed()));
    
  }
}
