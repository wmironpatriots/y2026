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
import org.frc6423.robot.Constants.Flags;
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
  private final Intake mIntake;
  private final Indexer mIndexer;

  public Auto(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      Feeder feeder,
      Indexer indexer,
      Intake intake) {
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
    // traj.atTime("score")
    //     .whileTrue(
    //         mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));
    traj.atTime("score")
        .whileTrue(
            mShooter.runSetpoint(
                () -> ShooterSubsystem.kHubShotMap,
                () -> Flags.getRobotAlliancePose2d(Rebuilt.kHubPose2d)));
    traj.atTime("score")
        .and(mShooter::isHoldingSetpoint)
        .whileTrue(Commands.parallel(mIndexer.index(), mFeeder.feed()));
  }
}
  // // Starting Depot Edges
  // public Command driveS4toA1() {
  //   return mDrive
  //       .runOnce(() -> kRobotState.resetPose(Node.S4.getPose2d()))
  //       .andThen(mFactory.trajectoryCmd("S4_A1"));
  // }

  // public Command driveS2toA1() {
  //   return mDrive
  //       .runOnce(() -> kRobotState.resetPose(Node.S2.getPose2d()))
  //       .andThen(mFactory.trajectoryCmd("S2_A1"));
  // }

  // public Command driveS3toA1() {
  //   return mDrive
  //       .runOnce(() -> kRobotState.resetPose(Node.S3.getPose2d()))
  //       .andThen(mFactory.trajectoryCmd("S3_A1"));
  // }

  // // Starting Firing edge
  // public Command driveA1toF(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("A1_F");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveS1toF(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("S1_F");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // // Starting S1 to Neutral Zone edges
  // public Command driveS1toN2() {
  //   return mDrive
  //       .runOnce(() -> kRobotState.resetPose(Node.S1.getPose2d()))
  //       .andThen(mFactory.trajectoryCmd("S1_N2"));
  // }

  // public Command driveS1toN3() {
  //   return mDrive
  //       .runOnce(() -> kRobotState.resetPose(Node.S1.getPose2d()))
  //       .andThen(mFactory.trajectoryCmd("S1_N3"));
  // }

  // public Command driveS1toN4() {
  //   return mDrive
  //       .runOnce(() -> kRobotState.resetPose(Node.S1.getPose2d()))
  //       .andThen(mFactory.trajectoryCmd("S1_N4"));
  // }

  // public Command driveS1toN5() {
  //   return mDrive
  //       .runOnce(() -> kRobotState.resetPose(Node.S1.getPose2d()))
  //       .andThen(mFactory.trajectoryCmd("S1_N5"));
  // }

  // // Starting Neutral Zone reverse edges
  // public Command driveN2toS1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N2_S1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveN3toS1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N3_S1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveN4toS1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N4_S1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveN5toS1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N5_S1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveN2toF1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N2_F1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveN3toF1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N3_F1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveN4toF1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N4_F1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // public Command driveN5toF1(AutoRoutine routine) {
  //   final AutoTrajectory trajectory = routine.trajectory("N5_F1");

  //   return Commands.sequence(trajectory.resetOdometry(),
  // trajectory.cmd().until(trajectory.done()));
  // }

  // NEUTRAL ZONE ROUTINES
//   public AutoRoutine S1_N2_cycle() {
//     String[] stops = {"S1", "N2", "S1", "F"};
//     AutoRoutine routine = buildRoutine("S1_N2_cycle", stops);

//     // Event Markers from Choreo [Remember to change the Commands.none()]
//     routine.get("S1_N2").atTime("intake_start").onTrue(Commands.none());
//     routine.get("S1_N2").atTime("intake_end").onTrue(Commands.none());
//     steps.get("S1_N2").atTime("intake_end").onTrue(Commands.none());

//     return routine;
//   }

//   public AutoRoutine S1_N3_cycle() {

//     final var routine = mFactory.newRoutine("S1-N3-Cycle");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();

//     // define the route (Highlander style hehe)
//     String[] stops = {"S1", "N3", "S1", "F"};

//     // load all trajectories automatically
//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     // Event Markers from Choreo [Remember to change the Commands.none()]

//     steps.get("S1_N3").atTime("intake_start").onTrue(Commands.none());

//     steps.get("S1_N3").atTime("intake_end").onTrue(Commands.none());

//     steps
//         .get("S1_F")
//         .atTime("score")
//         .onTrue(mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () ->
// Rebuilt.kHubPose2d));

//     // Running the actual start path
//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S1_N3").resetOdometry(), steps.get("S1_N3").cmd()));
//     // Running return path
//     routine.observe(steps.get("S1_N3").done()).onTrue(steps.get("N3_S1").cmd());

//     // Running shooter path
//     routine.observe(steps.get("N3_S1").done()).onTrue(steps.get("S1_F").cmd());

//     // return command
//     return routine;
//   }

//   public AutoRoutine S1_N4_cycle() {

//     final var routine = mFactory.newRoutine("S1-N4-Cycle");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();

//     // define the route (Highlander style hehe)
//     String[] stops = {"S1", "N4", "S1", "F"};

//     // load all trajectories automatically
//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     // Event Markers from Choreo [Remember to change the Commands.none()]

//     steps.get("S1_N4").atTime("intake_start").onTrue(Commands.none());

//     steps.get("S1_N4").atTime("intake_end").onTrue(Commands.none());

//     steps
//         .get("S1_F")
//         .atTime("score")
//         .onTrue(mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () ->
// Rebuilt.kHubPose2d));

//     // Running the actual start path
//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S1_N4").resetOdometry(), steps.get("S1_N4").cmd()));

//     // Running return path
//     routine.observe(steps.get("S1_N4").done()).onTrue(steps.get("N4_S1").cmd());

//     // Running shooter path
//     routine.observe(steps.get("N4_S1").done()).onTrue(steps.get("S1_F").cmd());

//     // return command
//     return routine;
//   }

//   public AutoRoutine S1_N5_cycle() {

//     final var routine = mFactory.newRoutine("S1-N5-Cycle");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();

//     // define the route (Highlander style hehe)
//     String[] stops = {"S1", "N5", "S1", "F"};

//     // load all trajectories automatically
//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     // Event Markers from Choreo [Remember to change the Commands.none()]

//     steps.get("S1_N5").atTime("intake_start").onTrue(Commands.none());

//     steps.get("S1_N5").atTime("intake_end").onTrue(Commands.none());

//     steps
//         .get("S1_F")
//         .atTime("score")
//         .onTrue(mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () ->
// Rebuilt.kHubPose2d));

//     // Running the actual start path
//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S1_N5").resetOdometry(), steps.get("S1_N5").cmd()));

//     // Running return path
//     routine.observe(steps.get("S1_N5").done()).onTrue(steps.get("N5_S1").cmd());

//     // Running shooter path
//     routine.observe(steps.get("N5_S1").done()).onTrue(steps.get("S1_F").cmd());

//     // return command
//     return routine;
//   }

//   // DEPOT SHOT ROUTINES

//   public AutoRoutine Depot_Shot_cycle_S4() {

//     final var routine = mFactory.newRoutine("Depot_Shot_cycle_S4");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();

//     // define the route (Highlander style hehe)
//     String[] stops = {"S4", "A1", "F"};

//     // load all trajectories automatically
//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     // Event Markers from Choreo [Remember to change the Commands.none()]

//     steps.get("S4_A1").atTime("intake_start").onTrue(Commands.none());

//     steps.get("S4_A1").atTime("intake_end").onTrue(Commands.none());

//     steps
//         .get("A1_F")
//         .atTime("score")
//         .whileTrue(
//             mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));

//     // Running the actual start path
//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S4_A1").resetOdometry(), steps.get("S4_A1").cmd()));
//     // Running return path
//     routine.observe(steps.get("S4_A1").done()).onTrue(steps.get("A1_F").cmd());

//     // return command
//     return routine;
//   }

//   public AutoRoutine Depot_Shot_cycle_S2() {

//     final var routine = mFactory.newRoutine("Depot_Shot_cycle_S2");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();

//     // define the route (Highlander style hehe)
//     String[] stops = {"S2", "A1", "F"};

//     // load all trajectories automatically
//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     // Event Markers from Choreo [Remember to change the Commands.none()]

//     steps.get("S2_A1").atTime("intake_start").onTrue(Commands.none());

//     steps.get("S2_A1").atTime("intake_end").onTrue(Commands.none());

//     steps
//         .get("A1_F")
//         .atTime("score")
//         .whileTrue(
//             mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));

//     // Running the actual start path
//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S2_A1").resetOdometry(), steps.get("S2_A1").cmd()));

//     // Running return path
//     routine.observe(steps.get("S2_A1").done()).onTrue(steps.get("A1_F").cmd());

//     // return command
//     return routine;
//   }

//   public AutoRoutine Depot_Shot_cycle_S3() {

//     final var routine = mFactory.newRoutine("Depot_Shot_cycle_S3");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();

//     // define the route (Highlander style hehe)
//     String[] stops = {"S3", "A1", "F"};

//     // load all trajectories automatically
//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     // Event Markers from Choreo [Remember to change the Commands.none()]

//     steps.get("S3_A1").atTime("intake_start").onTrue(Commands.none());

//     steps.get("S3_A1").atTime("intake_end").onTrue(Commands.none());

//     steps
//         .get("A1_F")
//         .atTime("score")
//         .whileTrue(
//             mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));

//     // Running the actual start path
//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S3_A1").resetOdometry(), steps.get("S3_A1").cmd()));
//     // Running return path
//     routine.observe(steps.get("S3_A1").done()).onTrue(steps.get("A1_F").cmd());

//     // return command
//     return routine;
//   }

//   public AutoRoutine S1_N2_F1_cycle() {
//     final var routine = mFactory.newRoutine("S1-N2-F1-Cycle");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();

//     String[] stops = {"S1", "N2", "F1"};

//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     steps.get("S1_N2").atTime("intake_start").onTrue(Commands.none());
//     steps.get("S1_N2").atTime("intake_end").onTrue(Commands.none());
//     steps
//         .get("N2_F1")
//         .atTime("score")
//         .whileTrue(
//             mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));
//     steps
//         .get("N2_F1")
//         .atTime("score")
//         .and(mShooter::isHoldingSetpoint)
//         .whileTrue(Commands.parallel(mIndexer.index(), mFeeder.feed()));

//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S1_N2").resetOdometry(), steps.get("S1_N2").cmd()));
//     routine.observe(steps.get("S1_N2").done()).onTrue(steps.get("N2_F1").cmd());

//     return routine;
//   }

//   public AutoRoutine S1_N3_F1_cycle() {
//     final var routine = mFactory.newRoutine("S1-N3-F1-Cycle");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();
//     String[] stops = {"S1", "N3", "F1"};

//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     steps.get("S1_N3").atTime("intake_start").onTrue(Commands.none());
//     steps.get("S1_N3").atTime("intake_end").onTrue(Commands.none());
//     steps
//         .get("N3_F1")
//         .atTime("score")
//         .whileTrue(
//             mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));

//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S1_N3").resetOdometry(), steps.get("S1_N3").cmd()));
//     routine.observe(steps.get("S1_N3").done()).onTrue(steps.get("N3_F1").cmd());

//     return routine;
//   }

//   public AutoRoutine S1_N4_F1_cycle() {
//     final var routine = mFactory.newRoutine("S1-N4-F1-Cycle");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();
//     String[] stops = {"S1", "N4", "F1"};

//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     steps.get("S1_N4").atTime("intake_start").onTrue(Commands.none());
//     steps.get("S1_N4").atTime("intake_end").onTrue(Commands.none());
//     steps
//         .get("N4_F1")
//         .atTime("score")
//         .whileTrue(
//             mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));

//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S1_N4").resetOdometry(), steps.get("S1_N4").cmd()));
//     routine.observe(steps.get("S1_N4").done()).onTrue(steps.get("N4_F1").cmd());

//     return routine;
//   }

//   public AutoRoutine S1_N5_F1_cycle() {
//     final var routine = mFactory.newRoutine("S1-N5-F1-Cycle");

//     HashMap<String, AutoTrajectory> steps = new HashMap<>();
//     String[] stops = {"S1", "N5", "F1"};

//     for (int i = 0; i < stops.length - 1; i++) {
//       String name = stops[i] + "_" + stops[i + 1];
//       steps.put(name, routine.trajectory(name));
//     }

//     steps.get("S1_N5").atTime("intake_start").onTrue(Commands.none());
//     steps.get("S1_N5").atTime("intake_end").onTrue(Commands.none());
//     steps
//         .get("N5_F1")
//         .atTime("score")
//         .whileTrue(
//             mShooter.runSetpoint(() -> ShooterSubsystem.kHubShotMap, () -> Rebuilt.kHubPose2d));

//     routine
//         .active()
//         .onTrue(Commands.sequence(steps.get("S1_N5").resetOdometry(), steps.get("S1_N5").cmd()));
//     routine.observe(steps.get("S1_N5").done()).onTrue(steps.get("N5_F1").cmd());

//     return routine;
//   }
// }
