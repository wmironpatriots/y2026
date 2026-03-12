// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import org.frc6423.lib.driver.CommandRobot;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.command.Auto;
import org.frc6423.robot.command.DriveTeleoperatedCommands;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;
import org.frc6423.robot.subsystem.feeder.Feeder;
import org.frc6423.robot.subsystem.indexer.Indexer;
import org.frc6423.robot.subsystem.intake.Intake;
import org.frc6423.robot.subsystem.shooter.ShooterSubsystem;
import org.frc6423.robot.subsystem.vision.Vision;
import org.frc6423.robot.util.HubShiftUtil;
import org.frc6423.robot.util.sim.FuelSimulation;
import org.frc6423.robot.util.sim.HopperSimulation;

@Logged
public class Robot extends CommandRobot {
  private final RobotState mRobotState = RobotState.getInstance();

  // * ~~~~~~~~ SUBSYSTEMS ~~~~~~~~

  private final DriveSubsystem mDrive = DriveSubsystem.create();
  private final Vision mVision = Vision.create();
  private final Intake mIntake = Intake.create();
  private final Indexer mIndexer = Indexer.create();
  private final Feeder mFeeder = Feeder.create();
  private final ShooterSubsystem mShooter = ShooterSubsystem.create();

  // * ~~~~~~~~ SIM SYSTEMS ~~~~~~~~

  private Optional<FuelSimulation> mFuelSim = Optional.empty();
  private Optional<HopperSimulation> mHopperSim = Optional.empty();

  // * ~~~~~~~~ CONTROL ~~~~~~~~

  private final Auto mAuto = new Auto(mDrive);

  private final CommandXboxController mController = new CommandXboxController(0);
  private final Trigger mIntakeRequest = mController.leftBumper();
  private final Trigger mFireRequest =
      mController.rightBumper().and(new Trigger(mShooter::isHoldingSetpoint));

  public Robot() {
    // Shut up DS
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure Epilogue
    Epilogue.configure(
        (EpilogueConfiguration config) -> {
          // Set log data path
          config.root = "Telemetry";

          // Lazylog to NT
          config.backend =
              new LazyBackend(new NTEpilogueBackend(NetworkTableInstance.getDefault()));

          // Start logging
          if (Robot.isReal()) {
            DataLogManager.start();
          }

          // Set error handling
          if (Robot.isSimulation()) {
            config.errorHandler = ErrorHandler.crashOnError();
          } else {
            config.errorHandler = ErrorHandler.printErrorMessages();
          }

          // Set lowest importance level to be logged
          config.minimumImportance = Flags.kLoggingLevel;
        });

    // Bind Epilogue to robot periodic
    Epilogue.bind(this);

    // Log metadata
    final String metadataPath = "/BuildData/";
    var config = Epilogue.getConfig();
    config.backend.log(metadataPath + "RuntimeType", getRuntimeType().toString());
    config.backend.log(metadataPath + "ProjectName", BuildConstants.MAVEN_NAME);
    config.backend.log(metadataPath + "Version", BuildConstants.VERSION);
    config.backend.log(metadataPath + "GitDirty", BuildConstants.DIRTY);
    config.backend.log(metadataPath + "GitRev", BuildConstants.GIT_REVISION);
    config.backend.log(metadataPath + "GitSHA", BuildConstants.GIT_SHA);
    config.backend.log(metadataPath + "GitDate", BuildConstants.GIT_DATE);
    config.backend.log(metadataPath + "GitBranch", BuildConstants.GIT_BRANCH);
    config.backend.log(metadataPath + "BuildDate", BuildConstants.BUILD_DATE);
    config.backend.log(metadataPath + "BuildUnixTime", BuildConstants.BUILD_UNIX_TIME);

    setupSimulation();
    configureDashboardNotifiers();
    configureBindings();
  }

  public void configureDashboardNotifiers() {
    addPeriodic(
        () -> {
          SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
          SmartDashboard.putString(
              "Shifts/Current Shift",
              HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
          SmartDashboard.putBoolean(
              "Shifts/Is Active", HubShiftUtil.getOfficialShiftInfo().active());
          SmartDashboard.putString(
              "Shifts/Remaining Shift Time",
              String.format(
                  "%.1f", Math.max(HubShiftUtil.getOfficialShiftInfo().remainingTime(), 0.0), 0.0));
        },
        0.04);
  }

  /** Configure driver bindings */
  public void configureBindings() {
    // Norm mode
    RobotModeTriggers.teleop()
        .and(mIntakeRequest.negate())
        .whileTrue(
            DriveTeleoperatedCommands.runTeleoperatedDriveWhileFacing(
                mDrive,
                mController::getLeftY,
                mController::getLeftX,
                () ->
                    Flags.getRobotAlliancePose2d(Rebuilt.kHubPose2d)
                        .getTranslation(), // TODO replace placeholder for fcs target
                true));

    // Intake
    RobotModeTriggers.teleop()
        .and(mIntakeRequest)
        .whileTrue(
            DriveTeleoperatedCommands.runTeleoperatedDrive(
                mDrive, mController::getLeftY, mController::getLeftX, mController::getRightX))
        .whileTrue(mIntake.intake());

    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
  }

  /** Setup simulation optionals if robot is simulated */
  public void setupSimulation() {
    // Initialize Simulation
    if (Robot.isSimulation()) {
      mFuelSim = Optional.of(new FuelSimulation("Fuel Simulation"));
      mHopperSim = Optional.of(new HopperSimulation());

      mFuelSim.ifPresent(
          (sim) -> {
            // Initial Configuration
            sim.enableAirResistance();

            // Setup robot
            var chassisWidth =
                Meters.of(
                    Flags.kDriveConstants.getTrackWidthMeters()
                        + Flags.kDriveConstants.getBumperThicknessInches());
            sim.registerRobot(
                chassisWidth,
                chassisWidth,
                Inches.of(6),
                mDrive::getPose2d,
                mDrive::getChassisSpeedsWrtField);

            // Setup arena
            if (Flags.kSpawnStartingFuel) {
              sim.spawnStartingFuel();
            }

            // Reset field when auton opp mode starts
            RobotModeTriggers.autonomous()
                .onTrue(
                    Commands.runOnce(
                        () -> {
                          sim.clearFuel();
                          sim.spawnStartingFuel();
                        }));

            // Start sim
            sim.start();

            // Start sim notifier
            addPeriodic(() -> sim.updateSim(), 0.002);
          });
    } else {
      mFuelSim = Optional.empty();
      mHopperSim = Optional.empty();
    }
  }

  @Override
  protected Command getAutonCommand() {
    return mAuto.getSelectedAuto();
  }
}
