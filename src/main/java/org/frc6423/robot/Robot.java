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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.Optional;
import org.frc6423.lib.driver.CommandRobot;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.drive.Drive;
import org.frc6423.robot.subsystem.shooter.Shooter;
import org.frc6423.robot.util.FuelSimulation;

@Logged
public class Robot extends CommandRobot {
  private final Optional<FuelSimulation> mFuelSim;

  private final RobotState mRobotState = RobotState.getInstance();

  private final Drive mDrive = Drive.create();
  private final Shooter mShooter = Shooter.create();

  private final CommandXboxController mController;

  public Robot() {
    // Initialize Devices
    mController = new CommandXboxController(0);

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

          // TODO remove
          DataLogManager.start();

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

    // Initialize Simulation
    if (Robot.isSimulation()) {
      mFuelSim = Optional.of(new FuelSimulation("Fuel Simulation"));

      mFuelSim.ifPresent(
          (sim) -> {
            // Initial Configuration
            sim.setSubticks(1);
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
                mDrive::getFieldRelativeChassisSpeeds);

            // Setup arena
            sim.spawnStartingFuel();

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
            addPeriodic(() -> sim.updateSim(), 0.02);
          });
    } else mFuelSim = Optional.empty();

    RobotModeTriggers.teleop()
        .whileTrue(
            mDrive.driveWhileFacingTarget(
                () -> modifyJoystick(mController.getLeftY()),
                () -> modifyJoystick(mController.getLeftX()),
                () -> Flags.getRobotAlliancePose2d(Rebuilt.kHubPose2d).getTranslation()));
  }

  public Optional<FuelSimulation> getFueldSimulation() {
    return mFuelSim;
  }

  private static double modifyJoystick(double value) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(value, 3)) * Math.signum(value), 0.02);
  }

  @Override
  protected Command getAutonCommand() {
    return Commands.none();
  }
}
