// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc6423.lib.driver.CommandRobot;
import org.frc6423.lib.util.InputStream;
import org.frc6423.robot.Constants.Field;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;
import org.frc6423.robot.subsystem.feeder.FeederSubsystem;
import org.frc6423.robot.subsystem.indexer.IndexerSubsystem;
import org.frc6423.robot.subsystem.intake.IntakeSubsystem;
import org.frc6423.robot.subsystem.shooter.ShooterSubsystem;

/**
 * Robot initializes all components and defines the behavior of the program
 *
 * <p>This is an extension of {@link CommandRobot}
 *
 * <p>Virtual systems should be initialized within {@link #Robot()}
 *
 * <p>Driver/operator controls should be initialized within {@link #configureBindings()}
 *
 * <p>Dashboard widgets should be initilaized within {@link #configureDashboard()}
 *
 * <p>Simulation members should be initialized within {@link #configureSimulation()}
 */
@Logged
public class Robot extends CommandRobot {
  private final CommandXboxController mDriverController = new CommandXboxController(0);

  private final DriveSubsystem mDrive = DriveSubsystem.create();
  private final IntakeSubsystem mIntake = IntakeSubsystem.create();
  private final IndexerSubsystem mIndexer = IndexerSubsystem.create();
  private final FeederSubsystem mFeeder = FeederSubsystem.create();
  private final ShooterSubsystem mShooter = ShooterSubsystem.create();

  private final Trigger mIntakeTrigger = mDriverController.leftTrigger(0.1);
  private final Trigger mAgitateTrigger = mDriverController.leftBumper();
  private final Trigger mSpinupTrigger = mDriverController.rightTrigger(0.1);
  private final Trigger mFireTrigger =
      mDriverController.rightBumper().and(mShooter::isHoldingSetpoint);

  private final Trigger mInAllianceZone =
      new Trigger(() -> Field.getAllianceZone().contains(mDrive.getPose2d().getTranslation()));

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
          config.minimumImportance = Flags.kLowestLoggingLevel;
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

    configureBindings();
    configureDashboard();
    configureSimulation();
  }

  /** Configure driver bindings */
  public void configureBindings() {

    // ~~~ Intake Controls ~~~

    mIntakeTrigger.and(mAgitateTrigger.negate()).whileTrue(mIntake.intake());

    mIntakeTrigger.and(mAgitateTrigger).whileTrue(mIntake.intakeAgitated());

    // ~~~ Indexer/Feeder Controls ~~~

    mAgitateTrigger.and(mIntakeTrigger.negate()).whileTrue(mIndexer.index());

    mFireTrigger.whileTrue(mIndexer.index()).whileTrue(mFeeder.feed());

    // ~~~ Shooter Controls ~~~

    // ~~~ Drive Controls ~~~

    InputStream rawX = InputStream.of(mDriverController::getLeftY).negate();
    InputStream rawY = InputStream.of(mDriverController::getLeftX).negate();

    InputStream r =
        InputStream.hypot(rawX, rawY)
            .clamp(1.0)
            .deadband(0.02, 1.0)
            .signedPow(2.0)
            .scale(() -> Flags.kDrivetrainContants.getMaxLinearVelocityMetersPerSecond());
    // .rateLimit(
    //     Flags.kDrivetrainContants
    //         .getMaxLinearAccelerationMetersPerSecondPerSecond()); // tODO remove?

    InputStream theta = InputStream.atan(rawX, rawY);

    InputStream x = r.scale(theta.map(Math::cos));
    InputStream y = r.scale(theta.map(Math::sin));

    InputStream omega =
        InputStream.of(mDriverController::getRightX)
            .negate()
            .clamp(1.0)
            .deadband(0.02, 1.0)
            .signedPow(2.0)
            .scale(() -> Flags.kDrivetrainContants.getMaxAngularVelocityRadsPerSec())
            .rateLimit(Flags.kDrivetrainContants.getMaxAngularVelocityRadsPerSec());

    RobotModeTriggers.teleop().whileTrue(mDrive.driveTeleoperated(x, y, omega));
  }

  /** Configure driver dashboard */
  public void configureDashboard() {}

  /** Configure simulation */
  public void configureSimulation() {}

  @Override
  protected Command getAutonCommand() {
    return Commands.none();
  }
}
