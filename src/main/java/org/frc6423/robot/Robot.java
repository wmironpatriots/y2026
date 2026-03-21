// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import org.frc6423.lib.driver.CommandRobot;
import org.frc6423.lib.util.InputStream;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.fcs.FireControlSystem;
import org.frc6423.robot.simulation.FuelSimulation;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;
import org.frc6423.robot.subsystem.feeder.FeederSubsystem;
import org.frc6423.robot.subsystem.indexer.IndexerSubsystem;
import org.frc6423.robot.subsystem.intake.IntakeSubsystem;
import org.frc6423.robot.subsystem.led.Led;
import org.frc6423.robot.subsystem.shooter.ShooterSubsystem;
import org.frc6423.robot.subsystem.vision.VisionSubsystem;

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

  // * ~~~~~~~~ SUBSYSTEMS ~~~~~~~~

  private final Led mLed = new Led();

  @Logged(name = "Drive Subsystem")
  private final DriveSubsystem mDrive = DriveSubsystem.create();

  @Logged(name = "Intake Subsystem")
  private final IntakeSubsystem mIntake = IntakeSubsystem.create();

  @Logged(name = "Indexer Subsystem")
  private final IndexerSubsystem mIndexer = IndexerSubsystem.create();

  @Logged(name = "Feeder Subsystem")
  private final FeederSubsystem mFeeder = FeederSubsystem.create();

  @Logged(name = "Shooter Subsystem")
  private final ShooterSubsystem mShooter = ShooterSubsystem.create();

  @Logged(name = "Vision Subsystem")
  private final VisionSubsystem mVision = VisionSubsystem.create();

  // * ~~~~~~~~ CONTROLLERS ~~~~~~~~

  private final AutoBuilder mAutoBuilder =
      new AutoBuilder(mDrive, mIntake, mIndexer, mFeeder, mShooter);

  private final CommandXboxController mDriverController = new CommandXboxController(0);

  // * ~~~~~~~~ DRIVER TRIGGERS ~~~~~~~~

  @Logged(name = "Operator Triggers/Request Intake")
  private final Trigger mTriggerIsIntaking = mDriverController.leftTrigger(0.1);

  @Logged(name = "Operator Triggers/Request Outake")
  private final Trigger mTriggerIsOutaking = mDriverController.leftBumper();

  @Logged(name = "Operator Triggers/Request Spinup")
  private final Trigger mTriggerIsSpinningUp = mDriverController.rightTrigger(0.1);

  @Logged(name = "Operator Triggers/Request Lock")
  private final Trigger mTriggerIsLocking = mDriverController.rightTrigger(0.5);

  @Logged(name = "Operator Triggers/Request Fire")
  private final Trigger mTriggerIsFiring = mDriverController.rightBumper();

  // * ~~~~~~~~ ROBOT TRIGGERS ~~~~~~~~

  private final Optional<FuelSimulation> mFuelSim =
      (isSimulation()) ? Optional.of(new FuelSimulation()) : Optional.empty();

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

    addPeriodic(
        () -> {
          var estimates = mVision.getLatestPoseEstimates();
          for (int i = 0; i < estimates.size(); i++) {
            var est = estimates.get(i);
            var stdDevs = mVision.getEstimationStdDevs().get(i);
            mDrive.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
          }
        },
        0.02);

    configureBindings();
    configureDashboard();
    configureSimulation();
    configureBehavior();
  }

  public void configureBehavior() {
    var estopTrigger = new Trigger(() -> DriverStation.isEStopped());
    var isActiveTrigger = new Trigger(() -> MatchInfo.getOfficialShiftInfo().active());
    var isLocked = new Trigger(() -> mDrive.isFacingAngularTarget());

    RobotModeTriggers.disabled().whileTrue(mLed.breath(Color.kDarkRed, 2).ignoringDisable(true));

    RobotModeTriggers.disabled()
        .and(estopTrigger)
        .whileTrue(mLed.solid(Color.kRed).ignoringDisable(true));

    RobotModeTriggers.autonomous().whileTrue(mLed.rainbow(MetersPerSecond.of(10)));

    RobotModeTriggers.teleop().whileTrue(mLed.solid(Color.kWhite));

    RobotModeTriggers.teleop().and(isActiveTrigger).whileTrue(mLed.solid(Color.kGreen));

    RobotModeTriggers.teleop()
        .and(mTriggerIsIntaking)
        .whileTrue(mLed.chase(MetersPerSecond.of(2.5), Color.kGreen, Color.kGold));

    RobotModeTriggers.teleop().and(mTriggerIsLocking).whileTrue(mLed.strobe(Color.kBlue, 0.05));

    RobotModeTriggers.teleop()
        .and(mTriggerIsLocking)
        .and(isLocked)
        .whileTrue(mLed.solid(Color.kBlue));

    RobotModeTriggers.teleop()
        .and(mTriggerIsFiring)
        .whileTrue(mLed.strobe(Color.kBlue, Color.kGreen, 0.5));
  }

  /** Configure driver bindings */
  public void configureBindings() {

    // ~~~ Intake Controls ~~~

    mTriggerIsIntaking.whileTrue(mIntake.intake());

    mTriggerIsOutaking.whileTrue(mIntake.outake());

    // ~~~ Indexer/Feeder Controls ~~~

    mTriggerIsOutaking.and(mTriggerIsIntaking.negate()).whileTrue(mIndexer.feedInverse());

    mTriggerIsFiring.whileTrue(mIndexer.index()).whileTrue(mFeeder.feed());

    // ~~~ Shooter Controls ~~~

    mTriggerIsSpinningUp.whileTrue(
        mShooter.runSetpoint(
            () ->
                FireControlSystem.calculateParameters(
                    mDrive.getPose2d(), mDrive.getChassisSpeedsWrtField())));

    mTriggerIsFiring.whileTrue(mIndexer.index()).whileTrue(mFeeder.feed());

    // ~~~ Drive Controls ~~~

    InputStream rawX =
        InputStream.of(mDriverController::getLeftY)
            .negate()
            .log("Telemetry/Operator Triggers/Raw Vx Vec");
    InputStream rawY =
        InputStream.of(mDriverController::getLeftX)
            .negate()
            .log("Telemetry/Operator Triggers/Raw Vy Vec");

    InputStream r =
        InputStream.hypot(rawX, rawY)
            .clamp(1.0)
            .deadband(0.02, 1.0)
            .signedPow(2.0)
            .scale(() -> mTriggerIsLocking.getAsBoolean() ? 0.2 : 1.0)
            .scale(() -> Flags.kDrivetrainContants.getMaxLinearVelocityMetersPerSecond())
            .log("Telemetry/Operator Triggers/V Vec");

    InputStream theta = InputStream.atan(rawX, rawY);

    InputStream x = r.scale(theta.map(Math::cos)).log("Telemetry/Operator Triggers/Vx Vec");
    InputStream y = r.scale(theta.map(Math::sin)).log("Telemetry/Operator Triggers/Vy Vec");

    InputStream omega =
        InputStream.of(mDriverController::getRightX)
            .negate()
            .clamp(1.0)
            .deadband(0.02, 1.0)
            .signedPow(2.0)
            .scale(0.85)
            .scale(() -> Flags.kDrivetrainContants.getMaxAngularVelocityRadsPerSec())
            .log("Telemetry/Operator Triggers/Omega");

    RobotModeTriggers.teleop()
        .and(mTriggerIsLocking.negate())
        .whileTrue(mDrive.driveTeleoperated(x, y, omega));

    RobotModeTriggers.teleop()
        .and(mTriggerIsLocking)
        .whileTrue(
            mDrive.driveTeleoperatedFacingTarget(
                x, y, () -> FireControlSystem.getVirtualTarget(), true));

    mDriverController.x().onTrue(Commands.runOnce(() -> mDrive.reset(new Pose2d()), mDrive));
  }

  /** Configure driver dashboard */
  public void configureDashboard() {
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> MatchInfo.initialize()));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> MatchInfo.initialize()));

    addPeriodic(
        () -> {
          SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
          SmartDashboard.putString(
              "Shifts/Current Shift", MatchInfo.getOfficialShiftInfo().currentShift().toString());
          SmartDashboard.putBoolean("Shifts/Is Active", MatchInfo.getOfficialShiftInfo().active());
          SmartDashboard.putString(
              "Shifts/Remaining Shift Time",
              String.format(
                  "%.1f", Math.max(MatchInfo.getOfficialShiftInfo().remainingTime(), 0.0), 0.0));
        },
        0.01);
  }

  /** Configure simulation */
  public void configureSimulation() {
    mFuelSim.ifPresent(
        (sim) -> {
          // Initial Configuration
          sim.enableAirResistance();

          // Setup robot
          var chassisWidth =
              Meters.of(
                  Flags.kDrivetrainContants.getTrackWidthMeters()
                      + Flags.kDrivetrainContants.getBumperThicknessInches());
          sim.registerRobot(
              chassisWidth,
              chassisWidth,
              Inches.of(6),
              mDrive::getPose2d,
              mDrive::getChassisSpeedsWrtField);

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

          // Configure sim actions
          mTriggerIsFiring.whileTrue(
              Commands.runOnce(
                      () ->
                          sim.launchFuel(
                              MetersPerSecond.of(mShooter.getApproximatedMuzzleVelocityMps()),
                              mShooter.getRotation2d().getMeasure(),
                              Rotation2d.k180deg.getMeasure(),
                              ShooterSubsystem.kRobotToShooter.getMeasureZ()))
                  .andThen(Commands.waitSeconds(0.5))
                  .repeatedly());

          mDriverController.povDown().onTrue(Commands.runOnce(() -> sim.clearFuel()));

          //   // Start sim notifier
          //   addPeriodic(() -> sim.updateSim(), 0.02);
        });
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    mFuelSim.ifPresent((sim) -> sim.updateSim());
  }

  @Override
  protected Command getAutonCommand() {
    return mAutoBuilder.getSelectedAuton();
  }
}
