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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import org.frc6423.lib.driver.CommandRobot;
import org.frc6423.lib.util.InputStream;
import org.frc6423.robot.Constants.Field;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.fcs.FireControlSystem;
import org.frc6423.robot.simulation.FuelSimulation;
import org.frc6423.robot.subsystem.AutonBuilder;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;
import org.frc6423.robot.subsystem.feeder.FeederSubsystem;
import org.frc6423.robot.subsystem.indexer.IndexerSubsystem;
import org.frc6423.robot.subsystem.intake.IntakeSubsystem;
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
  private final CommandXboxController mDriverController = new CommandXboxController(0);

  private final DriveSubsystem mDrive = DriveSubsystem.create();
  private final IntakeSubsystem mIntake = IntakeSubsystem.create();
  private final IndexerSubsystem mIndexer = IndexerSubsystem.create();
  private final FeederSubsystem mFeeder = FeederSubsystem.create();
  private final ShooterSubsystem mShooter = ShooterSubsystem.create();
  private final VisionSubsystem mVision = VisionSubsystem.create();

  private final AutonBuilder mAuto = new AutonBuilder(mDrive);

  private final Trigger mIntakeTrigger = mDriverController.leftTrigger(0.1);
  private final Trigger mOutakeTrigger = mDriverController.leftBumper();
  private final Trigger mSpinupTrigger = mDriverController.rightTrigger(0.1);
  private final Trigger mLockTrigger = mDriverController.rightTrigger(0.5);
  private final Trigger mFireTrigger = mDriverController.rightBumper();
  // .and(mShooter::isHoldingSetpoint)
  // .and(mDrive::isFacingAngularTarget);

  private final Trigger mInAllianceZone =
      new Trigger(() -> Field.getAllianceZone().contains(mDrive.getPose2d().getTranslation()));

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
  }

  @Logged
  public Pose2d getTarget() {
    return new Pose2d(FireControlSystem.getVirtualTarget(), Rotation2d.kZero);
  }

  /** Configure driver bindings */
  public void configureBindings() {

    // ~~~ Intake Controls ~~~

    mIntakeTrigger.whileTrue(mIntake.intake());

    mOutakeTrigger.whileTrue(mIntake.outake());

    mOutakeTrigger.whileTrue(mIntake.intakeAgitated());

    // ~~~ Indexer/Feeder Controls ~~~

    mOutakeTrigger.and(mIntakeTrigger.negate()).whileTrue(mIndexer.feedInverse());

    mFireTrigger.whileTrue(mIndexer.index()).whileTrue(mFeeder.feed());

    // ~~~ Shooter Controls ~~~

    mSpinupTrigger.whileTrue(
        mShooter.runSetpoint(
            () ->
                FireControlSystem.calculateParameters(
                    mDrive.getPose2d(), mDrive.getChassisSpeedsWrtField())));

    mFireTrigger.whileTrue(mIndexer.index()).whileTrue(mFeeder.feed());

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
            .scale(() -> Flags.kDrivetrainContants.getMaxAngularVelocityRadsPerSec());

    RobotModeTriggers.teleop()
        .and(mLockTrigger.negate())
        .whileTrue(mDrive.driveTeleoperated(x, y, omega));

    RobotModeTriggers.teleop()
        .and(mLockTrigger)
        .whileTrue(
            mDrive.driveTeleoperatedFacingTarget(
                x, y, () -> FireControlSystem.getVirtualTarget(), true));
  }

  /** Configure driver dashboard */
  public void configureDashboard() {
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> MatchInfo.initialize()));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> MatchInfo.initialize()));
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> MatchInfo.stop()));

    addPeriodic(() -> MatchInfo.log(), 0.02);
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
                        // sim.spawnStartingFuel();
                      }));

          // Start sim
          sim.start();

          mFireTrigger.whileTrue(
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
    return Commands.none();
  }
}
