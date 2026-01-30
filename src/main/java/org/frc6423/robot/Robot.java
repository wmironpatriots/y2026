// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc6423.lib.driver.CommandRobot;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;
import org.frc6423.lib.io.ServoIOTalonFxSim;
import org.frc6423.lib.sim.PivotMechSim;

@Logged
public class Robot extends CommandRobot {
  private final CommandXboxController mDriverController, mOperatorController, mDevController;

  private final ServoIO io =
      new ServoIOTalonFxSim(
          "bruh",
          0,
          new CANBus(),
          new TalonFXConfiguration()
              .withVoltage(
                  new VoltageConfigs().withPeakForwardVoltage(16).withPeakReverseVoltage(16)),
          new PivotMechSim(
              new PivotMechSim.Config(
                  DCMotor.getKrakenX60Foc(1),
                  175,
                  Meters.of(3),
                  KilogramSquareMeters.of(0.001),
                  false,
                  Degrees.of(0.0),
                  Degrees.of(90),
                  Radians.zero())));

  public Robot() {
    // Initialize Devices
    mDriverController = new CommandXboxController(0);
    mOperatorController = new CommandXboxController(1);
    mDevController = new CommandXboxController(3);

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
          config.minimumImportance = Importance.DEBUG;
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
    configureGameBehavior();
  }

  private Subsystem subsystem = new SubsystemBase() {};

  private Setpoint run = Setpoint.createVoltageSetpoint(Volts.of(12));
  private Setpoint run2 = Setpoint.createVoltageSetpoint(Volts.of(-12));

  /** Define Driver & Operator controller bindings */
  public void configureBindings() {
    io.setFocStatus(true);
    addPeriodic(() -> io.periodic(), 0.02);

    mDriverController.button(1).whileTrue(subsystem.run(() -> io.applySetpoint(run)));

    mDriverController.button(2).whileTrue(subsystem.run(() -> io.applySetpoint(run2)));

    mDriverController.button(3).whileTrue(subsystem.run(() -> io.applySetpoint(Setpoint.stop())));
  }

  /** Define behavior during different oppmodes */
  public void configureGameBehavior() {}

  @Override
  protected Command getAutonCommand() {
    return Commands.none();
  }
}
