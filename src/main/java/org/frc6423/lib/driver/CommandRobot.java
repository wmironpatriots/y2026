// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.driver;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc6423.lib.util.Tracer;

/**
 * {@link CommandRobot} is an extension of {@link TimedRobot}
 *
 * <p>The {@link CommandRobot} class is intended to subclassed by a user creating a Robot Program
 *
 * <p>The {@link CommandScheduler} is automatically called and traced by {@link Tracer} every period
 *
 * <p>Abstract method getAutonomousCommand() is automatically scheduled on autonomous initialization
 *
 * <p>Garbage collector automatically called every 5 seconds
 *
 * @see https://github.com/wpilibsuite/allwpilib/pull/5939
 * @see {@link TimedRobot}
 */
public abstract class CommandRobot extends TimedRobot {
  private final CommandScheduler mScheduler = CommandScheduler.getInstance();

  private Command mAutonCmd = Commands.none();

  private final Timer gcTimer = new Timer();

  /** Create new {@link CommandRobot} with looptime of 20 milliseconds */
  public CommandRobot() {
    this(Seconds.of(kDefaultPeriod));
  }

  /**
   * Create new {@link CommandRobot} with specified looptime
   *
   * @param period looptime in period
   */
  public CommandRobot(Time period) {
    super(period.in(Seconds));

    // Start garbage collection timer
    gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    // Run and trace CommandScheduler
    Tracer.traceFunc("CommandScheduler", mScheduler::run);

    // Run Garbage Collector every 5 seconds
    if (gcTimer.hasElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void disabledInit() {
    mScheduler.cancelAll();
    System.gc();
  }

  @Override
  public void disabledExit() {
    mScheduler.cancelAll();
    System.gc();
  }

  @Override
  public void autonomousInit() {
    mAutonCmd = getAutonCmd();

    if (mAutonCmd != null) {
      mScheduler.schedule(mAutonCmd);
    }
  }

  @Override
  public void autonomousExit() {
    mScheduler.cancelAll();
    System.gc();
  }

  @Override
  public void teleopInit() {
    if (mAutonCmd != null) {
      mAutonCmd.cancel();
    }
  }

  @Override
  public void teleopExit() {
    mScheduler.cancelAll();
    System.gc();
  }

  /**
   * @return {@link Command} to schedule on autonomous oppmode initialization
   */
  protected abstract Command getAutonCmd();
}
