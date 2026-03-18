// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc6423.robot.Constants.Flags;

/** Static watcher for keeping track of the state of match */
public class MatchInfo {
  /** Period of match */
  public static enum Shift {
    /** Disabled (no time) (no active) */
    DISABLED(0.0),
    /** Autonomous Period (20 seconds) (both active) */
    AUTONOMOUS(20.0),
    /** Transition Shift (10 seconds) (both active) */
    TRANSITION(30.0),
    /** Shift 1 (25 seconds) (loosing alliance active) */
    SHIFT_1(55.0),
    /** Shift 2 (25 seconds) (winning alliance active) */
    SHIFT_2(80.0),
    /** Shift 3 (25 seconds) (loosing alliance active) */
    SHIFT_3(105.0),
    /** Shift 4 (25 seconds) (winning alliance active) */
    SHIFT_4(130.0),
    /** Endgame (30 seconds) (both active) */
    END_GAME(160.0);

    public final double startingTimestamp;

    private Shift(double startTimestamp) {
      this.startingTimestamp = startTimestamp;
    }
  }

  static {
  }

  private static final Timer kShiftTimer = new Timer();
  private static double kShiftTimerOffset = 0.0;

  public static void initialize() {
    if (DriverStation.isAutonomousEnabled()) {
      kShiftTimer.reset();
      kShiftTimerOffset = 0.0;
    }
    if (DriverStation.isTeleopEnabled()) {
      kShiftTimer.start();
      kShiftTimer.restart();
      kShiftTimerOffset = Shift.AUTONOMOUS.startingTimestamp;
    }
  }

  public static void log() {
    SmartDashboard.putNumber("Match/Match Time", getMatchTime());
    SmartDashboard.putString("Match/Current Shift", getActiveShift().toString());
    SmartDashboard.putString("Match/Starting Alliance", getStartingAlliance().toString());
    SmartDashboard.putBoolean("Match/Is Hub Active (bool)", isActive());
    SmartDashboard.putNumber("Match/Remaining Shift Time", getRemainingShiftTime());
  }

  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public static void stop() {
    kShiftTimer.stop();
  }

  public static double getShiftTime() {
    return kShiftTimer.get() + kShiftTimerOffset;
  }

  public static boolean isActive() {
    var activeShift = getActiveShift();
    var isStartingAlliance = getStartingAlliance().equals(Flags.getRobotAlliance());

    if (activeShift == Shift.AUTONOMOUS
        || activeShift == Shift.TRANSITION
        || activeShift == Shift.END_GAME) {
      return true;
    }
    if (activeShift == Shift.SHIFT_1 || activeShift == Shift.SHIFT_3) {
      return isStartingAlliance;
    } else {
      return !isStartingAlliance;
    }
  }

  public static Alliance getStartingAlliance() {
    var robotAlliance = Flags.getRobotAlliance();

    var fmsData = DriverStation.getGameSpecificMessage();
    if (fmsData.length() > 0) {
      if (fmsData.charAt(0) == 'R') {
        return Alliance.Blue;
      } else if (fmsData.charAt(0) == 'B') {
        return Alliance.Red;
      }
    }

    return (robotAlliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
  }

  public static double getRemainingShiftTime() {
    var activeShift = getActiveShift();
    var recordedShiftTime = getShiftTime();

    return activeShift.startingTimestamp - recordedShiftTime;
  }

  public static Shift getActiveShift() {
    for (int i = 0; i < Shift.values().length; i++) {
      var recordedShiftTime = getShiftTime();
      if (recordedShiftTime < Shift.values()[i].startingTimestamp) {
        return Shift.values()[i];
      }
    }

    return Shift.DISABLED;
  }
}
