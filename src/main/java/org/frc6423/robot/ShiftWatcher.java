// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.frc6423.robot.Constants.Flags;

public class ShiftWatcher {
  public static enum Shift {
    DISABLED(0.0),
    AUTONOMOUS(20.0),
    TRANSITION(30.0),
    SHIFT_1(55.0),
    SHIFT_2(80.0),
    SHIFT_3(105.0),
    SHIFT_4(130.0),
    END_GAME(160.0);

    public final double startingTimestamp;

    private Shift(double startTimestamp) {
      this.startingTimestamp = startTimestamp;
    }
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

  public static double getShiftTime() {
    return kShiftTimer.get() + kShiftTimerOffset;
  }

  public static boolean isActive() {
    var startingAlliance = getStartingAlliance();
    if (startingAlliance.isEmpty()) return false;

    var activeShift = getActiveShift();

    if (activeShift == Shift.AUTONOMOUS
        || activeShift == Shift.TRANSITION
        || activeShift == Shift.SHIFT_2
        || activeShift == Shift.SHIFT_4
        || activeShift == Shift.END_GAME) {
      return startingAlliance.get().equals(Flags.getRobotAlliance());
    } else {
      return false;
    }
  }

  public static Optional<Alliance> getStartingAlliance() {
    var fmsData = DriverStation.getGameSpecificMessage();

    if (fmsData.length() > 0) {
      if (fmsData.charAt(0) == 'R') {
        return Optional.of(Alliance.Blue);
      } else if (fmsData.charAt(0) == 'B') {
        return Optional.of(Alliance.Red);
      } else return Optional.empty();
    } else return Optional.empty();
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
