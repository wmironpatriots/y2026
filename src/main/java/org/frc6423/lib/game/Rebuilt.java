// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.game;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

/** Rebuilt specific static methods/values */
public class Rebuilt {
  public static final Time MATCH_TIME = Seconds.of(2 * 60 + 20);
  public static final Time SHIFT_ZERO_END = Seconds.of(2 * 60 + 10);
  public static final Time SHIFT_ONE_END = Seconds.of(60 + 45);
  public static final Time SHIFT_TWO_END = Seconds.of(60 + 20);
  public static final Time SHIFT_THREE_END = Seconds.of(55);
  public static final Time SHIFT_FOUR_END = Seconds.of(30);

  /**
   * @return true if the alliance's hub is active
   */
  public static boolean isAllianceHubActive() {
    var staringAlliance = getStartingInactiveAlliance();
    var alliance = DriverStation.getAlliance();
    var timestamp = Seconds.of(Timer.getFPGATimestamp());

    if (alliance.isEmpty()) {
      alliance = Optional.of(Alliance.Red);
    }

    if (staringAlliance.isEmpty() || timestamp.gt(SHIFT_ZERO_END) || timestamp.lt(SHIFT_FOUR_END)) {
      return true;
    } else if ((timestamp.lte(SHIFT_ZERO_END) && timestamp.gt(SHIFT_ONE_END))
        || (timestamp.lte(SHIFT_TWO_END) && timestamp.gt(SHIFT_THREE_END))) {
      return staringAlliance.get() != alliance.get();
    } else {
      return staringAlliance.get() == alliance.get();
    }
  }

  /**
   * Returns the first inactive alliance in the match
   *
   * <p>If autonomous scores have not been proccessed, this method will return an empty {@link
   * Optional}
   *
   * @return {@link Optional} of {@link Alliance}
   */
  public static Optional<Alliance> getStartingInactiveAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'R':
          return Optional.of(Alliance.Red);
        case 'B':
          return Optional.of(Alliance.Blue);
        default:
          return Optional.empty();
      }
    } else return Optional.empty();
  }
}
