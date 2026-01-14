// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class Rebuilt {
  /**
   * @return true if the alliance's hub is active
   */
  public static boolean allianceHubActive() {
    String gameData = DriverStation.getGameSpecificMessage();
    var alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      alliance = Optional.of(Alliance.Red);
    }

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'R':
          return alliance.get() == Alliance.Red ? true : false;
        case 'B':
          return alliance.get() == Alliance.Blue ? true : false;
        default:
          return false;
      }
    } else return false;
  }
}
