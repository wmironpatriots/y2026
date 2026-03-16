// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Main contains the entry point of the robot program {@link #main(String...)}
 *
 * <p><strong> NO INITIALIZATION SHOULD HAPPEN HERE </strong>
 *
 * <p>This file should only be modified if you know what you are doing
 */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
