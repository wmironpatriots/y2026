// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import org.frc6423.robot.subsystem.shooter.components.Flywheel;
import org.frc6423.robot.subsystem.shooter.components.Hood;

public class Shooter {
  private final Hood mHood = Hood.create();
  private final Flywheel mFlywheel = Flywheel.create();
}
