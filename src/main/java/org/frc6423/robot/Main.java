// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;
import org.frc6423.robot.Constants.RobotType;

public final class Main {
  /** Runtime flags determining the robots initialization */
  public static final class Flags {
    /** {@link RobotType} representing the robot chassis being used */
    protected static RobotType kRobotType = RobotType.Y2026_L2;

    /** When true, subsystems will not be initialized */
    protected static boolean kSubsystemDisabled = false;

    /** When true, drive will not be initialized */
    protected static boolean kDriveDisabled = false;

    /**
     * @return {@link RobotType} representing the robot chassis being used
     */
    public static RobotType getRobotType() {
      return kRobotType;
    }

    /**
     * @return true if subsystems are disabled (defaults to true when running on cascade)
     */
    public static boolean areSubsystemsDisabled() {
      return kSubsystemDisabled;
    }

    /**
     * @return true if drive subsystem is disabled
     */
    public static boolean isDriveDisabled() {
      return kDriveDisabled;
    }
  }

  private Main() {}

  public static void main(String... args) {
    var argsList = Arrays.asList(args);

    // Get RobotType
    if (argsList.contains("--y2025")) {
      System.out.println("Initializing y2025 Robot; Subsystems will not Initialize");
      Flags.kRobotType = RobotType.Y2025;
    } else {
      if (argsList.contains("--L1")) {
        System.out.println("Initializing y2026 Robot /w L1 Swerve Ratio");
        Flags.kRobotType = RobotType.Y2026_L1;
      } else if (argsList.contains("--L3")) {
        System.out.println("Initializing y2026 Robot /w L3 Swerve Ratio");
        Flags.kRobotType = RobotType.Y2026_L3;
      } else {
        System.out.println("Initializing y2026 Robot /w L2 Swerve Ratio");
        Flags.kRobotType = RobotType.Y2026_L2;
      }
    }

    // Check if subsystems should init
    if (argsList.contains("--noSubsystems")) {
      System.out.println("Initializing Without Subsystems");
      Flags.kSubsystemDisabled = true;
    }

    // Check if drive should init
    if (argsList.contains("--noDrive")) {
      System.out.println("Initializing Without Drive");
      Flags.kSubsystemDisabled = true;
    }

    RobotBase.startRobot(Robot::new);
  }
}
