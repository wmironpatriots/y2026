// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.command;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Namespace for custom command factory methods.
 *
 * <p>For convenience, you might want to static import the members of this class.
 */
public class IronCommands {
  /**
   * Run a group of commands in series as fast as possible
   *
   * @param commands {@link Command} sequence should run
   * @return {@link Commadn}
   */
  public static Command fastSequence(Command... commands) {
    return new FastSequentialCommandGroup(commands);
  }
}
