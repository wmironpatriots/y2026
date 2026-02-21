// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Utility methods for the Phoneix 6 Vendor */
public class PhoneixUtils {
  /**
   * Attempt to run Phoneix 6 Request until it returns successful
   *
   * @param maxAttempts {@link Integer} representing maximum amount of times to retry until giving
   *     up
   * @param command {@link StatusCode} supplier representing the request to run
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }
}
