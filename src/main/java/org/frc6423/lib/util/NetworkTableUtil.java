// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.util;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Utility methods for interacting with {@link NetworkTable} */
public class NetworkTableUtil {
  private static final NetworkTableInstance mNtInstance = NetworkTableInstance.getDefault();

  /**
   * Create a new {@link DoubleEntry}
   *
   * @param path entry path in {@link NetworkTable}
   * @param value entry value
   * @return {@link DoubleEntry}
   */
  public static DoubleEntry createEntry(String path, double value) {
    DoubleEntry entry = mNtInstance.getDoubleTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }

  /**
   * Create a new {@link BooleanEntry}
   *
   * @param path entry path in {@link NetworkTable}
   * @param value entry value
   * @return {@link BooleanEntry}
   */
  public static BooleanEntry createEntry(String path, boolean value) {
    BooleanEntry entry = mNtInstance.getBooleanTopic(path).getEntry(value);
    entry.set(value);

    return entry;
  }
}
