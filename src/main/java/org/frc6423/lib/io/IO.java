// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import java.util.ArrayList;

/**
 * A template hardware interface class
 *
 * <p>For {@link IO} devices to actually update every period, the updateDevices method must be
 * called every period in the robot periodic
 */
public abstract class IO {
  private static final ArrayList<IO> devices = new ArrayList<>();

  /** Update all registered {@link IO} devices */
  public static void updateDevices() {
    for (IO io : devices) {
      io.periodic();
    }
  }

  /**
   * Register new {@link IO} device
   *
   * @param device {@link IO} representing device to register
   */
  private static void registerDevice(IO device) {
    devices.add(device);
  }

  /** Periodic ran logic for device */
  protected abstract void periodic();

  /** Create new {@link IO} */
  public IO() {
    registerDevice(this);
  }
}
