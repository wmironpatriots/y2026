// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import java.util.ArrayList;

/**
 * A template class for Hardware Interfaces
 *
 * <p>A Hardware Interface is a layer that handles communication between robot hardware and
 * subsystem classes. These classes typically contain various public methods to control the
 * hardware.
 *
 * <p>Objects that inherit this class will automatically register into a static array that can be
 * updated using the updateDevices method. This method must be called within the robot periodic for
 * devices to update.
 *
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
 */
public abstract class IO {
  private static final ArrayList<IO> devices = new ArrayList<>();

  /** Run the periodic logic of all registered {@link IO} devices */
  public static void updateDevices() {
    for (IO io : devices) {
      io.periodic();
    }
  }

  /**
   * Register new {@link IO} device to the static {@link IO} array
   *
   * @param device {@link IO} representing device to register
   */
  private static void registerDevice(IO device) {
    devices.add(device);
  }

  /** Periodically run method */
  protected abstract void periodic();

  /** Create new {@link IO} */
  public IO() {
    registerDevice(this);
  }
}
