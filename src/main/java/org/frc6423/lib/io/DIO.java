// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;

/**
 * A Hardware Interface for controlling a Digital Input device A {@link ServoIO} instance
 * <strong>must</strong> have its <strong>periodic method called every robot loop for values to be
 * properly logged</strong>
 *
 * @see https://docs.wpilib.org/en/stable/docs/hardware/sensors/digital-inputs-hardware.html
 * @see https://en.wikipedia.org/wiki/Digital_signal
 */
public abstract class DIO {
  private final Debouncer debouncer;

  /**
   * Create new {@link DIO} /w a {@link Debouncer}
   *
   * @param debounceTime {@link Time} representing how long signal must be true for state to be true
   * @param debounceType {@link DebounceType} representing which transition the debouncer should be
   *     applied to
   */
  public DIO(Time debounceTime, DebounceType debounceType) {
    debouncer = new Debouncer(debounceTime.in(Seconds), debounceType);
  }

  /** Update all logged values */
  public void periodic() {
    // Update debouncer
    getState();
  }

  /**
   * @return true when debouncer calculates signal to be on
   */
  @Logged(name = "State", importance = Importance.INFO)
  public boolean getState() {
    return debouncer.calculate(getRawState());
  }

  /**
   * @return true when signal is on
   */
  @Logged(name = "RawState", importance = Importance.INFO)
  public abstract boolean getRawState();
}
