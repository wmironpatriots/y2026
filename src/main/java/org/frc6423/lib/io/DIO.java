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
 * An {@link IO} for controlling a DIO device
 *
 * @see https://docs.wpilib.org/en/stable/docs/hardware/sensors/digital-inputs-hardware.html
 */
public abstract class DIO extends IO {
  private final Debouncer debouncer;

  /**
   * Create new {@link DIO} /w a debouncer
   *
   * @param debounceTime {@link Time} representing how long signal must be true for state to be true
   * @param debounceType {@link DebounceType} representing which transition the debouncer should be
   *     applied to
   */
  public DIO(Time debounceTime, DebounceType debounceType) {
    debouncer = new Debouncer(debounceTime.in(Seconds), debounceType);
  }

  @Override
  protected void periodic() {
    // Update debouncer periodically
    getState();
  }

  /**
   * @return true if state debouncer returns 1
   */
  public boolean getState() {
    return debouncer.calculate(getRawState());
  }

  /**
   * @return true if DIO port returns 1
   */
  @Logged(name = "RawState", importance = Importance.INFO)
  public abstract boolean getRawState();
}
