// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;

/** Implementation of {@link DIO} for DIO devices wired to RIO */
public class DIORio extends DIO {
  private final DigitalInput dio;

  /**
   * Create new {@link DIORio}
   *
   * @param channel represents the DIO channel device is on
   */
  public DIORio(int channel) {
    this(channel, Seconds.zero(), DebounceType.kBoth);
  }

  /**
   * Create new {@link DIORio}
   *
   * @param channel represents the DIO channel device is on
   * @param debounceTime {@link Time} representing how long signal must be true for state to be true
   * @param debounceType {@link DebounceType} representing which transition the debouncer should be
   *     applied to
   */
  public DIORio(int channel, Time debounceTime, DebounceType debounceType) {
    super(debounceTime, debounceType);

    dio = new DigitalInput(0);
  }

  @Override
  public boolean getRawState() {
    return dio.get();
  }
}
