// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.io;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class DIONone extends DIO {
  public DIONone() {
    super(Seconds.zero(), DebounceType.kBoth);
  }

  @Override
  public boolean getRawState() {
    return false;
  }
}
