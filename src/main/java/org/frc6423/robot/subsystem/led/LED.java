// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.led;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** {@link SubsystemBase} extension representing the LED subsystem */
public class LED extends SubsystemBase {
  public static final int kStripPort = 67;
  public static final int kStripLength = 100 * 7;

  private final LedIO mLedStrip = new LedIOReal(kStripPort, kStripLength);
  private double rainbowStart = 0;

  public void setStripSolid(LedIO strip, Color color, int low, int high) {
    for (int i = low; i < high; i++) {
      strip.setPixelColor(i, color);
    }
    mLedStrip.updateInputs();
  }

  public void setStripStrobe(
      LedIO strip, Color color1, Color color2, double period, int low, int high) {
    boolean useC1 = ((Timer.getTimestamp() % period) / period) > 0.5;
    setStripSolid(strip, useC1 ? color1 : color2, low, high);
  }

  public void setRainbow(int low, int high) {
    for (int i = low; i < high; i++) {
      mLedStrip.setPixelColor(i, Color.fromHSV((int) rainbowStart % 180 + i, 255, 255));
      rainbowStart += 6;
    }
    mLedStrip.updateInputs();
  }
}
