// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedIOReal extends LedIO {

  private final AddressableLED mAddressableLED;
  private final AddressableLEDBuffer mbuffer;

  public LedIOReal(int port, int length) {
    super(port, length);
    mAddressableLED = new AddressableLED(port);
    mbuffer = new AddressableLEDBuffer(length);
    mAddressableLED.setLength(mbuffer.getLength());
    mAddressableLED.start();
  }

  @Override
  public void setPixelColor(int pixel, Color color) {
    mbuffer.setLED(pixel, color);
  }
}
