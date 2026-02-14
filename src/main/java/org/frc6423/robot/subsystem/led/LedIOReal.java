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

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LedIOReal(int port, int length) {
    super(port, length);
    led = new AddressableLED(LEDSubsystem.LED_PORT);
    buffer = new AddressableLEDBuffer(LEDSubsystem.LED_LENGTH);
    led.setLength(buffer.getLength());
    led.start();
  }

  @Override
  public void setPixelColor(int pixel, Color color) {
    buffer.setLED(pixel, color);
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    led.setData(buffer);
  }
}
