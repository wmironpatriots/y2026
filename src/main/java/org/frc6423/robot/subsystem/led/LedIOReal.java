// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;

public class LedIOReal extends LedIO {

  private final AddressableLED mAddressableLED;
  private final AddressableLEDBuffer mBuffer;

  public final AddressableLEDBufferView mLeftLedStrip;
  public final AddressableLEDBufferView mFrontLeftStrip;
  public final AddressableLEDBufferView mCenterLeftStrip;
  public final AddressableLEDBufferView mCenterStrip;
  public final AddressableLEDBufferView mCenterRightStrip;
  public final AddressableLEDBufferView mFrontRightStrip;
  public final AddressableLEDBufferView mRightStrip;

  /**
   * Creates {@link AddressableLED} 7 strips, (0-99,100-199,200-299,300-399,400-499,500-599,600-699)
   *
   * @param port {@link int} pwm port
   * @param length {@link int} length of strips (combined)
   */
  public LedIOReal(int port, int length) {
    super(port, length);
    mAddressableLED = new AddressableLED(port);
    mBuffer = new AddressableLEDBuffer(length);
    mAddressableLED.setLength(mBuffer.getLength());
    mAddressableLED.setData(mBuffer);
    mAddressableLED.start();
    this.mLeftLedStrip = mBuffer.createView(0, length / 7 - 1);
    this.mFrontLeftStrip = mBuffer.createView(length / 7, ((length / 7) * 2) - 1);
    this.mCenterLeftStrip = mBuffer.createView((length / 7) * 2, ((length / 7) * 3) - 1);
    this.mCenterStrip = mBuffer.createView((length / 7) * 3, ((length / 7) * 4) - 1);
    this.mCenterRightStrip = mBuffer.createView((length / 7) * 4, ((length / 7) * 5) - 1);
    this.mFrontRightStrip = mBuffer.createView((length / 7) * 5, ((length / 7) * 6) - 1);
    this.mRightStrip = mBuffer.createView((length / 7) * 6, length);
  }

  @Override
  public void setPixelColor(int pixel, Color color) {
    mBuffer.setLED(pixel, color);
  }

  @Override
  public void updateInputs() {
    mAddressableLED.setData(mBuffer);
  }
}
