// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart0;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart1;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart10;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart11;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart12;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart13;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart14;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart15;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart16;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart2;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart3;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart4;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart5;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart6;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart7;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart8;
import org.frc6423.robot.subsystem.led.BadAppleData.BadApplePart9;

/** {@link SubsystemBase} extension representing the LED subsystem */
public class LED extends SubsystemBase {
  public static final int kStripPort = 67;
  public static final int kStripLength = 100 * 7;

  private final LedIO mLedStrip = new LedIOReal(kStripPort, kStripLength);
  private double rainbowStart = 0;

  private class BadAppleRegistry {
    // A simple array of arrays
    public static final String[][] ALL_PARTS = {
      BadApplePart0.DATA, BadApplePart1.DATA, BadApplePart2.DATA,
      BadApplePart3.DATA, BadApplePart4.DATA, BadApplePart5.DATA,
      BadApplePart6.DATA, BadApplePart7.DATA, BadApplePart8.DATA,
      BadApplePart9.DATA, BadApplePart10.DATA, BadApplePart11.DATA,
      BadApplePart12.DATA, BadApplePart13.DATA, BadApplePart14.DATA,
      BadApplePart15.DATA, BadApplePart16.DATA
    };

    public static String[] getPart(int index) {
      if (index >= 0 && index < ALL_PARTS.length) {
        return ALL_PARTS[index];
      }
      return new String[0];
    }
  }

  public void setStripSolid(Color color, int low, int high) {
    for (int i = low; i < high; i++) {
      mLedStrip.setPixelColor(i, color);
    }
    mLedStrip.updateInputs();
  }

  public void setStripStrobe(Color color1, Color color2, double period, int low, int high) {
    boolean useC1 = ((Timer.getTimestamp() % period) / period) > 0.5;
    setStripSolid(useC1 ? color1 : color2, low, high);
  }

  public Command setSolidCmd(Color color, int low, int high) {
    return this.run(() -> setStripSolid(color, low, high));
  }

  public Command setRainbowCmd(int low, int high) {
    return this.run(
        () -> {
          for (int i = low; i < high; i++) {
            mLedStrip.setPixelColor(i, Color.fromHSV((int) rainbowStart % 180 + i, 255, 255));
          }
          rainbowStart += 6;
        });
  }

  public Command setWaveCmd(int low, int high) {
    return this.run(
        () -> {
          setRainbowCmd(low, high);
        });
  }

  public Command setBlinkingCmd(
      Color onColor, Color offColor, double frequency, int low, int high) {
    return Commands.repeatingSequence(
        setSolidCmd(onColor, low, high).withTimeout(1.0 / frequency),
        setSolidCmd(offColor, low, high).withTimeout(1.0 / frequency));
  }

  // 14 * 50 --- 2/50 blocks for 1 strip

  private String getBadAppleData(int dataBlock, int chunk) {
    String data = BadAppleRegistry.getPart(dataBlock)[chunk];
    return (data);
  }

  private int[] convertStringtoIntArray(String chunk) {
    int[] data = new int[chunk.length()];
    for (int i = 0; i < chunk.length(); i++) {
      data[i] = chunk.charAt(i) - '0';
    }
    return (data);
  }

  private void convertIntArraytoPixels(int[] chunk) {
    for (int i = 0; i < chunk.length; i++) {
      boolean useWhite = (chunk[i] == 1);
      mLedStrip.setPixelColor(i, useWhite ? Color.fromHSV(0, 0, 255) : Color.fromHSV(0, 0, 0));
    }
  }

  public Command playBadApple(int fps) {
    return this.run(
        () -> {
          for (int i = 0; i < BadAppleRegistry.ALL_PARTS.length; i++) {
            for (int n = 0; i < BadAppleRegistry.ALL_PARTS.length; i++) {
              convertIntArraytoPixels(convertStringtoIntArray(getBadAppleData(i, n)));
              mLedStrip.updateInputs();
              try {
                Thread.sleep(1000 / fps);
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            }
          }
        });
  }
}
