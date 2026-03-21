// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
  // public static final int kFirstLength = 15;
  // public static final int kSecondLength = 39;
  // public static final int kThirdLength = 15;
  public static final int kLength = 15 + 39 + 15;
  public static final Distance kDensity = Meters.of(1 / 120.0);

  private final AddressableLED mAddressable;
  private final AddressableLEDBuffer mBuffer;

  public Led() {
    mAddressable = new AddressableLED(0);
    mBuffer = new AddressableLEDBuffer(kLength);

    mAddressable.setLength(kLength);
    mAddressable.start();

    setDefaultCommand(off());
  }

  @Override
  public void periodic() {
    mAddressable.setData(mBuffer);
  }

  public Command strobe(Color c1, Color c2, double period) {
    return Commands.sequence(
            solid(c1), Commands.waitSeconds(period), solid(c2), Commands.waitSeconds(period))
        .repeatedly();
  }

  public Command strobe(Color color, double period) {
    return Commands.sequence(
            solid(color), Commands.waitSeconds(period), off(), Commands.waitSeconds(period))
        .repeatedly();
  }

  public Command breath(Color color, double period) {
    return setPattern(LEDPattern.solid(color).breathe(Seconds.of(period)));
  }

  public Command chase(LinearVelocity speed, Color c1, Color c2) {
    return setPattern(
        LEDPattern.gradient(GradientType.kDiscontinuous, c1, c2)
            .scrollAtAbsoluteSpeed(speed, kDensity));
  }

  public Command rainbow(LinearVelocity speed) {
    return setPattern(LEDPattern.rainbow(225, 128).scrollAtAbsoluteSpeed(speed, kDensity));
  }

  public Command solid(Color color) {
    return setPattern(LEDPattern.solid(color));
  }

  public Command off() {
    return setPattern(LEDPattern.kOff);
  }

  public Command setPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(mBuffer));
  }
}
