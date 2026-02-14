// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** {@link SubsystemBase} extension representing the LED subsystem */
@SuppressWarnings("unused")
public class LED extends SubsystemBase {

  // When looking at the center strip

  public static final int kLeftStripPort = 67;
  public static final int kLeftStripLength = 69;
  private final LedIO mLeftLedStrip = new LedIOReal(kLeftStripPort, kLeftStripLength);

  public static final int kFrontLeftStripPort = 67;
  public static final int kFrontLeftStripLength = 69;
  private final LedIO mFrontLeftStrip = new LedIOReal(kFrontLeftStripPort, kFrontLeftStripLength);

  public static final int kCenterLeftStripPort = 67;
  public static final int kCenterLeftStripLength = 69;
  private final LedIO mCenterLeftStrip =
      new LedIOReal(kCenterLeftStripPort, kCenterLeftStripLength);

  public static final int kCenterStripPort = 67;
  public static final int kCenterStripLength = 69;
  private final LedIO mCenterStrip = new LedIOReal(kCenterStripPort, kCenterStripLength);

  public static final int kCenterRightStripPort = 67;
  public static final int kCenterRightStripLength = 69;
  private final LedIO mCenterRightStrip =
      new LedIOReal(kCenterRightStripPort, kCenterRightStripLength);

  public static final int kFrontRightStripPort = 67;
  public static final int kFrontRightStripLength = 69;
  private final LedIO mFrontRightStrip =
      new LedIOReal(kFrontRightStripPort, kFrontRightStripLength);

  public static final int kRightStripPort = 67;
  public static final int kRightStripLength = 69;
  private final LedIO mRightStrip = new LedIOReal(kRightStripPort, kRightStripLength);
}
