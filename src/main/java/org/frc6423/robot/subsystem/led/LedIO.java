// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.led;
import edu.wpi.first.wpilibj.util.Color;

/** A Hardware Interface for controlling a strip of LEDs */
public abstract class LedIO {

    public final int mPort;
    public final int mLength;

    /**
     * Create {@link LedIO}
     * 
     * @param port {@link int} PWM port LEDs are connected to
     * @param length {@link int} representing length in pixels
     */

    public LedIO(int port, int length){
        mPort = port;
        mLength = length;
    }

    /**
     * Set color of specified pixel
     * 
     * @param pixel {@link Integer} representing the pixel that you want to set
     * @param color {@link Color} representing the color you want the pixel to display
     */
    public abstract void setPixelColor(int pixel, Color color);

}
