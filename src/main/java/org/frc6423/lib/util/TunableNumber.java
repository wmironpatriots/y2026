// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.util;

import edu.wpi.first.networktables.DoubleEntry;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.frc6423.robot.Constants.Flags;

/**
 * Stolen from FRC 6328 (modified to use a NT Double Entry instead)
 *
 * <p>Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not
 * or value not in dashboard.
 */
@SuppressWarnings("unused")
public class TunableNumber implements DoubleSupplier {
  private static final String kTableKey = "/Tuning";

  private final String mKey;
  private boolean mHasDefault = false;
  private double mDefaultValue;
  private DoubleEntry mNtEntry;
  private Map<Integer, Double> mLastHasChangedValues = new HashMap<>();

  /**
   * Create a new {@link TunableNumber}
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.mKey = kTableKey + "/" + dashboardKey;
  }

  /**
   * Create a new {@link TunableNumber} with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!mHasDefault) {
      mHasDefault = true;
      this.mDefaultValue = defaultValue;
      if (Flags.kTuningModeEnabled) {
        mNtEntry = NetworkTableUtil.createEntry(mKey, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!mHasDefault) {
      return 0.0;
    } else {
      return Flags.kTuningModeEnabled ? mNtEntry.get() : mDefaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = mLastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      mLastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(int id, Consumer<double[]> action, TunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableNumber::get).toArray());
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, TunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
