// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants;
import org.frc6423.robot.subsystem.drive.constants.DriveConstants.ModuleConfig;

public abstract class SwerveModuleIO {
  public final String mName;

  protected final ModuleConfig mConfig;
  protected final DriveConstants mConstants;

  public SwerveModuleIO(String name, ModuleConfig config, DriveConstants constants) {
    mName = name;

    mConfig = config;
    mConstants = constants;
  }

  public void periodic() {}

  /**
   * Get angular position of module wheel
   *
   * @return {@link Rotation2d}
   */
  public abstract Rotation2d getRotation2d();

  /**
   * Get linear velocity of module wheel
   *
   * @return {@link LinearVelocity}
   */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(
        mConstants
            .getWheelRadius()
            .times(getWheelAngularVelocity().in(RadiansPerSecond))
            .in(Meters));
  }

  /**
   * Get linear accleration of module wheel
   *
   * @return {@link LinearAcceleration}
   */
  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(
        mConstants
            .getWheelRadius()
            .times(getWheelAngularAcceleration().in(RadiansPerSecondPerSecond))
            .in(Meters));
  }

  /**
   * Get angular velocity of module wheel
   *
   * @return {@link AngularVelocity}
   */
  public abstract AngularVelocity getWheelAngularVelocity();

  /**
   * Get angular acceleration of module wheel
   *
   * @return {@link AngularAcceleration}
   */
  public abstract AngularAcceleration getWheelAngularAcceleration();

  /** Stop module completely */
  public abstract void stop();
}
