// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.driver;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;

/**
 * {@link SubsystemBase} for a subsystem that uses at least one {@link ServoIO}
 *
 * <p>Control of this subsystem is centered around two Commands:
 *
 * <p>the setSetpointCmd will set a setpoint
 *
 * <p>the runSetpointCmd will continuously run a Setpoint from a supplier until the command is
 * interrupted
 */
public class MotorSubsystem extends SubsystemBase {
  @Logged protected final ServoIO mLeader;
  @Logged protected final Optional<ServoIO[]> mFollowers;

  /**
   * Create new {@link MotorSubsystem}
   *
   * @param servo {@link ServoIO} representing the servo driving subsystem
   */
  public MotorSubsystem(ServoIO servo) {
    this(servo, new ServoIO[] {}, new boolean[] {});
  }

  /**
   * Create new coupled {@link MotorSubsystem}
   *
   * <p>A coupled motor subsystem is where one or more follower servos copy the actions of one
   * leader servo
   *
   * @param leader {@link ServoIO} representing leader servo
   * @param followers {@link ServoIO} array representing follower servos
   * @param followersFlipped {@link boolean} array storing the follow direction of each follower
   */
  public MotorSubsystem(ServoIO leader, ServoIO[] followers, boolean[] followersFlipped) {
    this.mLeader = leader;
    if (followers.length > 0) {
      this.mFollowers = Optional.of(followers);

      for (int i = 0; i < followers.length; i++) {
        followers[i].setLeader(leader, followersFlipped[i]);
      }
    } else this.mFollowers = Optional.empty();
  }

  @Override
  public void periodic() {
    mLeader.periodic();

    if (mFollowers.isPresent()) {
      for (var follower : mFollowers.get()) {
        follower.periodic();
      }
    }
  }

  /**
   * @return {@link Setpoint} of subsystem
   */
  public Setpoint getSetpoint() {
    return mLeader.getSetpoint();
  }

  /**
   * @return {@link Voltage} representing the applied voltage of subsystem servo
   */
  public Voltage getAppliedVoltage() {
    return mLeader.getAppliedVoltage();
  }

  /**
   * @return {@link Current} representing the input supply current of subsystem servo
   */
  public Current getSupplyCurrent() {
    return mLeader.getSupplyCurrent();
  }

  /**
   * @return {@link Current} representing the output stator current of subsystem servo
   */
  public Current getStatorCurrent() {
    return mLeader.getStatorCurrent();
  }

  /**
   * @return {@link Current} representing the torque current output of subsystem servo
   */
  public Current getTorqueCurrent() {
    return mLeader.getTorqueCurrent();
  }

  /**
   * @return {@link Angle} representing the angular position of subsystem
   */
  public Angle getAngle() {
    return mLeader.getAngle();
  }

  /**
   * @return {@link AngularVelocity} representing the angular velocity of subsystem
   */
  public AngularVelocity getAngularVelocity() {
    return mLeader.getAngularVelocity();
  }

  /**
   * @return {@link AngularAcceleration} representing the angular acceleration of subsystem
   */
  public AngularAcceleration getAngularAcceleration() {
    return mLeader.getAngularAcceleration();
  }

  /**
   * Follows specified {@link Setpoint} until interrupted
   *
   * @param setpointSupplier {@link Setpoint} representing desired state
   * @return {@link Command}
   */
  protected Command followSetpointCmd(Setpoint setpoint) {
    return followSetpointCmd(() -> setpoint);
  }

  /**
   * Follows specified {@link Setpoint} until interrupted
   *
   * @param setpointSupplier {@link Setpoint} representing desired state
   * @return {@link Command}
   */
  protected Command followSetpointCmd(Supplier<Setpoint> setpointSupplier) {
    return this.runEnd(() -> mLeader.applySetpoint(setpointSupplier.get()), () -> mLeader.stop())
        .withName("Run Setpoint");
  }

  /**
   * Set servo setpoint
   *
   * @param setpoint {@link Setpoint} representing desired state
   * @return {@link Command}
   */
  protected Command setSetpointCmd(Setpoint setpoint) {
    return this.runOnce(() -> mLeader.applySetpoint(setpoint)).withName("Set Setpoint");
  }

  /**
   * Stop subsystem
   *
   * @return {@link Command}
   */
  public Command stop() {
    return this.run(() -> mLeader.stop());
  }
}
