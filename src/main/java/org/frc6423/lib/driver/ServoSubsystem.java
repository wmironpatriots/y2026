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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;

/** Base {@link SubsystemBase} for a subsystem that uses at least one {@link ServoIO} */
public class ServoSubsystem extends SubsystemBase {
  @Logged protected final ServoIO leader;
  @Logged protected final Optional<ServoIO[]> followers;

  /**
   * Create new {@link ServoSubsystem}
   *
   * @param servo {@link ServoIO} representing servo driving subsystem
   */
  public ServoSubsystem(ServoIO servo) {
    this(servo, new ServoIO[] {}, new boolean[] {});
  }

  /**
   * Create new coupled {@link ServoSubsystem}
   *
   * <p>A coupled servo subsystem is where one or more follower servos copy the actions of one
   * leader servo
   *
   * @param leader {@link ServoIO} representing leader servo
   * @param followers {@link ServoIO} array representing follower servos
   * @param followersFlipped {@link boolean} array storing the follow direction of each follower
   */
  public ServoSubsystem(ServoIO leader, ServoIO[] followers, boolean[] followersFlipped) {
    this.leader = leader;
    if (followers.length > 0) {
      this.followers = Optional.of(followers);

      for (int i = 0; i < followers.length; i++) {
        followers[i].setLeader(leader, followersFlipped[i]);
      }
    } else this.followers = Optional.empty();
  }

  @Override
  public void periodic() {
    leader.periodic();

    if (followers.isPresent()) {
      for (var follower : followers.get()) {
        follower.periodic();
      }
    }
  }

  /**
   * @return true if leader servo is disabled due to overheating
   */
  public boolean isLeaderOverheated() {
    return leader.isOverheated();
  }

  /**
   * @return false if any follower servo is disabled due to overheating; Returns false if subsystem
   *     has no followers
   */
  public boolean areFollowersOverheated() {
    if (followers.isPresent()) {
      Boolean[] overheated = new Boolean[followers.get().length];
      for (int i = 0; i < followers.get().length; i++) {
        overheated[i] = followers.get()[i].isOverheated();
      }

      return Arrays.asList(overheated).contains(true);
    }

    return false;
  }

  /**
   * @return current {@link Setpoint}
   */
  public Setpoint getCurrentSetpoint() {
    return leader.getCurrentSetpoint();
  }

  /**
   * @return {@link Angle} representing the angular position of subsystem
   */
  public Angle getAngle() {
    return leader.getAngle();
  }

  /**
   * @return {@link AngularVelocity} representing the angular velocity of servo
   */
  public AngularVelocity getAngularVelocity() {
    return leader.getAngularVelocity();
  }

  /**
   * @return {@link AngularAcceleration} representing the angular acceleration of servo
   */
  public AngularAcceleration getAngularAcceleration() {
    return leader.getAngularAcceleration();
  }

  /**
   * Run sepcified {@link Setpoint} until interrupted
   *
   * @param setpointSupplier {@link Setpoint} representing desired state
   * @return {@link Command}
   */
  public Command runSetpointCmd(Setpoint setpoint) {
    return runSetpointCmd(() -> setpoint);
  }

  /**
   * Run sepcified {@link Setpoint} until interrupted
   *
   * @param setpointSupplier {@link Setpoint} representing desired state
   * @return {@link Command}
   */
  protected Command runSetpointCmd(Supplier<Setpoint> setpointSupplier) {
    return this.runEnd(() -> leader.applySetpoint(setpointSupplier.get()), () -> leader.stop())
        .withName("Run Setpoint");
  }

  /**
   * Set servo setpoint
   *
   * @param setpoint {@link Setpoint} representing desired state
   * @return {@link Command}
   */
  protected Command setSetpointCmd(Setpoint setpoint) {
    return this.runOnce(() -> leader.applySetpoint(setpoint)).withName("Set Setpoint");
  }

  /**
   * Stop subsystem
   *
   * @return {@link Command}
   */
  public Command stop() {
    return this.run(() -> leader.stop());
  }
}
