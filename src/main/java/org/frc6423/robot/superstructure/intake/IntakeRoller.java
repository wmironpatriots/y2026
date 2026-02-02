// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.superstructure.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc6423.lib.driver.FlywheelSubsystem;
import org.frc6423.lib.io.DIO;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;

/**
 * {@link FlywheelSubsystem} representing the rolling component of the {@link Intake}
 *
 * <p>{@link IntakeRoller} is an extension of the {@link FlywheelSubsystem}
 *
 * <p>{@link IntakeRoller} has four states: STOPPED, INTAKING, OUTAKING, STUCK
 */
public class IntakeRoller extends FlywheelSubsystem {
  // * SETPOINTS
  public static final Setpoint kIntakingSetpoint =
      Setpoint.createVelocitySetpoint(RevolutionsPerSecond.of(10));
  public static final Setpoint kOutakingSetpoint =
      Setpoint.createVelocitySetpoint(RevolutionsPerSecond.of(-10));

  // * LOGIC
  private State mState = State.STOPPED;

  /**
   * Create new {@link IntakeRoller} subsystem
   *
   * @param servo {@link ServoIO} representing servo driving subsystem
   * @param epsilon largest allowed percent error for angular velocity control
   */
  public IntakeRoller(ServoIO servo, DIO beambreak, double epsilon) {
    super(servo, epsilon);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (mState != State.STOPPED) {
      if (getAngularVelocity().gt(RadiansPerSecond.zero())
          || getAngularVelocity().lt(RadiansPerSecond.zero())) mState = State.STUCK;
      else if (mState == State.STUCK)
        mState = getAngularVelocity().gt(RadiansPerSecond.zero()) ? State.INTAKING : State.OUTAKING;
    }
  }

  /**
   * @return {@link State} representing the current action the subsystem is performing
   */
  @Logged(name = "State", importance = Importance.CRITICAL)
  public State getState() {
    return mState;
  }

  /**
   * Request subsystem to idle
   *
   * @return {@link Command}
   */
  public Command stop() {
    return stop().andThen(setState(State.STOPPED)).withName("IntakeRoller Idle");
  }

  /**
   * Request subsystem to start intaking
   *
   * @return {@link Command}
   */
  public Command startIntaking() {
    return setSetpoint(kIntakingSetpoint)
        .andThen(setState(State.INTAKING))
        .withName("IntakeRoller Intaking");
  }

  /**
   * Request subsystem to start outaking
   *
   * @return {@link Command}
   */
  public Command startOutaking() {
    return setSetpoint(kOutakingSetpoint)
        .andThen(setState(State.OUTAKING))
        .withName("IntakeRoller Outaking");
  }

  /**
   * Set current state value
   *
   * @param state {@link State}
   * @return {@link Command}
   */
  private Command setState(State state) {
    return this.runOnce(() -> mState = state);
  }

  /** Represents an action an {@link IntakeRoller} subsystem can do */
  public static enum State {
    /** {@link IntakeRoller} is not moving */
    STOPPED,
    /** {@link IntakeRoller} is pulling balls towards the robot */
    INTAKING,
    /** {@link IntakeRoller} is pushing balls away from the robot */
    OUTAKING,
    /** {@link IntakeRoller} is unable to intake/outake */
    STUCK
  }
}
