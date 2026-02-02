// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc6423.lib.driver.ServoSubsystem;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIO.Setpoint;
import org.frc6423.robot.Constants.IntakeConstants;

/**
 * {@link ServoSubsystem} representing the pivoting component of the {@link Intake}
 *
 * <p>{@link IntakePivot} is an extension of the {@link ServoSubsystem}
 *
 * <p>{@link IntakePivot} has five {@link State}s: STOWING, STOWED, DEPLOYING, DEPLOYED, & RAISED
 *
 * @see {@link State}
 */
public class IntakePivot extends ServoSubsystem {
  // * SETPOINTS
  public static final Setpoint kStowedSetpoint =
      Setpoint.createProfiledPositionSetpoint(IntakeConstants.kPivotAngleMin);
  public static final Setpoint kDeployedSetpoint =
      Setpoint.createProfiledPositionSetpoint(IntakeConstants.kPivotAngleMax);
  public static final Setpoint kRaisedSetpoint =
      Setpoint.createProfiledPositionSetpoint(IntakeConstants.kPivotAngleMax.minus(Degrees.of(5)));

  // * LOGIC
  @Logged private final EncoderIO mEncoder;

  private State mState = State.STOWED;

  /**
   * Create new {@link IntakePivot}
   *
   * @param servo {@link ServoIO} representing servo driving subsystem
   * @param encoder {@link EncoderIO} representing the absolute encoder used for closed-loop control
   * @param epsilon {@link Angle} representing largest allowed error for angular position control
   */
  public IntakePivot(ServoIO servo, EncoderIO encoder, Angle epsilon) {
    super(servo, epsilon);

    mEncoder = encoder;
  }

  @Override
  public void periodic() {
    super.periodic();

    if (isNearSetpoint()) {
      if (mState == State.DEPLOYING) mState = State.DEPLOYED;
      else if (mState == State.STOWING) mState = State.STOWED;
    }
  }

  @Override
  public Angle getAngle() {
    return mEncoder.getAngle();
  }

  /**
   * @return {@link State} representing the current action the subsystem is performing
   */
  @Logged(name = "State", importance = Importance.CRITICAL)
  public State getState() {
    return mState;
  }

  /**
   * Request subsystem to collapse inwards completely to a "stowed" state
   *
   * @return {@link Command}
   */
  public Command stowCmd() {
    return setSetpointCmd(kDeployedSetpoint)
        .andThen(setStateCmd(State.STOWING))
        .withName("IntakePivot Deploy");
  }

  /**
   * Request subsystem to pivot outwards completely to a "deployed" state
   *
   * @return {@link Command}
   */
  public Command deployCmd() {
    return setSetpointCmd(kDeployedSetpoint)
        .andThen(setStateCmd(State.DEPLOYING))
        .withName("IntakePivot Deploy");
  }

  /**
   * Request subsystem to raise slightly from deployed position to help unjam fuel
   *
   * @return {@link Command}
   */
  public Command riseCmd() {
    return setSetpointCmd(kRaisedSetpoint)
        .andThen(setStateCmd(State.RAISED))
        .withName("IntakePivot Raised");
  }

  /**
   * Set current state value
   *
   * @param state {@link State}
   * @return {@link Command}
   */
  private Command setStateCmd(State state) {
    return this.runOnce(() -> mState = state);
  }

  /** Represents an action an {@link IntakePivot} subsystem can do */
  public static enum State {
    STOWING,
    STOWED,
    DEPLOYING,
    DEPLOYED,
    RAISED
  }
}
