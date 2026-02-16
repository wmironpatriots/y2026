// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the intake subsystem
 *
 * <p>AThe{@link Intake} has 3 components: the rollers, the pivot holding the rollers, and the
 * passive kicker mechanism that assists with intaking
 *
 * <p>The kicker starts in a stowed position at the start of the match and must be hit outwards by
 * the pivot; This action is set to automatically happen
 */
public class Intake extends SubsystemBase {
  /** {@link Intake} subsystem constants */
  public class Constants {
    /** {@link CANbus} representing the bus devices are connected to */
    private static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    /** {@link Integer} representing the Pivot's CAN ID on CANBUS */
    private static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

    /** {@link Integer} representing the Encoder ID on CANBUS */
    private static final int kEncoderCanDeviceId = Matrix.kIntakeEncoderId;

    /** {@link Integer} representing the Roller ID on CANBUS */
    private static final int kRollerCanDeviceId = Matrix.kIntakeRollerId;

    /** {@link TalonFXConfiguration} representing the hardware config of the pivot servo */
    private static final TalonFXConfiguration kPivotTalonConfig = new TalonFXConfiguration();

    /** {@link CANcoderConfiguration} representing the hardware config of the encoder servo */
    private static final CANcoderConfiguration kEncoderConfig = new CANcoderConfiguration();

    /** {@link TalonFXConfiguration} representing the hardware config of the roller servo */
    private static final TalonFXConfiguration kRollerTalonConfig = new TalonFXConfiguration();

    /** {@link Angle} representing maximum possible pivot angle */
    private static final Angle kMaxAngle = Degrees.of(148.976);

    /** {@link Angle} representing minimum possible pivot angle */
    private static final Angle kMinAngle = Degrees.of(90.676);

    /** {@link Voltage} representing the voltage required to 'kick' kicker outwards */
    private static final Voltage kKickVoltage = Volts.of(3.0);

    /** {@link Seconds} representing the time spent 'kicking' kicker outwards */
    private static final Time kKickTime = Seconds.of(0.5);

    /** {@link Voltage} representing the intaking roller speed */
    private static final Voltage kIntakingSpeed = Volts.of(5.0);

    /** {@link Angle} representing the maximum allowed error for pivot servo */
    private static final Angle kEpsilon = Degrees.of(0.4);
  }

  @Logged private final EncoderIO mEncoder;
  @Logged private final ServoIO mPivot, mRoller;

  private Request mRequest = Request.STOW;
  private State mState = State.FULLY_STOWED;

  private final Timer mStateTimer = new Timer();

  private Angle mSetpoint = Degrees.of(0);

  /**
   * Create new {@link Intake}
   *
   * @return {@link Intake}
   */
  public static Intake create() {
    // TODO sim
    return new Intake(
        new ServoIOTalonFx(
            "IntakePivot",
            Constants.kCanBus,
            Constants.kPivotCanDeviceId,
            Constants.kPivotTalonConfig),
        new EncoderIOCanCoder(
            Constants.kEncoderCanDeviceId, Constants.kCanBus, Constants.kEncoderConfig),
        new ServoIOTalonFx(
            "IntakeRoller",
            Constants.kCanBus,
            Constants.kRollerCanDeviceId,
            Constants.kRollerTalonConfig));
  }

  /**
   * Create new {@link Intake}
   *
   * @param pivot {@link ServoIO} representing pivot servo
   * @param encoder {@link EncoderIO} representing pivot abs encoder
   * @param roller {@link ServoIO} representing roller servo
   */
  protected Intake(ServoIO pivot, EncoderIO encoder, ServoIO roller) {
    mPivot = pivot;
    mEncoder = encoder;
    mRoller = roller;
  }

  @Override
  public void periodic() {
    // Update hardware
    mPivot.periodic();
    mEncoder.periodic();
    mRoller.periodic();

    // Apply request if edge exists
    switch (mRequest) {
      case DEPLOY:
        if (mState == State.STOWED) mState = State.DEPLOYING;
        break;
      case STOW:
        if (mState == State.DEPLOYING || mState == State.DEPLOYED) mState = State.STOWING;
        break;
    }

    // Run state logic
    switch (mState) {
      case FULLY_STOWED:

        // Only start kicker deploy routine when robot enables
        if (DriverStation.isEnabled()) mState = State.DEPLOYING_KICKER;

        break;

      case DEPLOYING_KICKER:
        if (!mStateTimer.isRunning()) mStateTimer.start();
        mPivot.setVoltageSetpoint(Constants.kKickVoltage, true);

        if (mStateTimer.hasElapsed(Constants.kKickTime)) {
          mStateTimer.reset();
          mState = State.STOWED;
        }

        break;

      case STOWING:
        setPivotSetpoint(Constants.kMinAngle);
        setRollerSetpoint(Constants.kIntakingSpeed);

        if (isNearSetpoint()) {
          mState = State.STOWED;
        }

        break;

      case STOWED:
        mPivot.stop();
        mRoller.stop();

        break;

      case DEPLOYED:
        mPivot.stop();
        setRollerSetpoint(Constants.kIntakingSpeed);

        break;

      case DEPLOYING:
        setPivotSetpoint(Constants.kMaxAngle);
        setRollerSetpoint(Constants.kIntakingSpeed);

        if (isNearSetpoint()) {
          mState = State.DEPLOYED;
        }

        break;
    }
  }

  /**
   * Clamp and set a pivot angle setpoint
   *
   * @param setpoint {@link Angle} representing desired angle
   */
  private void setPivotSetpoint(Angle setpoint) {
    mSetpoint =
        Rotations.of(
            MathUtil.clamp(
                setpoint.in(Rotations),
                Constants.kMinAngle.in(Rotations),
                Constants.kMaxAngle.in(Rotations)));
    mPivot.setTorqueMotionProfiledPositionSetpoint(setpoint, 0);
  }

  /**
   * Set a roller speed setpoint
   *
   * @param volts {@link Voltage} representing desired speed
   */
  private void setRollerSetpoint(Voltage volts) {
    mRoller.setVoltageSetpoint(volts, true);
  }

  /**
   * @return {@link State} representing the current mode of being subsystem is in
   */
  @Logged(name = "State", importance = Importance.INFO)
  public State getState() {
    return mState;
  }

  /**
   * @return {@link Angle} representing the angular position of pivot
   */
  @Logged(name = "Pivot Angle", importance = Importance.INFO)
  public Angle getAngle() {
    return mEncoder.getAngle();
  }

  /**
   * @return true when the angular position error of pivot is less than epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return MathUtil.isNear(
        mSetpoint.in(Rotations), getAngle().in(Rotations), Constants.kEpsilon.in(Rotations));
  }

  /**
   * Request intake to deploy
   *
   * @return {@link Command}
   */
  public Command deploy() {
    return this.runOnce(() -> mRequest = Request.DEPLOY);
  }

  /**
   * Request intake to stow
   *
   * @return {@link Command}
   */
  public Command stow() {
    return this.runOnce(() -> mRequest = Request.STOW);
  }

  /** Represents a requested mode of being for the {@link Intake} */
  public static enum Request {
    /** {@link Request} representing a request for intake to stow */
    STOW,
    /** {@link Request} representing a request for intake to deploy */
    DEPLOY
  }

  /** Represents a mode of being the {@link Intake} subsystem can be in */
  public static enum State {
    /**
     * {@link State} where the {@link Intake} is folded and stopped while the kicker hasn't been
     * deployed
     */
    FULLY_STOWED,
    /** {@link State} where the {@link Intake} is 'kicking' the kicker to passively deploy it */
    DEPLOYING_KICKER,
    /** {@link State} where the {@link Intake} is attempting to stop and fold */
    STOWING,
    /**
     * {@link State} where the {@link Intake} is folded & stopped while the kicker is passively
     * deployed
     */
    STOWED,
    /** {@link State} where the {@link Intake} is unfolding and starting */
    DEPLOYING,
    /** {@link State} where the {@link Intake} is unfolded and running */
    DEPLOYED
  }
}
