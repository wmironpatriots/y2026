// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the hood subsystem
 *
 * <p>The {@link Hood} has a single pivoting component that folds and unfolds the hood to launch
 * fuel at different angles
 *
 * <p>The {@link Hood} moving to a setpoint angle is referred as "adjusting"
 */
public class Hood extends SubsystemBase {
  /** Represents a mode of being the {@link Hood} subsystem can be in */
  public static enum State {
    /** {@link State} where the {@link Hood} is completely folded */
    STOWED,
    /** {@link State} where the {@link Hood} is unfolding to a specified setpoint */
    ADJUSTING,
    /** {@link State} where the {@link Hood} is unfolded at a specified setpoint */
    ANGLED,
    /** {@link State} where the {@link Hood} is folding into a STOWED state */
    STOWING
  }

  public static class HoodConstants {
    public static final double kRotorToSensor = 26 / 14;

    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    public static final int kServoCanDeviceId = Matrix.kHoodId;

    public static final int kEncoderCanDeviceId = Matrix.kHoodEncoderId;

    public static final boolean frc6423RobotWin = true;

    public static final double kSensorToMech = 1 / 1;

    public static final double kMinAngle = 15;

    public static final double kMaxAngle = 75;

    public static final int kStatorCurrent = 80;

    public static final int kSupplyCurrent = 40;

    public static final Angle kEpsilon = Degrees.of(3);

    public static final Angle kEncoderAngularOffset = Rotations.of(0);

    public static final TalonFXConfiguration kServoTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKP(0)
                    .withKD(0)
                    .withGravityType(GravityTypeValue.Arm_Cosine));

    public static final CANcoderConfiguration kEncoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(HoodConstants.kEncoderAngularOffset));
  }

  private final ServoIO mServo;
  private final EncoderIO mEncoder;
  private State mState = State.STOWED;

  private Angle mSetpointAngle = Revolutions.zero();
  private final Angle mEpsilon;

  /**
   * Create new {@link Hood}
   *
   * @param servo {@link ServoIO} representing the servo pivoting hood
   * @param epsilon {@link Angle} representing the largest acceptable amount of angular position
   *     error
   */
  public static Hood create() {
    return new Hood(
        new ServoIOTalonFx(
            "HoodServo",
            HoodConstants.kCanBus,
            HoodConstants.kServoCanDeviceId,
            HoodConstants.kServoTalonConfig),
        new EncoderIOCanCoder(
            HoodConstants.kEncoderCanDeviceId, HoodConstants.kCanBus, HoodConstants.kEncoderConfig),
        HoodConstants.kEpsilon);
  }

  public Hood(ServoIO servo, EncoderIO encoder, Angle epsilon) {
    mServo = servo;
    mEpsilon = epsilon;
    mEncoder = encoder;
  }

  @Override
  public void periodic() {
    mServo.periodic();

    switch (mState) {
      case ADJUSTING:
        break;
      case ANGLED:
        break;
      case STOWED:
        break;
      case STOWING:
        stow();
        break;
    }
  }

  /**
   * @return {@link State} representing the current mode of being subsystem is in
   */
  @Logged(name = "State", importance = Importance.INFO)
  public State getState() {
    return mState;
  }

  /**
   * @return {@link Angle} representing the angular position of hood
   */
  @Logged(name = "Angle", importance = Importance.INFO)
  public Angle getAngle() {
    return mEncoder.getAngle();
  }

  /**
   * @return {@link Angle} representing the setpoint angular position
   */
  @Logged(name = "Setpoint", importance = Importance.INFO)
  public Angle getSetpointAngle() {
    return mSetpointAngle;
  }

  /**
   * @return true when the angular position error of hood is less than epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return MathUtil.isNear(
        mSetpointAngle.in(Rotations),
        getAngle().in(Rotations),
        HoodConstants.kEpsilon.in(Rotations));
  }

  /**
   * Attempt to stow hood
   *
   * @return {@link Command}
   */
  public Command stow() {
    return this.run(
            () -> {
              setAngleSetpoint(Angle.ofBaseUnits(HoodConstants.kMinAngle, Units.Degrees));
            })
        .until(this::isNearSetpoint)
        .andThen(
            () -> {
              mState = State.STOWED;
            });
  }

  /**
   * Set Angular Position Setpoint for Pivot
   *
   * @param setpoint {@link Angle} representing desired angle
   */
  private void setAngleSetpoint(Angle setpoint) {
    mSetpointAngle =
        Rotations.of(
            MathUtil.clamp(
                setpoint.in(Rotations),
                Angle.ofBaseUnits(HoodConstants.kMinAngle, Units.Degrees).in(Rotations),
                Angle.ofBaseUnits(HoodConstants.kMaxAngle, Units.Degrees).in(Rotations)));

    mServo.setTorqueMotionProfiledPositionSetpoint(mSetpointAngle, 0);
  }

  /**
   * Attempt to adjust hood to specified angle
   *
   * @param angle {@link Angle} representing setpoint angle to adjust to
   * @return {@link Command}
   */
  public Command adjustToAngle(Angle angle) {
    return this.run(
        () -> {
          setAngleSetpoint(angle);
        });
  }

  /**
   * Attempt to adjust hood to specified angle continuously
   *
   * @param angle {@link Supplier} of {@link Angle} representing setpoint angle to adjust to
   * @return {@link Command}
   */
  public Command adjustToAngle(Supplier<Angle> angle) {
    return Commands.none();
  }
}
