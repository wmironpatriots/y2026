// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxSim;
import org.frc6423.lib.sim.FlywheelSim;
import org.frc6423.lib.sim.PivotMechSim;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

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
    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    /** {@link Integer} representing the Pivot's CAN ID on CANBUS */
    public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

    /** {@link Integer} representing the Encoder ID on CANBUS */
    public static final int kEncoderCanDeviceId = Matrix.kIntakeEncoderId;

    /** {@link Integer} representing the Roller ID on CANBUS */
    public static final int kRollerCanDeviceId = Matrix.kIntakeRollerId;

    /** {@link Double} representing the gear ratio between the motor rotor to encoder */
    public static final double kPivotRotorToSensor = 5.0 * 3.0 * 1.0;

    /** {@link Double} representing the gear ratio between the sensor to mechanism */
    public static final double kPivotSensorToMechRatio = (36.0 / 16.0);

    /** {@link MomentOfInertia} representing the rotational inertia of intake */
    public static final MomentOfInertia kIntakeArmRotationalInertia =
        KilogramSquareMeters.of(0.0004); // TODO tune

    /** {@link Distance} representing the arm length of intake */
    public static final Distance kIntakeArmLength = Inches.of(1); // TODO tune

    /** {@link AngularVelocity} representing pivot motion magic cruise velocity */
    public static final AngularVelocity kPivotCruiseVelocity = RotationsPerSecond.of(0.25);

    /** {@link AngularVelocity} representing pivot motion magic acceleration */
    public static final AngularAcceleration kPivotAcceleration = RotationsPerSecondPerSecond.of(3);

    /** {@link Angle} representing the magnetic angular offset of encoder */
    public static final Angle kEncoderAngularOffset = Rotations.of(-0.046); // TODO tune

    /** {@link TalonFXConfiguration} representing the hardware config of the pivot servo */
    public static final TalonFXConfiguration kPivotTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
                    .withRotorToSensorRatio(kPivotRotorToSensor)
                    .withSensorToMechanismRatio(kPivotSensorToMechRatio))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kPivotCruiseVelocity)
                    .withMotionMagicAcceleration(kPivotAcceleration))
            .withSlot0(
                new Slot0Configs() // TODO tune
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKP(320.0)
                    .withKD(20.0)
                    .withGravityArmPositionOffset(Rotations.of(0.25)) // TODO remove
                    .withGravityType(
                        GravityTypeValue
                            .Arm_Cosine)); // Torque Based Motion Magic Position Controls

    /** {@link CANcoderConfiguration} representing the hardware config of the encoder servo */
    public static final CANcoderConfiguration kEncoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(kEncoderAngularOffset));

    /** {@link TalonFXConfiguration} representing the hardware config of the roller servo */
    public static final TalonFXConfiguration kRollerTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(20.0))
                    .withStatorCurrentLimitEnable(true));

    /** {@link Angle} representing lower angular position limit */
    public static final Angle kMinAngle = Radians.of(0.0);

    /** {@link Angle} representing stowed angular position */
    public static final Angle kStowedAngle = Radians.of(0.3);

    /** {@link Voltage} representing stowed speed */
    public static final Voltage kStowedSpeed = Volts.zero();

    /** {@link Angle} representing deployed angular position */
    public static final Angle kDeployedAngle = Radians.of(2.20);

    /** {@link Voltage} representing the intaking roller speed */
    public static final Voltage kIntakingSpeed = Volts.of(3.0);

    /** {@link Voltage} representing the outaking roller speed */
    public static final Voltage kOutakingSpeed = Volts.of(-4.0);

    /** {@link Angle} representing higher angular position limit */
    public static final Angle kMaxAngle = Radians.of(2.38);

    /** {@link Angle} representing the maximum allowed error for pivot servo */
    public static final Angle kEpsilon = Degrees.of(3);
  }

  /**
   * Create new {@link Intake}
   *
   * @return {@link Intake}
   */
  public static Intake create() {
    if (Robot.isReal()) {
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
    } else {
      return new Intake(
          new ServoIOTalonFxSim(
              "IntakePivot",
              Constants.kCanBus,
              Constants.kPivotCanDeviceId,
              Constants.kPivotTalonConfig,
              new PivotMechSim(
                  new PivotMechSim.Config(
                      DCMotor.getKrakenX60Foc(1),
                      Constants.kPivotSensorToMechRatio,
                      Constants.kIntakeArmLength,
                      Constants.kIntakeArmRotationalInertia,
                      true,
                      Constants.kMinAngle,
                      Constants.kMaxAngle,
                      Constants.kMinAngle))),
          new EncoderIOCanCoder(0, Constants.kCanBus, Constants.kEncoderConfig),
          new ServoIOTalonFxSim(
              "IntakeRoller",
              Constants.kCanBus,
              Constants.kRollerCanDeviceId,
              Constants.kRollerTalonConfig,
              new FlywheelSim(
                  new FlywheelSim.Config(
                      DCMotor.getKrakenX60Foc(1), 1.0, KilogramSquareMeters.of(1)))));
    }
  }

  @Logged private final EncoderIO mEncoder;
  @Logged private final ServoIO mPivot, mRoller;

  private Angle mSetpoint = Degrees.of(0);

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
  }

  /**
   * @return {@link Angle} representing the desired angular position of pivot
   */
  @Logged(name = "Pivot Setpoint Angle", importance = Importance.INFO)
  public Angle getSetpointAngle() {
    return mSetpoint;
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
   * Attempt to stow intake
   *
   * @return {@link Command}
   */
  public Command stow() {
    return this.run(
            () -> {
              setAngleSetpoint(Constants.kMinAngle);
              setSpeedSetpoint(Constants.kIntakingSpeed);
            })
        .until(this::isNearSetpoint)
        .andThen(() -> setSpeedSetpoint(Volts.zero()));
  }

  /**
   * Attempt to deploy and start intaking
   *
   * @return {@link Command}
   */
  public Command intake() {
    return this.run(
        () -> {
          setAngleSetpoint(Constants.kMaxAngle);
          setSpeedSetpoint(Constants.kIntakingSpeed);
        });
  }

  /**
   * Attempt to deploy and start outaking
   *
   * @return
   */
  public Command outake() {
    return this.run(
        () -> {
          setAngleSetpoint(Constants.kMaxAngle);
          setSpeedSetpoint(Constants.kOutakingSpeed);
        });
  }

  /**
   * Set Angular Position Setpoint for Pivot
   *
   * @param setpoint {@link Angle} representing desired angle
   */
  private void setAngleSetpoint(Angle setpoint) {
    mSetpoint =
        Rotations.of(
            MathUtil.clamp(
                setpoint.in(Rotations),
                Constants.kMinAngle.in(Rotations),
                Constants.kMaxAngle.in(Rotations)));

    mPivot.setTorqueMotionProfiledPositionSetpoint(mSetpoint, 0);
  }

  /**
   * Set Voltage Speed Setpoint for Roller
   *
   * @param speed {@link Voltage} representing desired speed
   */
  private void setSpeedSetpoint(Voltage speed) {
    mRoller.setVoltageSetpoint(speed, true);
  }
}
