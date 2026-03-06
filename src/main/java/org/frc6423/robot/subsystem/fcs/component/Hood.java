// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs.component;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import java.util.function.Supplier;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

public class Hood {
  // * ~~~~~~~~ STATIC CONSTANT MEMBERS ~~~~~~~~

  // Physical Constants

  /** {@link Distance} 'Length' of Pivot System */
  public static final Distance kLength = Inches.of(7.510000);

  /** {@link MomentOfInertia} Rotational Inertia of Pivot System */
  public static final MomentOfInertia kRotationalInertia =
      KilogramSquareMeters.of(75.752248 * 0.0002926397);

  // Control Constants

  /** {@link Angle} Max allowed angular position error in subsystem */
  public static final Angle kEpsilon = Degrees.of(0.5);

  public static final double kEpsilonDebounceTime = 0.1;

  /** {@link Angle} Lower limit on angular position */
  public static final Angle kMinAngle = Degrees.of(14.703759);

  /** {@link Angle} upper limit on angular position */
  public static final Angle kMaxAngle = Degrees.of(45.812);

  /** {@link AngularVelocity} The maximum allowed angular velocity of subsystem */
  public static final AngularVelocity kMaxVelocity = RevolutionsPerSecond.of(1);

  /** {@link AngularAcceleration} The maximum allowed angular acceleration of subsystem */
  public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(15.0);

  // Hardware Constants

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} CAN ID of servo */
  public static final int kServoCanDeviceId = Matrix.kHoodId;

  /** {@link Current} Stator current limit of servo */
  public static final Current kServoStatorCurrentLimit = Amps.of(40.0);

  /** {@link Double} Gear ratio between the servo rotor and the encoder shaft */
  public static final double kRotorToSensorRatio = 2.57142857143;

  /** {@link Double} Gear ratio between the encoder shaft and the mechanism pivot */
  public static final double kSensorToMechRatio = 10.83;

  /** {@link Integer} CAN ID of abs encoder */
  public static final int kEncoderCanDeviceId = Matrix.kHoodEncoderId;

  /** {@link Angle} Angular Offset to the stowed angle of abs encoder */
  public static final Angle kEncoderAngularOffset =
      Revolutions.of(-0.354736328125).plus(kMinAngle.times(kSensorToMechRatio)); // TODO

  /** {@link Angle} Angular Position in the middle of the 'unreachable' area of pivot */
  public static final Angle kEncoderSensorDiscontinuityPoint =
      Degrees.of(360).minus(kMaxAngle.minus(kMinAngle)).div(2).plus(kMaxAngle);

  /** {@link CANcoderConfiguration} Hardware config of abs encoder */
  public static final CANcoderConfiguration kEncoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(kEncoderAngularOffset)
                  .withAbsoluteSensorDiscontinuityPoint(kEncoderSensorDiscontinuityPoint));

  /** {@link TalonFXConfiguration} Hardware config of servo */
  public static final TalonFXConfiguration kServoTalonConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(kServoStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
                  .withRotorToSensorRatio(kRotorToSensorRatio)
                  .withSensorToMechanismRatio(kSensorToMechRatio))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(1)
                  .withMotionMagicAcceleration(2))
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKP(250.0)
                  .withKD(10.0)); // TODO Torque Current Control Gains (accelerating)

  // Characterization Constants

  /**
   * {@link Velocity} of {@link CurrentUnit} Rate at which current output ramps up at in Quasistatic
   * Characterization
   */
  public static final Velocity<CurrentUnit> kCharacterizationRampRate = Amps.of(15.0).per(Second);

  /** {@link Current} Current step size for Dynamic Characterization */
  public static final Current kCharacterizationStepSize = Amps.of(6.0);

  /**
   * Create new {@link Hood}
   *
   * @return {@link Hood}
   */
  public static Hood create() {
    return (Robot.isReal())
        ? new Hood(
            new ServoIOTalonFx("Servo", kCanBus, kServoCanDeviceId, kServoTalonConfig),
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig))
        : new Hood(
            new ServoIOTalonFxPivotSim(
                "Servo",
                kCanBus,
                kServoCanDeviceId,
                kServoTalonConfig,
                kRotationalInertia,
                kLength,
                kMinAngle,
                kMaxAngle,
                kMinAngle,
                true,
                MotorType.KrakenX44,
                DCMotor.getKrakenX44Foc(1),
                kSensorToMechRatio * kRotorToSensorRatio),
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig));
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final ServoIO mServo;
  @Logged private final EncoderIO mEncoder;

  private Rotation2d mTargetRotation2d = Rotation2d.kZero;

  private final Debouncer mIsAtSetpoint = new Debouncer(kEpsilonDebounceTime);

  /**
   * Create new {@link Hood}
   *
   * @param servo {@link ServoIO} Servo powering subsystem
   * @param encoder {@link EncoderIO} Encoder measuring angular motion
   */
  protected Hood(ServoIO servo, EncoderIO encoder) {
    // Init Hardware
    mServo = servo;
    mEncoder = encoder;
  }

  public void periodic() {
    mServo.periodic();
    mEncoder.periodic();
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  @Logged(name = "Holding Setpoint (bool)", importance = Importance.INFO)
  public boolean isHoldingSetpoint() {
    return mIsAtSetpoint.calculate(
        MathUtil.isNear(
            getTargetRotation2d().getRadians(),
            getRotation2d().getRadians(),
            kEpsilon.in(Radians)));
  }

  @Logged(name = "Rotation2d (rads)", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return new Rotation2d(mServo.getAngle());
  }

  @Logged(name = "Target Rotation2d (rads)", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return mTargetRotation2d;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  public void stow() {
    setTargetRotation2d(new Rotation2d(kMinAngle));
  }

  public void setTargetRotation2d(Rotation2d angle) {
    runTargetRotation2d(() -> angle);
  }

  public void runTargetRotation2d(Supplier<Rotation2d> angle) {
    mTargetRotation2d =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                getRotation2d().getRadians(), kMinAngle.in(Radians), kMaxAngle.in(Radians)));

    mServo.setTorqueMotionProfiledPositionSetpoint(kMaxAngle);
  }
}
