// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxFlywheelSim;
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.lib.util.NetworkTableUtil;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

public class Shooter extends SubsystemBase {
  /**
   * Create new {@link Shooter}
   *
   * @return {@link Shooter}
   */
  public static Shooter create() {
    return (Robot.isReal())
        ? new Shooter(
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig),
            new ServoIOTalonFx(
                "Pivot", MotorType.KrakenX44, kCanBus, kPivotCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Left", MotorType.KrakenX60, kCanBus, kFlywheelLeftCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Right", MotorType.KrakenX60, kCanBus, kFlywheelRightCanDeviceId, kPivotConfig))
        : new Shooter(
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig),
            new ServoIOTalonFxPivotSim(
                "Pivot",
                MotorType.KrakenX44,
                kCanBus,
                kPivotCanDeviceId,
                kPivotConfig,
                kPivotRotationalInertiaKgSquaredMeters,
                kPivotArmLengthMeters,
                kMinAngleRevs,
                kMaxAngleRevs,
                kMinAngleRevs,
                true,
                DCMotor.getKrakenX60Foc(1),
                kPivotSensorToMechRatio * kPivotRotorToSensorRatio),
            new ServoIOTalonFxFlywheelSim(
                "Left",
                MotorType.KrakenX60,
                kCanBus,
                kFlywheelLeftCanDeviceId,
                kPivotConfig,
                kFlywheelRotationalInertiaKgSquaredMeters,
                DCMotor.getKrakenX60Foc(1),
                kFlywheelRotorToMechRatio),
            new ServoIONone("Right"));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device indentifier for the abs encoder */
  public static final int kEncoderCanDeviceId = Matrix.kIntakeEncoderId;

  /** {@link Integer} Unique CAN device indentifier for the pivot servo */
  public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

  public static final int kFlywheelLeftCanDeviceId = Matrix.kFlywheelLeftId;

  public static final int kFlywheelRightCanDeviceId = Matrix.kFlywheelRightId;

  /** {@link Double} Lower limit on pivot angular position */
  public static final double kMinAngleRevs = Units.degreesToRotations(14.703759);

  /** {@link Double} Upper limit on pivot angular position */
  public static final double kMaxAngleRevs = Units.degreesToRotations(45.812);

  /** {@link Double} Gear ratio between pivot servo rotor and the abs encoder shaft */
  public static final double kPivotRotorToSensorRatio = 2.57142857143;

  /** {@link Double} Gear ratio between pivot servo rotor and the abs encoder shaft */
  public static final double kPivotSensorToMechRatio = 10.83;

  /** {@link Double} Gear ratio between flywheel servos to mechanism */
  public static final double kFlywheelRotorToMechRatio = (28.0 / 16.0);

  /** {@link Double} Angular Position offset to the stowed angle of abs encoder */
  public static final double kEncoderAngularOffsetRevs =
      0.0 + (kMinAngleRevs * kPivotSensorToMechRatio); // TODO

  /** {@link Double} Angular Position in the middle of the 'unreachable' area of pivot */
  public static final double kEncoderSensorDiscontinuityPointRevs =
      360
          - (kMaxAngleRevs - kMinAngleRevs) / 2
          + kMaxAngleRevs; // TODO we might have to add to encoder offset to zero

  /** {@link CANcoderConfiguration} Hardware config of abs encoder */
  public static final CANcoderConfiguration kEncoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(kEncoderAngularOffsetRevs)
                  .withAbsoluteSensorDiscontinuityPoint(kEncoderSensorDiscontinuityPointRevs));

  /** {@link TalonFXConfiguration} Hardware config of pivot servo */
  public static final TalonFXConfiguration kPivotConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(40.0)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
                  .withRotorToSensorRatio(kPivotRotorToSensorRatio)
                  .withSensorToMechanismRatio(kPivotSensorToMechRatio))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(1)
                  .withMotionMagicAcceleration(2))
          .withSlot0(new Slot0Configs().withKP(0.0).withKD(0.0)); // Torque Current Control Gains

  /** {@link TalonFXConfiguration} Hardware config of flywheel servos */
  public static final TalonFXConfiguration kServoTalonConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(120.0)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(120.0)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(kFlywheelRotorToMechRatio))
          .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(9999.0))
          .withSlot0(
              new Slot0Configs().withKS(0.0).withKV(0.0).withKA(0.0).withKP(10.0).withKD(0.1));

  /** {@link Double} Moment of Inertia of pivot system */
  public static final double kPivotRotationalInertiaKgSquaredMeters = 402.290096 * 0.0002926397;

  /** {@link Double} Length of pivot arm in meters */
  public static final double kPivotArmLengthMeters = 0.5;

  public static final double kFlywheelRotationalInertiaKgSquaredMeters = 10.491008 * 0.0002926397;

  public static final double kFlywheelRadiusMeters = Units.inchesToMeters(2);

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  private double mPivotEpsilonRevs = Units.degreesToRadians(0.1);
  private double mFlywheelEpsilonRevs = Units.degreesToRadians(1);

  private final DoubleEntry mPivotEpsilonTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Pivot Epsilon (revs)", mPivotEpsilonRevs);
  private final DoubleEntry mFlywheelEpsilonTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Shooter/Flywheel Epsilon (revs)", mFlywheelEpsilonRevs);

  private double mPivotKp = 0.0;
  private double mPivotKd = 0.0;

  private final DoubleEntry mPivotKpTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Pivot Kp", mPivotKp);
  private final DoubleEntry mPivotKdTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Pivot Kd", mPivotKd);

  private double mPivotMaxAngularVelocityRevsPerSec = 2;
  private double mPivotMaxAngularAccelerationRevsPerSecPerSec = 3;

  private final DoubleEntry mPivotMaxAngularVelocityTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Shooter/Pivot Max Velocity (revs per second)",
          mPivotMaxAngularVelocityRevsPerSec);
  private final DoubleEntry mPivotMaxAngularAccelerationTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Shooter/Pivot Max Acceleration (revs per second per second)",
          mPivotMaxAngularAccelerationRevsPerSecPerSec);

  private double mFlywheelKs = 0.0;
  private double mFlywheelKv = 0.0;
  private double mFlywheelKa = 0.0;
  private double mFlywheelKp = 0.0;
  private double mFlywheelKd = 0.0;

  private final DoubleEntry mFlywheelKsTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Flywheel Ks", mFlywheelKs);
  private final DoubleEntry mFlywheelKvTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Flywheel Kv", mFlywheelKv);
  private final DoubleEntry mFlywheelKaTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Flywheel Ka", mFlywheelKa);
  private final DoubleEntry mFlywheelKpTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Flywheel Kp", mFlywheelKp);
  private final DoubleEntry mFlywheelKdTunable =
      NetworkTableUtil.createEntry("Tunables/Shooter/Flywheel Kd", mFlywheelKd);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final EncoderIO mEncoder;
  @Logged private final ServoIO mPivot, mLeft, mRight;

  private double mTargetAngleRevs;
  private double mTargetFlywheelMuzzleVelocityMetersPerSec;

  private final Debouncer mHoodHoldingSetpoint = new Debouncer(0.1);
  private final Debouncer mFlywheelHoldingSetpoint = new Debouncer(0.03);

  protected Shooter(EncoderIO encoder, ServoIO pivot, ServoIO left, ServoIO right) {
    mEncoder = encoder;
    mPivot = pivot;
    mLeft = left;
    mRight = right;

    mRight.setLeader(mLeft, true);

    // TODO
    setDefaultCommand(null);
  }

  @Override
  public void periodic() {
    mEncoder.periodic();
    mPivot.periodic();
    mLeft.periodic();
    mRight.periodic();

    mHoodHoldingSetpoint.calculate(
        MathUtil.isNear(mTargetAngleRevs, getRotation2d().getRotations(), mPivotEpsilonRevs));
    mFlywheelHoldingSetpoint.calculate(
        MathUtil.isNear(
            getTargetAngularVelocityRevsPerSecond(),
            getAngularVelocityRevsPerSec(),
            mFlywheelEpsilonRevs));

    if (Flags.kTuningModeEnabled) {
      mPivotEpsilonRevs = mPivotEpsilonTunable.get();
      mFlywheelEpsilonRevs = mFlywheelEpsilonTunable.get();

      if (mPivotKpTunable.get() != mPivotKp || mPivotKdTunable.get() != mPivotKd) {
        mPivotKp = mPivotKpTunable.get();
        mPivotKd = mPivotKdTunable.get();
        mPivot.setFeedbackGains(mPivotKp, mPivotKd);
      }

      if (mPivotMaxAngularVelocityTunable.get() != mPivotMaxAngularVelocityRevsPerSec
          || mPivotMaxAngularAccelerationTunable.get()
              != mPivotMaxAngularAccelerationRevsPerSecPerSec) {
        mPivotMaxAngularVelocityRevsPerSec = mPivotMaxAngularVelocityTunable.get();
        mPivotMaxAngularAccelerationRevsPerSecPerSec = mPivotMaxAngularAccelerationTunable.get();

        mPivot.setProfilingConstraints(
            mPivotMaxAngularVelocityRevsPerSec, mPivotMaxAngularAccelerationRevsPerSecPerSec);
      }

      if (mFlywheelKsTunable.get() != mFlywheelKs
          || mFlywheelKvTunable.get() != mFlywheelKv
          || mFlywheelKaTunable.get() != mFlywheelKa
          || mFlywheelKpTunable.get() != mFlywheelKp
          || mFlywheelKdTunable.get() != mFlywheelKd) {
        mFlywheelKs = mFlywheelKsTunable.get();
        mFlywheelKv = mFlywheelKvTunable.get();
        mFlywheelKa = mFlywheelKaTunable.get();
        mFlywheelKp = mFlywheelKpTunable.get();
        mFlywheelKd = mFlywheelKdTunable.get();

        mLeft.setFeedforwardGains(mFlywheelKs, 0.0, mFlywheelKv, mFlywheelKa);
        mLeft.setFeedbackGains(mFlywheelKp, mFlywheelKd);
        mRight.setFeedforwardGains(mFlywheelKs, 0.0, mFlywheelKv, mFlywheelKa);
        mRight.setFeedbackGains(mFlywheelKp, mFlywheelKd);
      }
    }
  }

  @Logged(name = "Target Rotation2d (rads)", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return Rotation2d.fromRotations(mTargetAngleRevs);
  }

  @Logged(name = "Rotation2d (rads)", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(mPivot.getAngularPositionRevs());
  }

  @Logged(name = "Target Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getTargetMuzzleVelocityMetersPerSecond() {
    return mTargetFlywheelMuzzleVelocityMetersPerSec;
  }

  @Logged(name = "Target Angular Velocity (revs per second)", importance = Importance.INFO)
  public double getTargetAngularVelocityRevsPerSecond() {
    // omega = (v / 2) * radius
    return getTargetMuzzleVelocityMetersPerSecond() * 0.5 * kFlywheelRadiusMeters;
  }

  @Logged(name = "Approximated Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getApproximatedMuzzleVelocityMetersPerSecond() {
    // v = (omega / radius) * 2
    return getAngularVelocityRevsPerSec() / kFlywheelRadiusMeters * 2;
  }

  @Logged(name = "Angular Velocity (revs per second)", importance = Importance.INFO)
  public double getAngularVelocityRevsPerSec() {
    return mLeft.getAngularVelocityRevsPerSec();
  }

  public Command runSetpoint(double pivotAngleRevs, double muzzleVelocityMetersPerSec) {
    return this.run(
        () -> {
          mTargetAngleRevs = MathUtil.clamp(pivotAngleRevs, kMinAngleRevs, kMaxAngleRevs);

          mPivot.setProfiledPositionSetpoint(mTargetAngleRevs);
          // omega = (v / 2) * radius
          mLeft.setProfiledVelocitySetpoint(
              muzzleVelocityMetersPerSec * 0.5 * kFlywheelRadiusMeters);
        });
  }

  public Command stowAndCoast() {
    return this.run(
        () -> {
          mPivot.setProfiledPositionSetpoint(kMinAngleRevs);
          mLeft.setNeutral();
        });
  }
}
