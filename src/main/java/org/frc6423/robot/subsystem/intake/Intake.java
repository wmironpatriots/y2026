// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

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
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.lib.util.NetworkTableUtil;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

// TODO beambreak
/**
 * {@link SubsystemBase} Controller for the intaking subsystem
 *
 * <p>{@link Intake} is a fourbar ball intake pivoted by a single Kraken x60 and run by a single
 * Kraken x60
 *
 * <p>The purpose of {@link Intake} is to take ownership of fuel on the field
 *
 * <p>{@link Intake} has two actions: neutral and intake
 */
public class Intake extends SubsystemBase {
  /**
   * Create new {@link Intake}
   *
   * @return {@link Intake}
   */
  public static Intake create() {
    return (Robot.isReal())
        ? new Intake(
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig),
            new ServoIOTalonFx(
                "Pivot", MotorType.KrakenX60, kCanBus, kPivotCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Roller", MotorType.KrakenX60, kCanBus, kRollerCanDeviceId, kRollerConfig))
        : new Intake(
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig),
            new ServoIOTalonFxPivotSim(
                "Pivot",
                MotorType.KrakenX60,
                kCanBus,
                kPivotCanDeviceId,
                kPivotConfig,
                kRotationalInertiaKgSquaredMeters,
                kArmLengthMeters,
                kMinAngleRevs,
                kMaxAngleRevs,
                kMinAngleRevs,
                true,
                DCMotor.getKrakenX60Foc(1),
                kPivotRotorToSensorRatio * kPivotSensorToMechRatio),
            new ServoIONone("Roller"));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device indentifier for the abs encoder */
  public static final int kEncoderCanDeviceId = Matrix.kIntakeEncoderId;

  /** {@link Integer} Unique CAN device indentifier for the pivot servo */
  public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

  /** {@link Integer} Unique CAN device indentifier for the roller servo */
  public static final int kRollerCanDeviceId = Matrix.kIntakeRollerId;

  /** {@link Double} Max allowed angular position error in subsystem */
  public static final double kEpsilonRevs = Units.degreesToRadians(0.3);

  /** {@link Double} Lower limit on pivot angular position */
  public static final double kMinAngleRevs = Units.radiansToRotations(1.15);

  /** {@link Double} Upper limit on pivot angular position */
  public static final double kMaxAngleRevs = Units.radiansToDegrees(2.16);

  /** {@link Double} Gear ratio between pivot servo rotor and the abs encoder shaft */
  public static final double kPivotRotorToSensorRatio = (5.0 / 1.0) * (3.0 / 1.0) * (1.0 / 1.0);

  /** {@link Double} Gear ratio between the abs encoder shaft and the mechanism pivot */
  public static final double kPivotSensorToMechRatio = (36.0 / 16.0);

  /** {@link Double} Angular Position offset to the stowed angle of abs encoder */
  public static final double kEncoderAngularOffsetRevs = 0.6555; // TODO

  /** {@link Double} Angular Position in the middle of the 'unreachable' area of pivot */
  public static final double kEncoderSensorDiscontinuityPointRevs = 0.0;

  //   (kMinAngleRevs + kMaxAngleRevs) / 2.0 + 0.5;

  //   MathUtil.inputModulus(kMaxAngleRevs + kMinAngleRevs, 0.0, 1.0) / 2;
  // 360
  // - (kMaxAngleRevs - kMinAngleRevs) / 2
  // + kMaxAngleRevs; // TODO we might have to add to encoder offset to zero

  /** {@link CANcoderConfiguration} Hardware config of abs encoder */
  public static final CANcoderConfiguration kEncoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
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
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(kPivotSensorToMechRatio * kPivotRotorToSensorRatio))
          //   .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
          //   .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
          //   .withRotorToSensorRatio(kPivotRotorToSensorRatio)
          //   .withSensorToMechanismRatio(kPivotSensorToMechRatio))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(2)
                  .withMotionMagicAcceleration(3))
          .withSlot0(
              new Slot0Configs()
                  .withKS(1)
                  .withKP(350.0)
                  .withKD(50.0)); // Torque Current Control Gains (accelerating)

  /** {@link TalonFXConfiguration} Hardware config of roller servo */
  public static final TalonFXConfiguration kRollerConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(20.0)
                  .withStatorCurrentLimitEnable(true));

  /** {@link Double} Moment of Inertia of pivot system */
  public static final double kRotationalInertiaKgSquaredMeters = 402.290096 * 0.0002926397;

  /** {@link Double} Length of pivot arm in meters */
  public static final double kArmLengthMeters = 0.5;

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  private double mPivotKs = 1.0;
  private double mPivotKp = 350.0;
  private double mPivotKd = 50.0;

  private final DoubleEntry mPivotKsTunable =
      NetworkTableUtil.createEntry("Tunables/Intake/Ks", mPivotKs);
  private final DoubleEntry mPivotKpTunable =
      NetworkTableUtil.createEntry("Tunables/Intake/Kp", mPivotKp);
  private final DoubleEntry mPivotKdTunable =
      NetworkTableUtil.createEntry("Tunables/Intake/Kd", mPivotKd);

  private double mPivotMaxAngularVelocityRevsPerSec = 2;
  private double mPivotMaxAngularAccelerationRevsPerSecPerSec = 3;

  private final DoubleEntry mPivotMaxAngularVelocityTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Intake/Max Velocity (revs per second)", mPivotMaxAngularVelocityRevsPerSec);
  private final DoubleEntry mPivotMaxAngularAccelerationTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Intake/Max Acceleration (revs per second per second)",
          mPivotMaxAngularAccelerationRevsPerSecPerSec);

  private double mStowedPositionRevs = kMinAngleRevs;
  private double mDeployedPositionRevs = kMaxAngleRevs;

  private final DoubleEntry mStowedPositionTunable =
      NetworkTableUtil.createEntry("Tunables/Intake/Stowed Position (revs)", mStowedPositionRevs);
  private final DoubleEntry mDeployedPositionTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Intake/Deployed Position (revs)", mDeployedPositionRevs);

  private double mStowingSpeedVolts = 3.0;
  private double mStowedSpeedVolts = 0.0;
  private double mIntakingSpeedVolts = 9.0;

  private final DoubleEntry mStowingSpeedTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Intake/Stowing Speed (revs per second)", mStowingSpeedVolts);
  private final DoubleEntry mStowedSpeedTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Intake/Stowed Speed (revs per second)", mStowedSpeedVolts);
  private final DoubleEntry mIntakingSpeedTunable =
      NetworkTableUtil.createEntry(
          "Tunables/Intake/Intaking Speed (revs per second)", mIntakingSpeedVolts);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final EncoderIO mEncoder;
  @Logged private final ServoIO mPivot, mRoller;

  private double mTargetAngleRevs = kMinAngleRevs;

  private boolean mIsRunning = false;
  private final Debouncer mIsStuck = new Debouncer(0.1);
  private final Debouncer mIsAtSetpoint = new Debouncer(0.1);

  protected Intake(EncoderIO encoder, ServoIO pivot, ServoIO roller) {
    mEncoder = encoder;
    mPivot = pivot;
    mRoller = roller;

    mPivot.resetRelativeEncoder(kMinAngleRevs);

    setDefaultCommand(stow());
  }

  @Override
  public void periodic() {
    // Update all hardware
    mEncoder.periodic();
    mPivot.periodic();
    mRoller.periodic();

    mIsStuck.calculate(
        mIsRunning && MathUtil.isNear(0.0, mRoller.getAngularVelocityRevsPerSec(), 0.1));
    mIsAtSetpoint.calculate(
        MathUtil.isNear(
            getRotation2d().getRotations(), getTargetRotation2d().getRotations(), kEpsilonRevs));

    if (Flags.kTuningModeEnabled) {
      if (mPivotKpTunable.get() != mPivotKp
          || mPivotKdTunable.get() != mPivotKd
          || mPivotKsTunable.get() != mPivotKs) {
        mPivotKp = mPivotKpTunable.get();
        mPivotKd = mPivotKdTunable.get();
        mPivotKs = mPivotKsTunable.get();

        mPivot.setFeedforwardGains(mPivotKs, 0.0, 0.0, 0.0);
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

      mStowedPositionRevs = mStowedPositionTunable.get();
      mDeployedPositionRevs = mDeployedPositionTunable.get();

      mStowingSpeedVolts = mStowingSpeedTunable.get();
      mStowedSpeedVolts = mStowedSpeedTunable.get();
      mIntakingSpeedVolts = mIntakingSpeedTunable.get();
    }
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Get target angular position of subsystem
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Target Rotation2d (rads)", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return Rotation2d.fromRotations(mTargetAngleRevs);
  }

  /**
   * Get angular position of subsystem
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Rotation2d (rads)", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(mPivot.getAngularPositionRevs());
  }

  /**
   * Check if subsystem roller is trying to move but is unable to
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Roller Is Stuck (bool)", importance = Importance.INFO)
  public boolean isStuck() {
    return mIsStuck.calculate(
        mIsRunning && MathUtil.isNear(0.0, mRoller.getAngularVelocityRevsPerSec(), 0.1));
  }

  /**
   * Check if subsystem has reached setpoint (pivot)
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Has Reached Setpoint (bool)", importance = Importance.INFO)
  public boolean hasReachedSetpoint() {
    return mIsAtSetpoint.calculate(
        MathUtil.isNear(
            getRotation2d().getRotations(), getTargetRotation2d().getRotations(), kEpsilonRevs));
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Constructs a sequence where subsystem stops completely
   *
   * @return {@link Command}
   */
  public Command neutral() {
    return this.run(
        () -> {
          mPivot.setNeutral();
          mRoller.setNeutral();
          mIsRunning = false;
        });
  }

  /**
   * Constructs a sequence where subsystem stows away
   *
   * @return {@link Command}
   */
  public Command stow() {
    return runSetpoint(mStowedPositionRevs, mStowingSpeedVolts)
        .until(() -> hasReachedSetpoint())
        .andThen(runSetpoint(mStowedPositionRevs, mStowedSpeedVolts));
  }

  /**
   * Constructs a sequence where subsystem deploys and starts intaking
   *
   * @return {@link Command}
   */
  public Command intake() {
    return runSetpoint(mDeployedPositionRevs, mIntakingSpeedVolts);
  }

  /**
   * Constructs a sequence where subsystem runs intake at specified setpoints
   *
   * @param targetAngleRevs {@link Double} Desired angle in revolutions
   * @param targetSpeedVolts {@link Double} Desired voltage speed in volts
   * @return {@link Command}
   */
  public Command runSetpoint(double targetAngleRevs, double targetSpeedVolts) {
    return this.run(
        () -> {
          mTargetAngleRevs = MathUtil.clamp(targetAngleRevs, kMinAngleRevs, kMaxAngleRevs);

          mPivot.setProfiledPositionSetpoint(mTargetAngleRevs);
          mRoller.setVoltageOutput(targetSpeedVolts, true);
        });
  }
}
