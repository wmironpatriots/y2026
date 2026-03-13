// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.lib.util.TunableNumber;
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
            new ServoIOTalonFx(
                "Pivot", MotorType.KrakenX60, kCanBus, kPivotCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Roller", MotorType.KrakenX60, kCanBus, kRollerCanDeviceId, kRollerConfig))
        : new Intake(
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
                kPivotSensorToMechRatio * kPivotSensorToMechRatio),
            new ServoIONone("Roller"));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device indentifier for the pivot servo */
  public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

  /** {@link Integer} Unique CAN device indentifier for the roller servo */
  public static final int kRollerCanDeviceId = Matrix.kIntakeRollerId;

  /** {@link Double} Max allowed angular position error in subsystem */
  public static final double kEpsilonRevs = Units.degreesToRadians(0.3);

  /** {@link Double} Lower limit on pivot angular position */
  public static final double kMinAngleRevs = Units.degreesToRotations(98);

  /** {@link Double} Upper limit on pivot angular position */
  public static final double kMaxAngleRevs = Units.degreesToRotations(145);

  /** {@link Double} Gear ratio between pivot servo rotor and the abs encoder shaft */
  public static final double kPivotSensorToMechRatio =
      (5.0 / 1.0) * (3.0 / 1.0) * (1.0 / 1.0) * (36.0 / 16.0);

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
                  .withSensorToMechanismRatio(kPivotSensorToMechRatio))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(2)
                  .withMotionMagicAcceleration(3));

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

  public static final String kTunablesPrefix = "/Intake";

  public static final double kPivotHomingVolts = -3.5;

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  public static TunableNumber kPivotZeroingVelocityThreshold =
      new TunableNumber(kTunablesPrefix + "/Zeroing Velocity Threshold (deg per sec)");

  public static TunableNumber kPivotEpsilonDeg =
      new TunableNumber(kTunablesPrefix + "/Epsilon (degrees)");

  public static TunableNumber kPivotKs = new TunableNumber(kTunablesPrefix + "/Ks");
  public static TunableNumber kPivotKg = new TunableNumber(kTunablesPrefix + "/Kg");
  public static TunableNumber kPivotKv = new TunableNumber(kTunablesPrefix + "/Kv");
  public static TunableNumber kPivotKa = new TunableNumber(kTunablesPrefix + "/Ka");
  public static TunableNumber kPivotKp = new TunableNumber(kTunablesPrefix + "/Kp");
  public static TunableNumber kPivotKd = new TunableNumber(kTunablesPrefix + "/Kd");

  public static TunableNumber kPivotMaxVelocityRevsPerSec =
      new TunableNumber(kTunablesPrefix + "/Max Velocity (revs per sec)");
  public static TunableNumber kPivotMaxAccelerationRevsPerSecPerSec =
      new TunableNumber(kTunablesPrefix + "/Max Acceleration (revs per sec per sec)");

  public static TunableNumber kPivotStowedPositionDeg =
      new TunableNumber(kTunablesPrefix + "/Stowed Position (degrees)");
  public static TunableNumber kPivotDeployedPositionDeg =
      new TunableNumber(kTunablesPrefix + "/Deployed Position (degrees)");

  public static TunableNumber kStowedSpeedVolts =
      new TunableNumber(kTunablesPrefix + "/Stowed Speed (volts)");
  public static TunableNumber kStowingSpeedVolts =
      new TunableNumber(kTunablesPrefix + "/Stowing Speed (volts)");
  public static TunableNumber kIntakingSpeedVolts =
      new TunableNumber(kTunablesPrefix + "/Intaking Speed (volts)");
  public static TunableNumber kOutakingSpeedVolts =
      new TunableNumber(kTunablesPrefix + "/Outaking Speed (volts)");

  static {
    kPivotZeroingVelocityThreshold.initDefault(0.1);
    kPivotEpsilonDeg.initDefault(1.5);

    kPivotKs.initDefault(0.0);
    kPivotKg.initDefault(0.0);
    kPivotKv.initDefault(0.0);
    kPivotKa.initDefault(0.0);
    kPivotKp.initDefault(250.0);
    kPivotKd.initDefault(30.0);

    kPivotMaxVelocityRevsPerSec.initDefault(2.0);
    kPivotMaxAccelerationRevsPerSecPerSec.initDefault(3.0);

    kPivotStowedPositionDeg.initDefault(Units.rotationsToDegrees(kMinAngleRevs));
    kPivotDeployedPositionDeg.initDefault(Units.rotationsToDegrees(kMaxAngleRevs));

    kStowedSpeedVolts.initDefault(0.0);
    kStowingSpeedVolts.initDefault(4.5);
    kIntakingSpeedVolts.initDefault(4.5);
    kOutakingSpeedVolts.initDefault(-9.0);
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final ServoIO mPivot, mRoller;

  private boolean mIsHomed = false;

  private double mTargetAngleRevs = kMinAngleRevs;

  private boolean mIsRunning = false;
  private final Debouncer mIsStuck = new Debouncer(0.1);
  private final Debouncer mIsAtSetpoint = new Debouncer(0.1);

  protected Intake(ServoIO pivot, ServoIO roller) {
    mPivot = pivot;
    mRoller = roller;

    mPivot.resetRelativeEncoder(kMinAngleRevs);

    setDefaultCommand(stow().repeatedly());
  }

  @Override
  public void periodic() {
    // Update all hardware
    mPivot.periodic();
    mRoller.periodic();

    mIsStuck.calculate(
        mIsRunning && MathUtil.isNear(0.0, mRoller.getAngularVelocityRevsPerSec(), 0.1));
    mIsAtSetpoint.calculate(
        MathUtil.isNear(
            getRotation2d().getRotations(), getTargetRotation2d().getRotations(), kEpsilonRevs));

    if (kPivotKs.hasChanged(hashCode())
        && kPivotKg.hasChanged(hashCode())
        && kPivotKv.hasChanged(hashCode())
        && kPivotKa.hasChanged(hashCode())
        && kPivotKp.hasChanged(hashCode())
        && kPivotKd.hasChanged(hashCode())) {
      mPivot.setGains(
          kPivotKp.get(),
          kPivotKd.get(),
          kPivotKs.get(),
          kPivotKg.get(),
          kPivotKv.get(),
          kPivotKa.get());
    }

    if (kPivotMaxVelocityRevsPerSec.hasChanged(hashCode())
        || kPivotMaxAccelerationRevsPerSecPerSec.hasChanged(hashCode())) {
      mPivot.setProfilingConstraints(
          kPivotMaxVelocityRevsPerSec.get(), kPivotMaxAccelerationRevsPerSecPerSec.get());
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
   * Run pivot backwards at a constant voltage until it stops moving
   *
   * <p>Then, set current position as pivot home angle, aka its minimum angle
   *
   * @return {@link Command}
   */
  public Command runCurrentHoming() {
    return this.run(() -> mPivot.setVoltageOutput(kPivotHomingVolts, true))
        .until(
            () ->
                MathUtil.isNear(
                    0.0,
                    mPivot.getAngularVelocityRevsPerSec(),
                    kPivotZeroingVelocityThreshold.get()))
        .andThen(Commands.print("Intake Homed").alongWith(homePivot()));
  }

  /**
   * Reset pivot relative encoder to minimum angle
   *
   * @return {@link Command}
   */
  protected Command homePivot() {
    return this.runOnce(
        () -> {
          mPivot.setPositionSetpoint(kMinAngleRevs);
          mIsHomed = true;
        });
  }

  public Command stow() {
    return runSetpoints(
        Units.degreesToRotations(kPivotStowedPositionDeg.get()), kStowingSpeedVolts.get());
  }

  public Command intake() {
    return runSetpoints(
        Units.degreesToRotations(kPivotDeployedPositionDeg.get()), kIntakingSpeedVolts.get());
  }

  public Command outake() {
    return runSetpoints(
        Units.degreesToRotations(kPivotDeployedPositionDeg.get()), kOutakingSpeedVolts.get());
  }

  protected Command runSetpoints(double angleRevs, double speedVolts) {
    return this.run(
        () -> {
          mTargetAngleRevs = MathUtil.clamp(angleRevs, kMinAngleRevs, kMaxAngleRevs);
          mPivot.setProfiledPositionSetpoint(angleRevs);
          mRoller.setVoltageOutput(speedVolts, false);
        });
  }
}
