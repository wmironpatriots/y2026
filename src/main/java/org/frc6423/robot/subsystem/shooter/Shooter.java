// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxFlywheelSim;
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.lib.util.TunableNumber;
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
            new ServoIOTalonFx(
                "Pivot", MotorType.KrakenX44, kCanBus, kPivotCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Left", MotorType.KrakenX60, kCanBus, kFlywheelLeftCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Right", MotorType.KrakenX60, kCanBus, kFlywheelRightCanDeviceId, kPivotConfig))
        : new Shooter(
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
                DCMotor.getKrakenX60Foc(2),
                kFlywheelRotorToMechRatio),
            new ServoIONone("Right")); // No need for this one me thinks
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device identifier for the pivot servo */
  public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

  /** {@link Integer} Unique CAN device identifier for left flywheel servo */
  public static final int kFlywheelLeftCanDeviceId = Matrix.kFlywheelLeftId;

  /** {@link Integer} Unique CAN device identifier for right flywheel servo */
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
                  .withSensorToMechanismRatio(kPivotRotorToSensorRatio * kPivotSensorToMechRatio));

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
                  .withSensorToMechanismRatio(kFlywheelRotorToMechRatio));

  /** {@link Double} Moment of Inertia of pivot system */
  public static final double kPivotRotationalInertiaKgSquaredMeters = 402.290096 * 0.0002926397;

  /** {@link Double} Length of pivot arm in meters */
  public static final double kPivotArmLengthMeters = 0.5;

  public static final double kPivotHomingVolts = 2.5;

  /** {@link Double} Moment of Inertia of flywheel */
  public static final double kFlywheelRotationalInertiaKgSquaredMeters = 10.491008 * 0.0002926397;

  /** {@link Double} Radius of flywheel */
  public static final double kFlywheelRadiusMeters = Units.inchesToMeters(2);

  /** {@link String} Nt directory to store tunables in */
  public static final String kTunablesPrefix = "/Shooter";

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  public static TunableNumber kPivotZeroingVelocityThreshold =
      new TunableNumber(kTunablesPrefix + "/Pivot/Zeroing Velocity Threshold (deg per sec)");

  /** {@link TunableNumber} Maximum allowed angular position error for pivot */
  public static TunableNumber kPivotEpsilonDeg =
      new TunableNumber(kTunablesPrefix + "/Pivot/Epsilon (degrees)");

  /** {@link TunableNumber} Pivot control gain acting against static friction (kS * signum(vel)) */
  public static TunableNumber kPivotKs = new TunableNumber(kTunablesPrefix + "/Pivot/Ks");

  /** {@link TunableNumber} Pivot control gain inducing velocity (kV * desired_vel) */
  public static TunableNumber kPivotKv = new TunableNumber(kTunablesPrefix + "/Pivot/Kv");

  /** {@link TunableNumber} Pivot control gain inducing acceleration (kV * desired_accel) */
  public static TunableNumber kPivotKa = new TunableNumber(kTunablesPrefix + "/Pivot/Ka");

  /** {@link TunableNumber} Pivot control gain for driving angular position error to 0 */
  public static TunableNumber kPivotKp = new TunableNumber(kTunablesPrefix + "/Pivot/Kp");

  /** {@link TunableNumber} Pivot control gain for driving angular velocity error to 0 */
  public static TunableNumber kPivotKd = new TunableNumber(kTunablesPrefix + "/Pivot/Kd");

  /** {@link TunableNumber} Maximum allowed angular velocity error for flywheel */
  public static TunableNumber kFlywheelEpsilonDegPerSec =
      new TunableNumber(kTunablesPrefix + "/Flywheel/Epsilon (degrees per second)");

  /**
   * {@link TunableNumber} Flywheel control gain acting against static friction (kS * signum(vel))
   */
  public static TunableNumber kFlywheelKs = new TunableNumber(kTunablesPrefix + "/Flywheel/Ks");

  /** {@link TunableNumber} Flywheel control gain inducing velocity (kV * desired_vel) */
  public static TunableNumber kFlywheelKv = new TunableNumber(kTunablesPrefix + "/Flywheel/Kv");

  /** {@link TunableNumber} Flywheel control gain inducing acceleration (kV * desired_accel) */
  public static TunableNumber kFlywheelKa = new TunableNumber(kTunablesPrefix + "/Flywheel/Ka");

  /** {@link TunableNumber} Flywheel control gain for driving angular position error to 0 */
  public static TunableNumber kFlywheelKp = new TunableNumber(kTunablesPrefix + "/Flywheel/Kp");

  /** {@link TunableNumber} Flywheel control gain for driving angular velocity error to 0 */
  public static TunableNumber kFlywheelKd = new TunableNumber(kTunablesPrefix + "/Flywheel/Kd");

  static {
    if (Robot.isReal()) {
      kPivotZeroingVelocityThreshold.initDefault(0.1);
      kPivotEpsilonDeg.initDefault(0.0);
      kPivotKs.initDefault(0.0);
      kPivotKv.initDefault(0.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(0.0);
      kPivotKd.initDefault(0.0);

      kFlywheelEpsilonDegPerSec.initDefault(0.0);
      kFlywheelKs.initDefault(4.8691);
      kFlywheelKv.initDefault(0.10515);
      kFlywheelKa.initDefault(1.729);
      kFlywheelKp.initDefault(0.26322);
      kFlywheelKd.initDefault(0.0);
    } else {
      kPivotEpsilonDeg.initDefault(0.0);
      kPivotKs.initDefault(0.0);
      kPivotKv.initDefault(0.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(0.0);
      kPivotKd.initDefault(0.0);

      kFlywheelEpsilonDegPerSec.initDefault(0.0);
      kFlywheelKs.initDefault(4.8691);
      kFlywheelKv.initDefault(0.10515);
      kFlywheelKa.initDefault(1.729);
      kFlywheelKp.initDefault(0.26322);
      kFlywheelKd.initDefault(0.0);
    }
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final ServoIO mPivot, mLeft, mRight;

  private boolean mIsHomed = false;

  private Rotation2d mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);
  private double mTargetFlywheelVelocityRevsPerSec = 0.0;

  private final Debouncer mIsPivotHoldingTarget = new Debouncer(0.1);
  private final Debouncer mIsFlywheelHoldingTarget = new Debouncer(0.03);

  protected Shooter(ServoIO pivot, ServoIO left, ServoIO right) {
    mPivot = pivot;
    mLeft = left;
    mRight = right;

    mRight.setLeader(mLeft, true);
  }

  @Override
  public void periodic() {
    // Update all Hardware
    mPivot.periodic();
    mLeft.periodic();
    mRight.periodic();

    // Calculate debouncers
    mIsPivotHoldingTarget.calculate(
        MathUtil.isNear(
            getTargetFlywheelVelocityRevsPerSec(),
            getFlywheelVelocityRevsPerSec(),
            Units.degreesToRotations(kPivotEpsilonDeg.get())));
    mIsFlywheelHoldingTarget.calculate(
        MathUtil.isNear(
            getTargetFlywheelVelocityRevsPerSec(),
            getFlywheelVelocityRevsPerSec(),
            Units.degreesToRotations(kFlywheelEpsilonDegPerSec.get())));

    // Update gains from tunables
    if (kPivotKs.hasChanged(hashCode())
        || kPivotKv.hasChanged(hashCode())
        || kPivotKa.hasChanged(hashCode())
        || kPivotKp.hasChanged(hashCode())
        || kPivotKd.hasChanged(hashCode())) {
      mPivot.setGains(
          kPivotKp.get(), kPivotKd.get(), kPivotKs.get(), 0.0, kPivotKv.get(), kPivotKa.get());
    }

    if (kFlywheelKs.hasChanged(hashCode())
        || kFlywheelKv.hasChanged(hashCode())
        || kFlywheelKa.hasChanged(hashCode())
        || kFlywheelKp.hasChanged(hashCode())
        || kFlywheelKd.hasChanged(hashCode())) {
      mLeft.setGains(
          kFlywheelKp.get(),
          kFlywheelKd.get(),
          kFlywheelKs.get(),
          0.0,
          kFlywheelKv.get(),
          kFlywheelKa.get());
      mRight.setGains(
          kFlywheelKp.get(),
          kFlywheelKd.get(),
          kFlywheelKs.get(),
          0.0,
          kFlywheelKv.get(),
          kFlywheelKa.get());
    }
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Check if pivot has been homed
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Homed (bool)", importance = Importance.INFO)
  public boolean isHomed() {
    return mIsHomed;
  }

  /**
   * Check if pivot & flywheel are aimed and up to speed
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Locked (bool)", importance = Importance.INFO)
  public boolean isLocked() {
    return mIsPivotHoldingTarget.calculate(
            MathUtil.isNear(
                getTargetFlywheelVelocityRevsPerSec(),
                getFlywheelVelocityRevsPerSec(),
                Units.degreesToRotations(kPivotEpsilonDeg.get())))
        && mIsFlywheelHoldingTarget.calculate(
            MathUtil.isNear(
                getTargetFlywheelVelocityRevsPerSec(),
                getFlywheelVelocityRevsPerSec(),
                Units.degreesToRotations(kFlywheelEpsilonDegPerSec.get())));
  }

  /**
   * Get angle shooter is aimed at
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(mPivot.getAngularPositionRevs());
  }

  /**
   * Get target angle shooter needs to be at
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Target Rotation2d", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return mTargetRotation2d;
  }

  /**
   * Get an approximation of the exit velocity of projectiles
   *
   * @return {@link Double}
   */
  @Logged(name = "Approximated Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getApproximatedMuzzleVelocityMetersPerSec() {
    return getFlywheelVelocityRevsPerSec() * kFlywheelRadiusMeters * 0.5;
  }

  /**
   * Get angular velocity of flywheel
   *
   * @return {@link Double}
   */
  @Logged(name = "Flywheel Velocity (revs per sec)", importance = Importance.INFO)
  public double getFlywheelVelocityRevsPerSec() {
    return mLeft.getAngularVelocityRevsPerSec();
  }

  /**
   * Get the desired exit velocity of projectiles
   *
   * @return {@link Double}
   */
  @Logged(name = "Target Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getTargetMuzzleVelocityMetersPerSecond() {
    return getTargetFlywheelVelocityRevsPerSec() * kFlywheelRadiusMeters * 0.5;
  }

  /**
   * Get target flywheel velocity shooter needs to be spinning at
   *
   * @return {@link Double}
   */
  @Logged(name = "Target Flywheel Velocity (revs per sec)", importance = Importance.INFO)
  public double getTargetFlywheelVelocityRevsPerSec() {
    return mTargetFlywheelVelocityRevsPerSec;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  protected void setPivotStow() {
    mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);
    setPivotSetpoint(mTargetRotation2d);
  }

  protected void setPivotSetpoint(Rotation2d angle) {
    mTargetRotation2d =
        Rotation2d.fromRotations(
            MathUtil.clamp(angle.getRotations(), kMinAngleRevs, kMaxAngleRevs));

    mPivot.setPositionSetpoint(mTargetRotation2d.getRotations());
  }

  protected void setFlywheelCoast() {
    mLeft.setNeutral();
  }

  protected void setFlywheelStop() {
    mTargetFlywheelVelocityRevsPerSec = 0.0;
    setFlywheelSetpoint(0.0);
  }

  protected void setFlywheelSetpoint(double velocityRevsPerSec) {
    mTargetFlywheelVelocityRevsPerSec = velocityRevsPerSec;
    mLeft.setProfiledVelocitySetpoint(mTargetFlywheelVelocityRevsPerSec);
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Stow hood and deaccelerate flywheel
   *
   * @return {@link Command}
   */
  public Command stowAndStop() {
    return this.run(
        () -> {
          setPivotStow();
          setFlywheelStop();
        });
  }

  /**
   * Stow hood and let flywheel coast
   *
   * @return {@link Command}
   */
  public Command stowAndCoast() {
    return this.run(
        () -> {
          setPivotStow();
          setFlywheelCoast();
        });
  }

  /**
   * Adjust hood to specified angle and let flywheel coast
   *
   * @param angle {@link Supplier} of {@link Rotation} Angle to adjust to
   * @return {@link Command}
   */
  public Command adjustToAngleAndCoast(Supplier<Rotation2d> angle) {
    return this.run(
        () -> {
          setPivotSetpoint(angle.get());
          setFlywheelCoast();
        });
  }

  /**
   * Stow hood and accelerate flywheel to specified velocity
   *
   * @param velocityRevsPerSec {@link Supplier} of {@link Double} Velocity to accelerate to
   * @return {@link Command}
   */
  public Command stowAndAccelerateTo(DoubleSupplier velocityRevsPerSec) {
    return this.run(
        () -> {
          setPivotStow();
          setFlywheelSetpoint(velocityRevsPerSec.getAsDouble());
        });
  }

  /**
   * Adjust hood to specified angle and accelerate flywheel to specified velocity
   *
   * @param angle {@link Supplier} of {@link Rotation} Angle to adjust to
   * @param velocityRevsPerSec {@link Supplier} of {@link Double} Velocity to accelerate to
   * @return {@link Command}
   */
  public Command adjustToAndAccelerateTo(
      Supplier<Rotation2d> angle, DoubleSupplier velocityRevsPerSec) {
    return this.run(
        () -> {
          setPivotSetpoint(angle.get());
          setFlywheelSetpoint(velocityRevsPerSec.getAsDouble());
        });
  }

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
        .andThen(Commands.print("Hood Homed").alongWith(homePivot()));
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
}
