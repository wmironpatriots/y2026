// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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

/** {@link SubsystemBase} Manager class for the shooter */
public class ShooterSubsystem extends SubsystemBase {
  /**
   * Static Factory for automatically configuring and creating a {@link ShooterSubsystem}
   *
   * @return {@link ShooterSubsystem}
   */
  public static ShooterSubsystem create() {
    return (Robot.isReal())
        ? new ShooterSubsystem(
            new ServoIOTalonFx(
                "Pivot", MotorType.KrakenX44, kCanBus, kPivotCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Left", MotorType.KrakenX60, kCanBus, kFlywheelLeftCanDeviceId, kFlywheelConfig),
            new ServoIOTalonFx(
                "Right", MotorType.KrakenX60, kCanBus, kFlywheelRightCanDeviceId, kFlywheelConfig))
        : new ShooterSubsystem(
            new ServoIOTalonFxPivotSim(
                "Pivot",
                MotorType.KrakenX44,
                kCanBus,
                kPivotCanDeviceId,
                kPivotConfig,
                kPivotRotationalInertiaKgSquaredMeters,
                kPivotArmLengthMeters,
                Units.rotationsToDegrees(kMinAngleRevs),
                Units.rotationsToDegrees(kMaxAngleRevs),
                Units.rotationsToDegrees(kMinAngleRevs),
                true,
                DCMotor.getKrakenX60Foc(1),
                kPivotGearRatio),
            new ServoIOTalonFxFlywheelSim(
                "Left",
                MotorType.KrakenX60,
                kCanBus,
                kFlywheelLeftCanDeviceId,
                kPivotConfig,
                kFlywheelRotationalInertiaKgSquaredMeters,
                DCMotor.getKrakenX60Foc(2),
                kFlywheelGearRatio),
            new ServoIONone("Right")); // No need for this one me thinks
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device identifier for the pivot servo */
  public static final int kPivotCanDeviceId = Matrix.kHoodId;

  /** {@link Integer} Unique CAN device identifier for left flywheel servo */
  public static final int kFlywheelLeftCanDeviceId = Matrix.kFlywheelLeftId;

  /** {@link Integer} Unique CAN device identifier for right flywheel servo */
  public static final int kFlywheelRightCanDeviceId = Matrix.kFlywheelRightId;

  /** {@link Double} Lower limit on pivot angular position */
  public static final double kMinAngleRevs = Units.degreesToRotations(14.703759);

  /** {@link Double} Upper limit on pivot angular position */
  public static final double kMaxAngleRevs = Units.degreesToRotations(45.812);

  /** {@link Double} Gear ratio between pivot servo rotor and the mech output */
  public static final double kPivotGearRatio = 2.57142857143 * 10.83;

  /** {@link Double} Gear ratio between flywheel servos to mechanism */
  public static final double kFlywheelGearRatio = (24.0 / 20.0);

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
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kPivotGearRatio));

  /** {@link TalonFXConfiguration} Hardware config of flywheel servos */
  public static final TalonFXConfiguration kFlywheelConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(70.0)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(40.0)
                  .withSupplyCurrentLimitEnable(true))
          .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(100.0))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kFlywheelGearRatio));

  /** {@link Double} Min filtered stator current value to be considered homed */
  public static final double kPivotCurrentZeroThreshold = 30.0;

  /** {@link Double} Moment of Inertia of pivot system */
  public static final double kPivotRotationalInertiaKgSquaredMeters = 402.290096 * 0.0002926397;

  /** {@link Double} Length of pivot arm in meters */
  public static final double kPivotArmLengthMeters = 0.5;

  /** {@link Double} Moment of Inertia of flywheel */
  public static final double kFlywheelRotationalInertiaKgSquaredMeters = 10.491008 * 0.0002926397;

  /** {@link Double} Radius of flywheel */
  public static final double kFlywheelRadiusMeters = Units.inchesToMeters(2);

  /**
   * {@link Transform2d} Transform from center of robot on floor to approximated projectile exit
   * location
   */
  public static final Transform3d kRobotToShooter =
      new Transform3d(
          Units.inchesToMeters(-8.3), 0.0, Units.inchesToMeters(24.6), Rotation3d.kZero);

  public static final InterpolatingProjectileParametersTree kHubShotMap =
      new InterpolatingProjectileParametersTree();

  public static final InterpolatingProjectileParametersTree kAllianceShotMap =
      new InterpolatingProjectileParametersTree();

  static {
  }

  /** {@link String} Nt directory to store tunables in */
  public static final String kTunablesPrefix = "/Shooter";

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  /** {@link TunableNumber} Max acceptable angular position error (degrees) */
  public static TunableNumber kPivotToleranceDeg =
      new TunableNumber(kTunablesPrefix + "/Pivot/Epsilon (degrees)");

  /** {@link TunableNumber} Gain acting against static fricition */
  public static TunableNumber kPivotKs = new TunableNumber(kTunablesPrefix + "/Pivot/Ks");

  /** {@link TunableNumber} Gain inducing velocity */
  public static TunableNumber kPivotKv = new TunableNumber(kTunablesPrefix + "/Pivot/Kv");

  /** {@link TunableNumber} Gain inducing acceleration */
  public static TunableNumber kPivotKa = new TunableNumber(kTunablesPrefix + "/Pivot/Ka");

  /** {@link TunableNumber} Gain driving angular position error to zero */
  public static TunableNumber kPivotKp = new TunableNumber(kTunablesPrefix + "/Pivot/Kp");

  /** {@link TunableNumber} Gain driving the derivative of angular position error to zero */
  public static TunableNumber kPivotKd = new TunableNumber(kTunablesPrefix + "/Pivot/Kd");

  /** {@link TunableNumber} Angular velocity cap on pivot (revs per sec) */
  public static TunableNumber kPivotCruiseVelocityRevsPerSec =
      new TunableNumber(
          kTunablesPrefix + "/Pivot/Cruise Velocity (revolutions per second per second)");

  /** {@link TunaleNumber} Angular Acceleration cap on pivot (revs per sec per sec) */
  public static TunableNumber kPivotMaxAccelerationRevsPerSecPerSec =
      new TunableNumber(
          kTunablesPrefix + "/Pivot/Max Acceleratoin (revolutions per second per second)");

  /** {@link TunableNumber} Tunable for manually controlling the angular position of subsystem */
  public static TunableNumber kPivotTestingAngleDeg =
      new TunableNumber(kTunablesPrefix + "/Pivot/Testing Angle (degrees)");

  /** {@link TunableNumber} Max acceptable angular velocity error (degrees per second) */
  public static TunableNumber kFlywheelToleranceCmPerSec =
      new TunableNumber(kTunablesPrefix + "/Flywheel/Epsilon (centimeters per second)");

  /** {@link TunableNumber} Gain acting against static friciton */
  public static TunableNumber kFlywheelKs = new TunableNumber(kTunablesPrefix + "/Flywheel/Ks");

  /** {@link TunableNumber} Gain inducing velocity */
  public static TunableNumber kFlywheelKv = new TunableNumber(kTunablesPrefix + "/Flywheel/Kv");

  /** {@link TunableNumber} Gain inducing acceleration */
  public static TunableNumber kFlywheelKa = new TunableNumber(kTunablesPrefix + "/Flywheel/Ka");

  /** {@link TunableNumber} Gain driving angular position error to zero */
  public static TunableNumber kFlywheelKp = new TunableNumber(kTunablesPrefix + "/Flywheel/Kp");

  /** {@link TunableNumber} Gain driving the deriviative of angular position to zero */
  public static TunableNumber kFlywheelKd = new TunableNumber(kTunablesPrefix + "/Flywheel/Kd");

  /** {@link TunableNumber} Tunable for manually controlling the angular velocity of subsystem */
  public static TunableNumber kFlywheelTestingSpeedMetersPerSec =
      new TunableNumber(kTunablesPrefix + "/Flywheel/Testing Muzzle Velocity (meters per second)");

  public static TunableNumber kLatencyCompensationSec =
      new TunableNumber(kTunablesPrefix + "/LatencyCompensationSec");

  static {
    if (Robot.isReal()) {
      kPivotToleranceDeg.initDefault(1.0);
      kPivotKs.initDefault(0.0);
      kPivotKv.initDefault(0.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(4000.0);
      kPivotKd.initDefault(25.0);
      kPivotCruiseVelocityRevsPerSec.initDefault(2);
      kPivotMaxAccelerationRevsPerSecPerSec.initDefault(3);
      kPivotTestingAngleDeg.initDefault(Units.rotationsToDegrees(kMinAngleRevs));

      kFlywheelToleranceCmPerSec.initDefault(1.0);
      kFlywheelKs.initDefault(3.035);
      kFlywheelKv.initDefault(0.75631);
      kFlywheelKa.initDefault(7.4852);
      kFlywheelKp.initDefault(15.9);
      kFlywheelKd.initDefault(0.0);
      kFlywheelTestingSpeedMetersPerSec.initDefault(0.0);

      kLatencyCompensationSec.initDefault(0.3);
    } else {
      kPivotToleranceDeg.initDefault(1.0);
      kPivotKs.initDefault(0.0);
      kPivotKv.initDefault(0.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(0.0);
      kPivotKd.initDefault(0.0);
      kPivotTestingAngleDeg.initDefault(Units.rotationsToDegrees(kMinAngleRevs));

      kFlywheelToleranceCmPerSec.initDefault(1.0);
      kFlywheelKs.initDefault(4.8691);
      kFlywheelKv.initDefault(0.10515);
      kFlywheelKa.initDefault(1.729);
      kFlywheelKp.initDefault(0.26322);
      kFlywheelKd.initDefault(0.0);
      kFlywheelTestingSpeedMetersPerSec.initDefault(0.0);

      kLatencyCompensationSec.initDefault(0.3);
    }
  }

  /**
   * Convert a flywheel velocity (revolutions per second) to muzzle velocity (meters per second)
   *
   * @param flywheelVelocityRps {@link Double} Flywheel Velocity in revolutions per second
   * @return {@link Double}
   */
  public static double flywheelVelocityRpsToMuzzleVelocityMps(double flywheelVelocityRps) {
    return flywheelVelocityRps * Math.PI * 2 * kFlywheelRadiusMeters * 0.5;
  }

  /**
   * Convert a muzzle velocity (meters per second) to flywheel velocity (revolutions per second)
   *
   * @param muzzleVelocityMps {@link Double} Muzzle Velocity in meters per second
   * @return {@link Double}
   */
  public static double muzzleVelocityMpsToFlywheelVelocityRps(double muzzleVelocityMps) {
    return muzzleVelocityMps * 2 / kFlywheelRadiusMeters / (2 * Math.PI);
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  /** {@link ServoIO} Hardware Interface for the hood pivot */
  @Logged private final ServoIO mPivot;

  /** {@link ServoIO} Hardware Interface for the left side flywheel motor */
  @Logged private final ServoIO mLeft;

  /** {@link ServoIO} Hardware Interface for the right side flywheel motor */
  @Logged private final ServoIO mRight;

  /** {@link SysIdRoutine} Characterization routines for the flywheel */
  private final SysIdRoutine mFlywheelCharacterization;

  /** {@link LinearFilter} Moving average filter for checking current spikes for the pivot motor */
  private final LinearFilter mPivotCurrentFilter = LinearFilter.movingAverage(10);

  @Logged(name = "Filtered Stator Current (amps)")
  private double mPivotCurrentFilterValue = 0.0;

  /** {@link Boolean} Latch for checking if subsystem has been homed */
  @Logged(name = "Is Hood Homed (bool)")
  private boolean mIsHomed = false;

  private Debouncer mIsHolding = new Debouncer(0.2);

  @Logged(name = "Target Muzzle Velocity (meters per second)")
  private double mTargetMuzzleVelocityMps = 0.0;

  @Logged(name = "Target Angle (Rotation2d)")
  private Rotation2d mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);

  protected ShooterSubsystem(ServoIO pivot, ServoIO left, ServoIO right) {
    mPivot = pivot;
    mLeft = left;
    mRight = right;

    mRight.setLeader(mLeft, true);

    mLeft.resetRelativeEncoder();

    mFlywheelCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(25).per(Second),
                Volts.of(15),
                null,
                (state) -> SmartDashboard.putString("FlywheelSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> mLeft.setTorqueCurrentOutput(voltage.in(Volts)), null, this));

    mPivot.resetRelativeEncoder(kMinAngleRevs);
  }

  @Override
  public void periodic() {
    // Update all Hardware
    mPivot.periodic();
    mLeft.periodic();
    mRight.periodic();

    // Calculate filtered pivot current
    mPivotCurrentFilterValue = mPivotCurrentFilter.calculate(mPivot.getStatorCurrentAmps());

    // Update gains from tunables
    if (kPivotKs.hasChanged(hashCode())
        || kPivotKv.hasChanged(hashCode())
        || kPivotKa.hasChanged(hashCode())
        || kPivotKp.hasChanged(hashCode())
        || kPivotKd.hasChanged(hashCode())
        || kPivotCruiseVelocityRevsPerSec.hasChanged(hashCode())
        || kPivotMaxAccelerationRevsPerSecPerSec.hasChanged(hashCode())) {
      mPivot.setGains(
          kPivotKp.get(), kPivotKd.get(), kPivotKs.get(), 0.0, kPivotKv.get(), kPivotKa.get());
      mPivot.setProfilingConstraints(
          kPivotCruiseVelocityRevsPerSec.get(), kPivotMaxAccelerationRevsPerSecPerSec.get());
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
   * Check if shooter is holding setpoint aka is ready to shoot
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Holding Setpoint (bool)", importance = Importance.INFO)
  public boolean isHoldingSetpoint() {
    return mIsHolding.calculate(
        MathUtil.isNear(
                getTargetRotation2d().getDegrees(),
                getRotation2d().getDegrees(),
                kPivotToleranceDeg.getAsDouble())
            && MathUtil.isNear(
                getTargetMuzzleVelocityMetersPerSec(),
                getApproximatedMuzzleVelocityMetersPerSec(),
                kFlywheelToleranceCmPerSec.getAsDouble() / 100.0));
  }

  /**
   * Get hood angle
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Angle (Rotation2d)", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(mLeft.getAngularPositionRevs());
  }

  /**
   * Get desired hood angle
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Target Angle (Rotation2d)", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return mTargetRotation2d;
  }

  /**
   * Get expected projectile exit velocity assuming no slippage (meters per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Approximated Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getApproximatedMuzzleVelocityMetersPerSec() {
    return flywheelVelocityRpsToMuzzleVelocityMps(getFlywheelVelocityRevsPerSec());
  }

  /**
   * Get flywheel angular velocity (revolutions per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Flywheel Velocity (revolutions per second)", importance = Importance.INFO)
  public double getFlywheelVelocityRevsPerSec() {
    return mLeft.getAngularVelocityRevsPerSec();
  }

  /**
   * Get desired projectile exit velocity (meters per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Target Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getTargetMuzzleVelocityMetersPerSec() {
    return mTargetMuzzleVelocityMps;
  }

  /**
   * Get desired flywheel angular velocity (revolutions per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Target Flywheel Velocity (revolutions per second)", importance = Importance.INFO)
  public double getTargetFlywheelVelocityRevsPerSec() {
    return muzzleVelocityMpsToFlywheelVelocityRps(getTargetMuzzleVelocityMetersPerSec());
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Run flywheel System Identification (quasis forward, quasis reverse, dynamic forward, dynamic
   * reverse)
   *
   * @return {@link Command}
   */
  public Command runFlywheelCharacterizationSequence() {
    return Commands.sequence(
            Commands.print("Starting Quasistatic Forward"),
            mFlywheelCharacterization.quasistatic(Direction.kForward),
            Commands.print("Starting Quasistatic Reverse"),
            mFlywheelCharacterization.quasistatic(Direction.kReverse),
            Commands.print("Starting Dynamic Forward"),
            mFlywheelCharacterization.dynamic(Direction.kForward),
            Commands.print("Starting Dynamic Forward"),
            mFlywheelCharacterization.dynamic(Direction.kReverse))
        .beforeStarting(
            () -> {
              mTargetMuzzleVelocityMps = 0.0;
              mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);
            })
        .withName("Run Flywheel Characterization");
  }

  /**
   * Run homing sequence to reset hood pivot
   *
   * <p>Command will run a constant backwards voltage until pivot current spikes
   *
   * @return {@link Command}
   */
  public Command runCurrentHoming() {
    return this.run(() -> mPivot.setVoltageOutput(-4.5, true))
        .until(
            new Trigger(() -> Math.abs(mPivotCurrentFilterValue) > kPivotCurrentZeroThreshold)
                .debounce(0.25))
        .andThen(
            this.run(() -> mPivot.resetRelativeEncoder()).alongWith(Commands.print("Hood Homed")))
        .finallyDo((bool) -> mIsHomed = bool ? false : true)
        .withName("Run Current Homing");
  }

  /**
   * Run shooter at setpoints given by tunables {@link #kFlywheelTestingSpeedMetersPerSec} & {@link
   * #kPivotTestingAngleDeg}
   *
   * @return {@link Command}
   */
  public Command runTunablesSetpoint() {
    return runSetpoint(
            kFlywheelTestingSpeedMetersPerSec,
            () -> Rotation2d.fromDegrees(kPivotTestingAngleDeg.get()))
        .withName("Run Tunables Setpoint");
  }

  /**
   * Run shooter at specified setpoints
   *
   * @param desiredMuzzleVelMps {@link DoubleSupplier} Desired muzzle velocity (projectile exit
   *     velocity) in meters per second
   * @param desiredAngle {@link Supplier} of {@link Rotation2d} Desired hood angle
   * @return {@link Command}
   */
  public Command runSetpoint(
      DoubleSupplier desiredMuzzleVelMps, Supplier<Rotation2d> desiredAngle) {
    return this.run(
            () -> {
              // Clamp values
              mTargetMuzzleVelocityMps = desiredMuzzleVelMps.getAsDouble();
              mTargetRotation2d =
                  Rotation2d.fromRotations(
                      MathUtil.clamp(
                          desiredAngle.get().getRotations(), kMinAngleRevs, kMaxAngleRevs));

              mLeft.setProfiledVelocitySetpoint(getTargetFlywheelVelocityRevsPerSec());
              mPivot.setProfiledPositionSetpoint(mTargetRotation2d.getRotations());
            })
        .withName("Run Shooter Setpoint");
  }

  /**
   * Stow hood & let flywheel coast
   *
   * @return {@link Command}
   */
  public Command stowAndCoast() {
    return this.run(
        () -> {
          mTargetMuzzleVelocityMps = getApproximatedMuzzleVelocityMetersPerSec();
          mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);

          mLeft.setNeutral();
          mPivot.setProfiledPositionSetpoint(mTargetRotation2d.getRotations());
        });
  }

  /**
   * Stow hood & apply flywheel brake
   *
   * @return {@link Command}
   */
  public Command stowAndDeaccel() {
    return this.run(
        () -> {
          mTargetMuzzleVelocityMps = 0.0;
          mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);

          mLeft.setVelocitySetpoint(getTargetFlywheelVelocityRevsPerSec());
          mPivot.setProfiledPositionSetpoint(mTargetRotation2d.getRotations());
        });
  }

  // public ProjectileParameters calculateParameters() {
  // // Get current state
  // var position = RobotState.getInstance().getEstimatedPosition();
  // var fieldRelativeSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
  // var speeds = RobotState.getInstance().getChassisSpeeds();

  // var target = Flags.getRobotAlliancePose2d(Rebuilt.kHubPose2d);

  // // Compensate for predicted latency
  // var predictedPosition =
  //     position.exp(
  //         new Twist2d(
  //             speeds.vxMetersPerSecond * kLatencyCompensationSec.get(),
  //             speeds.vyMetersPerSecond * kLatencyCompensationSec.get(),
  //             speeds.omegaRadiansPerSecond * kLatencyCompensationSec.get()));

  // // Calculate for a standstill shot at position for a tof approximation
  // var approximatedTof =
  //     kHubShotTree
  //         .get(predictedPosition.getTranslation().getDistance(target.getTranslation()))
  //         .timeOfFlightSec();

  // // Calculate virtual target
  // mVirtualTarget =
  //     target
  //         .getTranslation()
  //         .minus(
  //             new Translation2d(
  //                 fieldRelativeSpeeds.vxMetersPerSecond * approximatedTof,
  //                 fieldRelativeSpeeds.vyMetersPerSecond * approximatedTof));

  // // Calculate final shot
  // var parameters =
  //     kHubShotTree.get(predictedPosition.getTranslation().getDistance(mVirtualTarget));

  // if (parameters != null) {
  //   return parameters;
  // } else return new ProjectileParameters(0.0, 0.0, 0.0);

  //   return kHubShotTree.calculateProjectileParameters(
  //       RobotState.getInstance().getEstimatedPosition().getTranslation(),
  //       Flags.getRobotAlliancePose2d(Rebuilt.kHubPose2d).getTranslation());
  // }
}
