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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;
import org.frc6423.robot.fcs.ProjectileParameters;

public class ShooterSubsystem extends SubsystemBase {
  public static ShooterSubsystem create() {
    return (Robot.isReal())
        ? new ShooterSubsystem(
            new HoodIOReal(kHoodCanDeviceId, kCanBus, kHoodTalonConfig),
            new FlywheelIOReal(
                kFlywheelLeftCanDeviceId, kFlywheelRightCanDeviceId, kCanBus, kFlywheelTalonConfig))
        : new ShooterSubsystem(
            new HoodIOReal(kHoodCanDeviceId, kCanBus, kHoodTalonConfig),
            new FlywheelIOReal(
                kFlywheelLeftCanDeviceId,
                kFlywheelRightCanDeviceId,
                kCanBus,
                kFlywheelTalonConfig));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  public static final int kHoodCanDeviceId = Matrix.kHoodId;

  public static final int kFlywheelLeftCanDeviceId = Matrix.kFlywheelLeftId;

  public static final int kFlywheelRightCanDeviceId = Matrix.kFlywheelRightId;

  public static final double kHoodSensorToMechRatio = 2.57142857143 * 10.83;

  public static final double kFlywheelSensorToMechRatio = (24.0 / 20.0);

  public static final TalonFXConfiguration kHoodTalonConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(20.0)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kHoodSensorToMechRatio));

  public static final TalonFXConfiguration kFlywheelTalonConfig =
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
                  .withSupplyCurrentLimit(80.0)
                  .withSupplyCurrentLimitEnable(true))
          .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(100.0))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(kFlywheelSensorToMechRatio));

  public static final double kMinAngleRevs = Units.degreesToRotations(48.7);

  public static final double kMaxAngleRevs = Units.degreesToRotations(75.3);

  public static final double kHoodCurrentZeroingThreshold = 5.0;

  public static final double kRotationalInertiaKgSquaredMeters = 402.290096 * 0.0002926397;

  public static final double kHoodArmLengthMeters = 0.5;

  public static final double kFlywheelRotationalInertiaKgSquaredMeters = 10.491008 * 0.0002926397;

  public static final double kFlywheelRadiusMeters = Units.inchesToMeters(2.0);

  public static final Transform3d kRobotToShooter =
      new Transform3d(
          Units.inchesToMeters(-8.3), 0.0, Units.inchesToMeters(24.6), Rotation3d.kZero);

  /**
   * Convert a flywheel velocity (revolutions per second) to muzzle velocity (meters per second)
   *
   * @param flywheelVelocityRps {@link Double} Flywheel Velocity in revolutions per second
   * @return {@link Double}
   */
  public static double flywheelVelocityRpsToMuzzleVelocityMps(double flywheelVelocityRps) {
    return flywheelVelocityRps * Math.PI * 2 * kFlywheelRadiusMeters * 0.6;
  }

  /**
   * Convert a muzzle velocity (meters per second) to flywheel velocity (revolutions per second)
   *
   * @param muzzleVelocityMps {@link Double} Muzzle Velocity in meters per second
   * @return {@link Double}
   */
  public static double muzzleVelocityMpsToFlywheelVelocityRps(double muzzleVelocityMps) {
    return muzzleVelocityMps * 1.65 / kFlywheelRadiusMeters / (2 * Math.PI);
  }

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  public static final TunableNumber kHoodKs = new TunableNumber("Shooter/Hood kS");
  public static final TunableNumber kHoodKa = new TunableNumber("Shooter/Hood kA");
  public static final TunableNumber kHoodKv = new TunableNumber("Shooter/Hood kV");
  public static final TunableNumber kHoodKp = new TunableNumber("Shooter/Hood kP");
  public static final TunableNumber kHoodKd = new TunableNumber("Shooter/Hood kD");

  public static final TunableNumber kHoodCruiseVelocity =
      new TunableNumber("Shooter/Hood Velocity");
  public static final TunableNumber kHoodAcceleration =
      new TunableNumber("Shooter/Hood Acceleration");

  public static final TunableNumber kHoodToleranceDeg =
      new TunableNumber("Shooter/Hood Tolerance (degrees)");

  public static final TunableNumber kFlywheelKs = new TunableNumber("Shooter/Flywheel kS");
  public static final TunableNumber kFlywheelKa = new TunableNumber("Shooter/Flywheel kA");
  public static final TunableNumber kFlywheelKv = new TunableNumber("Shooter/Flywheel kV");
  public static final TunableNumber kFlywheelKp = new TunableNumber("Shooter/Flywheel kP");
  public static final TunableNumber kFlywheelKd = new TunableNumber("Shooter/Flywheel kD");

  public static final TunableNumber kFlywheelToleranceMetersPerSec =
      new TunableNumber("Shooter/Flywheel Tolerance (meters per second)");

  static {
    if (Robot.isReal()) {
      kHoodKs.initDefault(0.0);
      kHoodKv.initDefault(0.0);
      kHoodKa.initDefault(0.0);
      kHoodKp.initDefault(4000.0);
      kHoodKd.initDefault(25.0);

      kHoodCruiseVelocity.initDefault(3);
      kHoodAcceleration.initDefault(4);

      kHoodToleranceDeg.initDefault(0.75);

      kFlywheelKs.initDefault(3.035);
      kFlywheelKv.initDefault(0.75631);
      kFlywheelKa.initDefault(7.4852);
      kFlywheelKp.initDefault(15.9);
      kFlywheelKd.initDefault(0.0);

      kFlywheelToleranceMetersPerSec.initDefault(0.3);
    } else {
      kHoodKs.initDefault(0.0);
      kHoodKv.initDefault(0.0);
      kHoodKa.initDefault(0.0);
      kHoodKp.initDefault(4000.0);
      kHoodKd.initDefault(25.0);

      kHoodCruiseVelocity.initDefault(3);
      kHoodAcceleration.initDefault(4);

      kHoodToleranceDeg.initDefault(0.75);

      kFlywheelKs.initDefault(3.035);
      kFlywheelKv.initDefault(0.75631);
      kFlywheelKa.initDefault(7.4852);
      kFlywheelKp.initDefault(15.9);
      kFlywheelKd.initDefault(0.0);

      kFlywheelToleranceMetersPerSec.initDefault(0.3);
    }
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final HoodIO mHood;

  @Logged private final FlywheelIO mFlywheel;

  private final SysIdRoutine mFlywheelCharacterization;

  private final LinearFilter mHoodCurrentFilter = LinearFilter.movingAverage(10);
  private double mHoodCurrentFilterValue = 0.0;

  private boolean mIsHomed = false;

  private Rotation2d mTargetAngle = Rotation2d.kZero;
  private double mTargetMuzzleVelocityMps = 0.0;

  private Debouncer mIsNearAngle = new Debouncer(0.1, DebounceType.kRising);
  private Debouncer mIsNearSpeed = new Debouncer(0.05, DebounceType.kRising);

  protected ShooterSubsystem(HoodIO hood, FlywheelIO flywheel) {
    mHood = hood;
    mFlywheel = flywheel;

    mFlywheelCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(25).per(Second),
                Volts.of(15),
                null,
                (state) -> SmartDashboard.putString("FlywheelSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> mFlywheel.setTargetTorqueCurrent(voltage.in(Volts)), null, this));

    mHood.resetEncoder(kMaxAngleRevs);
    setDefaultCommand(stowAndCoast());
  }

  @Override
  public void periodic() {
    // Update all hardware
    mHood.periodic();
    mFlywheel.periodic();

    // Calculate filtered pivot current
    mHoodCurrentFilterValue = Math.abs(mHoodCurrentFilter.calculate(mHood.getStatorCurrentAmps()));

    // Update Debouncers
    isHoldingSetpoint();

    // Update tunables
    if (kFlywheelKs.hasChanged(hashCode())
        || kFlywheelKv.hasChanged(hashCode())
        || kFlywheelKa.hasChanged(hashCode())
        || kFlywheelKp.hasChanged(hashCode())
        || kFlywheelKd.hasChanged(hashCode())) {
      mFlywheel.setGains(
          kFlywheelKs.get(),
          0.0,
          kFlywheelKv.get(),
          kFlywheelKa.get(),
          kFlywheelKp.get(),
          kFlywheelKd.get());
    }

    if (kHoodKs.hasChanged(hashCode())
        || kHoodKv.hasChanged(hashCode())
        || kHoodKa.hasChanged(hashCode())
        || kHoodKp.hasChanged(hashCode())
        || kHoodKd.hasChanged(hashCode())) {
      mHood.setGains(
          kHoodKs.get(), 0.0, kHoodKv.get(), kHoodKa.get(), kHoodKp.get(), kHoodKd.get());
    }

    if (kHoodCruiseVelocity.hasChanged(hashCode()) || kHoodAcceleration.hasChanged(hashCode())) {
      mHood.setProfilingConstraints(kHoodCruiseVelocity.get(), kHoodAcceleration.get());
    }
  }

  // * ~~~~~~~~ GETTERS/SETTERS ~~~~~~~~

  @Logged(name = "Is Homed (bool)", importance = Importance.INFO)
  public boolean isHomed() {
    return mIsHomed;
  }

  @Logged(name = "Is Holding Setpoint (bool)", importance = Importance.INFO)
  public boolean isHoldingSetpoint() {
    return isHoldingAngle() && isHoldingSpeed();
  }

  @Logged(name = "Is Holding Angle (bool)", importance = Importance.INFO)
  public boolean isHoldingAngle() {
    return mIsNearAngle.calculate(
        MathUtil.isNear(
            getTargetRotation2d().getDegrees(),
            getRotation2d().getDegrees(),
            kHoodToleranceDeg.get()));
  }

  @Logged(name = "Is Holding Speed (bool)", importance = Importance.INFO)
  public boolean isHoldingSpeed() {
    return mIsNearSpeed.calculate(
        MathUtil.isNear(
            getTargetMuzzleVelocityMps(),
            getApproximatedMuzzleVelocityMps(),
            kFlywheelToleranceMetersPerSec.get()));
  }

  @Logged(name = "Target Rotation2d", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return mTargetAngle;
  }

  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(mHood.getPositionRevs());
  }

  @Logged(name = "Target Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getTargetMuzzleVelocityMps() {
    return mTargetMuzzleVelocityMps;
  }

  @Logged(name = "Approximated Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getApproximatedMuzzleVelocityMps() {
    return flywheelVelocityRpsToMuzzleVelocityMps(mFlywheel.getVelocityRevsPerSec());
  }

  public void setHoodAngle(Rotation2d angle) {
    System.out.println(angle.getDegrees());
    mTargetAngle =
        Rotation2d.fromRotations(
            MathUtil.clamp(angle.getRotations(), kMinAngleRevs, kMaxAngleRevs));

    mHood.setTargetPosition(mTargetAngle.getRotations());
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  public Command runCurrentHoming() {
    return Commands.sequence(
        this.run(() -> mHood.setTargetVoltage(2))
            .until(() -> mHoodCurrentFilterValue > kHoodCurrentZeroingThreshold),
        this.runOnce(
            () -> {
              mHood.resetEncoder(kMaxAngleRevs);
              mIsHomed = true;
            }),
        Commands.print("Hood Homed"));
  }

  public Command stowAndCoast() {
    return this.run(
        () -> {
          setHoodAngle(Rotation2d.fromRotations(kMaxAngleRevs));

          mFlywheel.enableBrake(false);
          mFlywheel.stop();
        });
  }

  public Command stowAndDeaccel() {
    return this.run(
        () -> {
          setHoodAngle(Rotation2d.fromRotations(kMaxAngleRevs));

          mFlywheel.setTargetVelocity(0.0);
        });
  }

  public Command runSetpoint(Supplier<Rotation2d> angle, DoubleSupplier muzzleVelocityMps) {
    return this.run(
        () -> {
          setHoodAngle(angle.get());

          mTargetMuzzleVelocityMps = muzzleVelocityMps.getAsDouble();
          mFlywheel.setTargetVelocity(
              muzzleVelocityMpsToFlywheelVelocityRps(muzzleVelocityMps.getAsDouble()));
        });
  }

  public Command runSetpoint(Supplier<ProjectileParameters> parameters) {
    return runSetpoint(
        () -> Rotation2d.fromRadians(parameters.get().pitch()), () -> parameters.get().velocity());
  }
}
