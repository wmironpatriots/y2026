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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.RollerIO;
import org.frc6423.lib.io.RollerIONone;
import org.frc6423.lib.io.RollerIOTalonFx;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Static Factory for automatically configuring and creating a {@link IntakeSubsystem}
   *
   * @return {@link IntakeSubsystem}
   */
  public static IntakeSubsystem create() {
    return (Robot.isReal())
        ? new IntakeSubsystem(
            new RollerIOTalonFx(kRollerCanDeviceId, kCanBus, kRollerTalonFxConfig),
            new PivotIOReal(kPivotCanDeviceId, kCanBus, kPivotTalonFxConfig))
        : new IntakeSubsystem(
            new RollerIONone(),
            new PivotIOReal(
                kPivotCanDeviceId, kCanBus, kPivotTalonFxConfig)); // TODO - replace with sim
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

  public static final int kRollerCanDeviceId = Matrix.kIntakeRollerId;

  public static final double kPivotSensorToMechRatio =
      (5.0 / 1.0) * (3.0 / 1.0) * (1.0 / 1.0) * (36.0 / 16.0);

  public static final TalonFXConfiguration kPivotTalonFxConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(20.0)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(kPivotSensorToMechRatio))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(2)
                  .withMotionMagicAcceleration(3));

  public static final TalonFXConfiguration kRollerTalonFxConfig =
      RollerIOTalonFx.createGenericRollerConfig(true)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60.0) // OOOOO SHIVERS OOOOO :cold_face:
                  .withStatorCurrentLimitEnable(true));

  public static final double kMinAngleRevs = Units.degreesToRotations(0.0);

  public static final double kMaxAngleRevs = Units.degreesToRotations(56.425800397);

  public static final double kPivotCurrentZeroingThreshold = 10.0;

  public static final double kRotationalInertiaKgSquaredMeters = 402.290096 * 0.0002926397;

  public static final double kArmLengthMeters = 0.5;

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  public static final TunableNumber kPositionKs = new TunableNumber("Intake/Pivot/kS");
  public static final TunableNumber kPositionKg = new TunableNumber("Intake/Pivot/kG");
  public static final TunableNumber kPositionKv = new TunableNumber("Intake/Pivot/kV");
  public static final TunableNumber kPositionKa = new TunableNumber("Intake/Pivot/kA");
  public static final TunableNumber kPositionKp = new TunableNumber("Intake/Pivot/kP");
  public static final TunableNumber kPositionKd = new TunableNumber("Intake/Pivot/kD");

  public static final TunableNumber kPositionToleranceDeg =
      new TunableNumber("Intake/Pivot/Tolerance (degrees)");
  public static final TunableNumber kPositionStowedDeg =
      new TunableNumber("Intake/Pivot/Stowed (degrees)");
  public static final TunableNumber kPositionDeployedDeg =
      new TunableNumber("Intake/Pivot/Deployed (degrees)");
  public static final TunableNumber kPositionAgitatingDeg =
      new TunableNumber("Intake/Pivot/Agitating (degrees)");

  public static final TunableNumber kStowedSpeedVolts =
      new TunableNumber("Intake/Stowed Speed (volts)");
  public static final TunableNumber kStowingSpeedVolts =
      new TunableNumber("Intake/Stowing Speed (volts)");
  public static final TunableNumber kIntakingSpeedVolts =
      new TunableNumber("Intake/Intaking Speed (volts)");
  public static final TunableNumber kOutakingSpeedVolts =
      new TunableNumber("Intake/Outaking Speed (volts)");
  public static final TunableNumber kAgitatingPeriod = new TunableNumber("Intake/Agitating Period");

  static {
    kPositionKs.initDefault(0.0);
    kPositionKg.initDefault(0.0);
    kPositionKv.initDefault(0.0);
    kPositionKa.initDefault(0.0);
    kPositionKp.initDefault(250.0);
    kPositionKd.initDefault(30.0);

    kPositionToleranceDeg.initDefault(6.5);
    kPositionStowedDeg.initDefault(Units.rotationsToDegrees(kMinAngleRevs));
    kPositionDeployedDeg.initDefault(Units.rotationsToDegrees(kMaxAngleRevs));
    kAgitatingPeriod.initDefault(0.25);

    kStowedSpeedVolts.initDefault(0.0);
    kStowingSpeedVolts.initDefault(4.5);
    kIntakingSpeedVolts.initDefault(9.0);
    kOutakingSpeedVolts.initDefault(-9.0);
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final RollerIO mRoller;
  @Logged private final PivotIO mPivot;

  private boolean mIsHomed = false;

  private LinearFilter mCurrentFilter = LinearFilter.movingAverage(5);
  private double mFilteredCurrent = 0.0;

  private Rotation2d mTarget = Rotation2d.fromRotations(kMinAngleRevs);

  protected IntakeSubsystem(RollerIO roller, PivotIO pivot) {
    mRoller = roller;
    mPivot = pivot;

    setDefaultCommand(runCurrentHoming().unless(this::isHomed).andThen(stow()));
  }

  @Override
  public void periodic() {
    // Update hardware
    mPivot.periodic();
    mRoller.periodic();

    // Calculate filtered current
    mFilteredCurrent = mCurrentFilter.calculate(mPivot.getStatorCurrentAmps());

    // Update tunables
    if (kPositionKs.hasChanged(hashCode())
        || kPositionKg.hasChanged(hashCode())
        || kPositionKv.hasChanged(hashCode())
        || kPositionKa.hasChanged(hashCode())
        || kPositionKp.hasChanged(hashCode())
        || kPositionKd.hasChanged(hashCode())) {
      mPivot.setGains(
          kPositionKs.get(),
          kPositionKg.get(),
          kPositionKv.get(),
          kPositionKa.get(),
          kPositionKp.get(),
          kPositionKd.get());
    }
  }

  // * ~~~~~~~~ GETTERS/SETTERS ~~~~~~~~

  @Logged(name = "Is Homed (bool)", importance = Importance.INFO)
  public boolean isHomed() {
    return mIsHomed;
  }

  @Logged(name = "Is Near Pivot/(bool)", importance = Importance.INFO)
  public boolean isNearPosition() {
    return MathUtil.isNear(
        getTargetRotation2d().getDegrees(),
        getRotation2d().getDegrees(),
        kPositionToleranceDeg.get());
  }

  @Logged(name = "Target Rotation2d", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return mTarget;
  }

  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(mPivot.getPositionRevs());
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Run pivot backwards at constant voltage until pushing against hardstop, then zero
   *
   * @return {@link Command}
   */
  public Command runCurrentHoming() {
    return Commands.sequence(
        this.run(() -> mPivot.setTargetVoltage(-2))
            .until(() -> mFilteredCurrent > kPivotCurrentZeroingThreshold),
        this.runOnce(
            () -> {
              mPivot.resetEncoder(kMinAngleRevs);
              mIsHomed = true;
            }),
        Commands.print("Intake Homed"));
  }

  /**
   * Stow and stop once completely folded
   *
   * @return {@link Command}
   */
  public Command stow() {
    return this.run(
        () -> {
          mTarget = Rotation2d.fromDegrees(kPositionStowedDeg.get());
          mPivot.setTargetPosition(mTarget.getRotations());

          if (isNearPosition()) {
            mRoller.setVoltageOutput(kStowedSpeedVolts.get());
          } else {
            mRoller.setVoltageOutput(kStowingSpeedVolts.get());
          }
        });
  }

  /**
   * Intake while ocilating between {@link #kPositionDeployedDeg} and {@link #kPositionAgitatingDeg}
   *
   * @return {@link Command}
   */
  public Command intakeAgitated() {
    return this.run(
        () -> {
          mTarget =
              Rotation2d.fromDegrees(
                  kPositionDeployedDeg.getAsDouble()
                      - Math.abs(
                          kAgitatingPeriod.get()
                              * Math.sin(Timer.getTimestamp())
                              * kPositionAgitatingDeg.get()));
          mPivot.setTargetPosition(mTarget.getRotations());
          mRoller.setVoltageOutput(kIntakingSpeedVolts.get());
        });
  }

  /**
   * Rest in deployed position
   *
   * @return {@link Command}
   */
  public Command rest() {
    return this.run(
        () -> {
          mTarget = Rotation2d.fromDegrees(kPositionDeployedDeg.get());
          mPivot.setTargetPosition(mTarget.getRotations());
          mRoller.stop();
        });
  }

  /**
   * Deploy and run inwards
   *
   * @return {@link Command}
   */
  public Command intake() {
    return this.run(
        () -> {
          mTarget = Rotation2d.fromDegrees(kPositionDeployedDeg.getAsDouble());
          mPivot.setTargetPosition(mTarget.getRotations());
          mRoller.setVoltageOutput(kIntakingSpeedVolts.get());
        });
  }

  /**
   * Outake while ocilating between {@link #kPositionDeployedDeg} and {@link #kPositionAgitatingDeg}
   *
   * @return {@link Command}
   */
  public Command outakeAgitated() {
    return this.run(
        () -> {
          mTarget =
              Rotation2d.fromDegrees(
                  kPositionDeployedDeg.getAsDouble()
                      - Math.abs(
                          kAgitatingPeriod.get()
                              * Math.sin(Timer.getTimestamp())
                              * kPositionAgitatingDeg.get()));
          mPivot.setTargetPosition(mTarget.getRotations());
          mRoller.setVoltageOutput(kOutakingSpeedVolts.get());
        });
  }

  /**
   * Deploy and run outwards
   *
   * @return {@link Command}
   */
  public Command outake() {
    return this.run(
        () -> {
          mTarget = Rotation2d.fromDegrees(kPositionDeployedDeg.getAsDouble());
          mPivot.setTargetPosition(mTarget.getRotations());
          mRoller.setVoltageOutput(kOutakingSpeedVolts.get());
        });
  }

  /**
   * Stop subsystem completely
   *
   * @return {@link Command}
   */
  public Command stop() {
    return this.runOnce(
        () -> {
          mPivot.stop();
          mRoller.stop();
        });
  }
}
