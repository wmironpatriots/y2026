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
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.frc6423.lib.io.DIO;
import org.frc6423.lib.io.DIORio;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxSim;
import org.frc6423.lib.sim.FlywheelSim;
import org.frc6423.lib.sim.PivotMechSim;
import org.frc6423.lib.util.NetworkTableUtil;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/**
 * {@link SubsystemBase} extension representing the intake subsystem
 *
 * <p>A {@link Intake} has 3 components: the rollers, the pivot holding the rollers, and the passive
 * kicker mechanism that assists with intaking
 */
public class Intake extends SubsystemBase {
  /** {@link Intake} subsystem constants */
  public class Constants {
    // * PHYSICAL CONSTANTS
    public static final Distance kIntakeArmLength = Inches.of(1); // TODO check

    public static final MomentOfInertia kIntakeArmRotationalInertia =
        KilogramSquareMeters.of(0.0004); // TODO check

    // * CONTROL CONSTANTS
    /** {@link Angle} representing the lower limit on intake angular position */
    public static final Angle kMinAngle = Radians.of(0.0);

    /** {@link Angle} representing the deploy intake angular position */
    public static final Angle kDeployedAngle = Radians.of(0.02);

    /** {@link Angle} representing the stowed intake angular position */
    public static final Angle kStowedAngle = Radians.of(0.32373);

    /** {@link Angle} representing the higher limit on intake angular position */
    public static final Angle kMaxAngle = Rotations.of(0.353516);

    /** {@link Angle} representing the maximum allowed error for intake angular position */
    public static final Angle kEpsilon = Degrees.of(0.5);

    /** {@link Voltage} representing the intake speed when stowed */
    public static final Voltage kStowedSpeed = Volts.zero();

    /** {@link Voltage} representing the intake speed when deploy & intaking */
    public static final Voltage kIntakingSpeed = Volts.of(3.0);

    /** {@link Voltage} representing the outake speed when deployed & outaking */
    public static final Voltage kOutakingSpeed = Volts.of(-4.0);

    // * ENCODER HARDWARE CONSTANTS
    /** {@link Integer} representing the CAN ID of encoder */
    public static final int kEncoderCanDeviceId = Matrix.kIntakeEncoderId;

    /** {@link Angle} representing the angular position offset of encoder */
    public static final Angle kEncoderAngularOffset = Rotations.of(0.413330078125);

    /** {@link CANcoderConfiguration} representing the hardware configuration of the encoder */
    public static final CANcoderConfiguration kEncoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(kEncoderAngularOffset)
                    .withAbsoluteSensorDiscontinuityPoint(
                        MathUtil.inputModulus(
                                kMaxAngle.plus(kMinAngle).in(Rotations), kMinAngle.in(Rotations), 1)
                            / 2.0));

    // * PIVOT HARDWARE CONSTANTS
    /** {@link Integer} representing the CAN ID of pivot servo */
    public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

    /** {@link Current} representing the stator current limit of pivot servo */
    public static final Current kPivotStatorCurrentLimit = Amps.of(40.0);

    /** {@link Double} representing the gear ratio between the pivot servo roto and the encoder */
    public static final double kPivotRotorToSensorRatio = (5.0 / 1.0) * (3.0 / 1.0) * (1.0 / 1.0);

    /** {@link Double} representing the gear ratio between the encoder and the mechanism */
    public static final double kPivotSensorToMechRatio = (36.0 / 16.0);

    /** {@link TrapezoidProfile} representing the motion profiled used to move to target angles */
    public static final TrapezoidProfile kPivotMotionProfiled =
        new TrapezoidProfile(new Constraints(0.25, 3));

    /** {@link Current} representing the gain acting against static friction */
    public static final Current kS = Amps.zero();

    /** {@link Current} representing the gain acting against gravity */
    public static final Current kG = Amps.zero();

    /** {@link Angle} representing the offset of the zero degree mark of the pivot */
    public static final Rotation2d kGravityArmPositionOffset = Rotation2d.fromDegrees(31);

    /** {@link Current} representing the gain for inducing an acceleration */
    public static final Current kA = Amps.zero();

    /** {@link Double} representing the gain driving the error to zero */
    public static final double kP = 0.0;

    /** {@link Double} representing the gain driving the deriviate of the error to zero */
    public static final double kD = 0.0;

    /** {@link TalonFXConfiguration} representing the hardware configuration of the pivot servo */
    public static final TalonFXConfiguration kPivotTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(kPivotStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
                    .withRotorToSensorRatio(kPivotRotorToSensorRatio)
                    .withSensorToMechanismRatio(kPivotSensorToMechRatio))
            .withSlot0(new Slot0Configs().withKP(kP).withKD(kD));

    /**
     * {@link Current} representing the rate to increase the current output of pivot servo during
     * static characterization
     */
    public static final Current kPivotStaticCharacterizationRampRate = Amps.of(0.2);

    /**
     * {@link AngularVelocity} representing the maximum allowed velocity before ending pivot static
     * characterization
     */
    public static final AngularVelocity kPivotCharacterizationVelocityThreshold =
        RadiansPerSecond.of(6);

    // * ROLLER HARDWARE CONSTANTS
    /** {@link Integer} representing the CAN ID of pivot servo */
    public static final int kRollerCanDeviceId = Matrix.kIntakeRollerId;

    /** {@link Current} representing the stator current limit of pivot servo */
    public static final Current kRollerStatorCurrentLimit = Amps.of(20.0);

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
                    .withStatorCurrentLimit(kRollerStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true));

    /** {@link CANBus} representing the bus CAN devices are on */
    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;
  }

  /**
   * Create new {@link Intake}
   *
   * @param coastOverride {@link BooleanSupplier} supplying the status of coast mode
   * @return {@link Intake}
   */
  public static Intake create(BooleanSupplier coastOverride) {
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
              Constants.kRollerTalonConfig),
          new DIORio(1), // TODO replace placeholder
          coastOverride);
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
                      DCMotor.getKrakenX60Foc(1), 1.0, KilogramSquareMeters.of(1)))),
          new DIORio(1), // TODO replace placeholder
          coastOverride);
    }
  }

  @Logged private final EncoderIO mEncoder;
  @Logged private final ServoIO mPivot, mRoller;
  @Logged private final DIO mBeambreak;

  private final BooleanSupplier mCoastOverride;

  /** {@link Angle} representing the anglular position pivot wants to go to */
  private Angle mTargetAngle = Degrees.of(0.0);

  /** {@link State} representing the pivot setpoint */
  private State mSetpointProfileState = new State(0.0, 0.0);

  /**
   * Create new {@link Intake}
   *
   * @param pivot {@link ServoIO} representing pivot servo
   * @param encoder {@link EncoderIO} representing pivot abs encoder
   * @param roller {@link ServoIO} representing roller servo
   * @param beambreak {@link DIO} representing beambreak
   * @param coastOverride {@link BooleanSupplier} supplying the status of coast mode
   */
  protected Intake(
      ServoIO pivot,
      EncoderIO encoder,
      ServoIO roller,
      DIO beambreak,
      BooleanSupplier coastOverride) {
    mPivot = pivot;
    mEncoder = encoder;
    mRoller = roller;
    mBeambreak = beambreak;
    mCoastOverride = coastOverride;

    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData(
          "Characterization/Intake/RunStaticCharacterization", runStaticCharacterizationRoutine());
    }
  }

  @Override
  public void periodic() {
    // Update hardware
    mPivot.periodic();
    mEncoder.periodic();
    mRoller.periodic();

    boolean shouldRun = DriverStation.isEnabled() && !mCoastOverride.getAsBoolean();

    // Calculate & apply next setpoint
    if (shouldRun) {
      var targetState = new State(mTargetAngle.in(Rotations), 0.0);
      mSetpointProfileState =
          Constants.kPivotMotionProfiled.calculate(0, getProfileState(), targetState);

      var feedforward =
          Constants.kS
              .times(Math.signum(mPivot.getAngularVelocity().baseUnitMagnitude()))
              .plus(
                  Constants.kG.times(
                      getRotation2d().plus(Constants.kGravityArmPositionOffset).getCos()))
              .plus(
                  Constants.kA.times(
                      mPivot.getAngularAcceleration().in(RotationsPerSecondPerSecond)));

      if (MathUtil.isNear(
          mSetpointProfileState.position,
          getRotation2d().getRotations(),
          Constants.kEpsilon.in(Rotations))) {
        mPivot.setTorquePositionSetpoint(Rotations.of(mSetpointProfileState.position), feedforward);
      } else {
        mPivot.stop();
      }
    }
  }

  // * GETTERS
  /**
   * @return {@link Rotation2d} representing the angular position subsystem wants to go to
   */
  @Logged(name = "Pivot Target Rotation2d", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return new Rotation2d(mTargetAngle);
  }

  /**
   * @return {@link Rotation2d} representing the angular position of pivot
   */
  @Logged(name = "Pivot Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return new Rotation2d(mEncoder.getAngle());
  }

  /**
   * @return {@link State} representing the trapezoid profile setpoint state of the pivot
   */
  @Logged(name = "Pivot Setpoint Profile State")
  public State getSetpointProfileState() {
    return mSetpointProfileState;
  }

  /**
   * @return {@link State} representing the trapezoid profile state of the pivot
   */
  @Logged(name = "Pivot Profile State")
  public State getProfileState() {
    return new State(
        mPivot.getAngle().in(Rotations), mPivot.getAngularVelocity().in(RotationsPerSecond));
  }

  /**
   * @return true when the angular position error of pivot is less than epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearTarget() {
    return MathUtil.isNear(
        getTargetRotation2d().getRotations(),
        getRotation2d().getRotations(),
        Constants.kEpsilon.in(Rotations));
  }

  // * SETTERS
  /**
   * Set new intake setpoint
   *
   * @param setpoint {@link Angle} representing target angular postiion
   * @param speed {@link Voltage} representing desired speed
   */
  private void setSetpoint(Angle setpoint, Voltage speed) {
    mTargetAngle =
        Rotations.of(
            MathUtil.clamp(
                setpoint.in(Rotations),
                Constants.kMinAngle.in(Rotations),
                Constants.kMaxAngle.in(Rotations)));

    mRoller.setVoltageSetpoint(speed, true);
  }

  // * COMMANDS
  /**
   * Attempt to stow intake
   *
   * @return {@link Command}
   */
  public Command stow() {
    return this.run(() -> setSetpoint(Constants.kStowedAngle, Constants.kIntakingSpeed))
        .until(this::isNearTarget)
        .andThen(() -> setSetpoint(Constants.kStowedAngle, Constants.kStowedSpeed));
  }

  /**
   * Attempt to deploy and start intaking
   *
   * @return {@link Command}
   */
  public Command intake() {
    return this.run(() -> setSetpoint(Constants.kMaxAngle, Constants.kIntakingSpeed));
  }

  /**
   * Attempt to deploy and start outaking
   *
   * @return {@link Command}
   */
  public Command outake() {
    return this.run(() -> setSetpoint(Constants.kDeployedAngle, Constants.kOutakingSpeed));
  }

  /**
   * Run a characterization routine for determining the static friction gain (kS)
   *
   * <p>Torque current of pivot will ramp up at a constant rate until it moves
   *
   * <p>Characterization outputs will be located in the 'Characterization/Intake' NT folder
   *
   * @return {@link Command}
   */
  public Command runStaticCharacterizationRoutine() {
    Timer timer = new Timer();
    var entry = NetworkTableUtil.createEntry("Characterization/Intake/PivotStaticAmps", 0.0);

    return this.startRun(
            () -> timer.reset(),
            () -> {
              var output = Constants.kPivotStaticCharacterizationRampRate.times(timer.get());

              mPivot.setTorqueCurrentSetpoint(output);
              entry.accept(output.in(Amps));
            })
        .until(
            () -> mPivot.getAngularVelocity().gt(Constants.kPivotCharacterizationVelocityThreshold))
        .andThen(() -> mPivot.stop())
        .andThen(this.idle())
        .finallyDo(
            () -> {
              timer.stop();
            });
  }

  // TODO
  public Command runAccelerationCharacterization(Current staticGain) {
    return Commands.none();
  }
}
