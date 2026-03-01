// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Second;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.frc6423.lib.io.DIO;
import org.frc6423.lib.io.DIONone;
import org.frc6423.lib.io.DIORio;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.EncoderIOCanCoderSim;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/**
 * {@link SubsystemBase} Intake Subsystem
 *
 * <p>Angular Position of subsystem is controlled by Kraken x60 Pivot
 *
 * <p>Angular Position of subsystem is measured by Throughbore CANcoder
 *
 * <p>Speed voltage of subsystem is controlled by Kraken x60 Roller
 */
public class Intake extends SubsystemBase {
  /** Constants for the {@link Intake} */
  public class Constants {
    // * PHYSICAL CONSTANTS
    /** {@link Distance} 'Length' of Pivot System */
    public static final Distance kLength = Meters.of(0.5); // TODO

    /** {@link MomentOfInertia} Rotational Inertia of Pivot System */
    public static final MomentOfInertia kRotationalInertia =
        KilogramSquareMeters.of(402.290096 * 0.0002926397);

    // * CONTROL CONSTANTS
    // TODO check these values
    /** {@link Angle} Max allowed angular position error in subsystem */
    public static final Angle kEpsilon = Degrees.of(1.5);

    /** {@link Angle} Lower limit on angular position */
    public static final Angle kMinAngle = Degrees.of(90.0);

    /** {@link Angle} upper limit on angular position */
    public static final Angle kMaxAngle = Degrees.of(150);

    /** {@link Angle} Angular position when stowed */
    public static final Angle kStowedAngle = kMinAngle.plus(Degrees.of(2.5));

    /** {@link Angle} Angular postiion when deployed */
    public static final Angle kDeployedAngle = kMaxAngle.minus(Degrees.of(0.5));

    /** {@link AngularVelocity} The maximum allowed angular velocity of pivot */
    public static final AngularVelocity kPivotMaxVelocity = DegreesPerSecond.of(1.0);

    /** {@link AngularAcceleration} The maximum allowed angular acceleration of pivot */
    public static final AngularAcceleration kPivotMaxAcceleration =
        DegreesPerSecondPerSecond.of(5.0);

    /** {@link Voltage} Speed while idling */
    public static final Voltage kStowedSpeed = Volts.of(0.0);

    /** {@link Voltage} Speed when intaking */
    public static final Voltage kIntakingSpeed = Volts.of(5.0);

    /** {@link Voltage} Speed when outaking */
    public static final Voltage kOutakingSpeed = Volts.of(-7.0);

    // * HARDWARE CONSTANTS
    /** {@link Integer} CAN ID of abs encoder */
    public static final int kEncoderCanDeviceId = Matrix.kIntakeEncoderId;

    /** {@link Angle} Angular Offset to the stowed angle of abs encoder */
    public static final Angle kEncoderAngularOffset = Revolutions.of(0.0).plus(kMinAngle); // TODO

    /** {@link Angle} Angular Position in the middle of the 'unreachable' area of pivot */
    public static final Angle kEncoderSensorDiscontinuityPoint =
        Degrees.of(360).minus(kMaxAngle.minus(kMinAngle)).div(2).plus(kMaxAngle);

    /** {@link CANcoderConfiguration} Hardware config of abs encoder */
    public static final CANcoderConfiguration kEncoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(kEncoderAngularOffset)
                    .withAbsoluteSensorDiscontinuityPoint(kEncoderSensorDiscontinuityPoint));

    /** {@link CANBus} CAN bus devices are on */
    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    /** {@link Integer} CAN ID of pivot servo */
    public static final int kPivotCanDeviceId = Matrix.kIntakePivotId;

    /** {@link Current} Stator current limit of pivot servo */
    public static final Current kPivotStatorCurrentLimit = Amps.of(40.0);

    /** {@link Double} Gear ratio between the pivot servo rotor and the abs encoder shaft */
    public static final double kRotorToSensorRatio = (5.0 / 1.0) * (3.0 / 1.0) * (1.0 / 1.0);

    /** {@link Double} Gear ratio between the abs encoder shaft and the mechanism pivot */
    public static final double kSensorToMechRatio = (36.0 / 16.0);

    /** {@link TalonFXConfiguration} Hardware config of pivot servo */
    public static final TalonFXConfiguration kPivotTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(kPivotStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
                    .withRotorToSensorRatio(kRotorToSensorRatio)
                    .withSensorToMechanismRatio(kSensorToMechRatio))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kPivotMaxVelocity)
                    .withMotionMagicAcceleration(kPivotMaxAcceleration))
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.0)
                    .withKG(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKP(0.0)
                    .withKD(0.0)); // TODO Torque Current Control Gains (accelerating)

    /** {@link Integer} CAN ID of servo */
    public static final int kRollerCanDeviceId = Matrix.kIntakeRollerId;

    /** {@link Current} Stator current limit of servo */
    public static final Current kRollerStatorCurrentLimit = Amps.of(20.0);

    /** {@link TalonFXConfiguration} Hardware config of roller servo */
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

    /** {@link Integer} The DIO port beambreak is plugged into */
    public static final int kBeamBreakDioPort = Matrix.kIntakeBeamBreakDio;

    // * CHARACTERIZATON
    /**
     * {@link Velocity} of {@link CurrentUnit} Rate at which current output ramps up at in
     * Quasistatic Characterization
     */
    public static final Velocity<CurrentUnit> kCharacterizationRampRate = Amps.of(15.0).per(Second);

    /** {@link Current} Current step size for Dynamic Characterization */
    public static final Current kCharacterizationStepSize = Amps.of(6.0);
  }

  /**
   * Create new {@link Intake}
   *
   * @return {@link Intake}
   */
  public static Intake create() {
    return (Robot.isReal())
        ? new Intake(
            new ServoIOTalonFx(
                "Pivot",
                Constants.kCanBus,
                Constants.kPivotCanDeviceId,
                Constants.kPivotTalonConfig),
            new EncoderIOCanCoder(
                Constants.kEncoderCanDeviceId, Constants.kCanBus, Constants.kEncoderConfig),
            new ServoIOTalonFx(
                "Roller",
                Constants.kCanBus,
                Constants.kRollerCanDeviceId,
                Constants.kRollerTalonConfig),
            new DIORio(Constants.kBeamBreakDioPort))
        : new Intake(
            new ServoIOTalonFxPivotSim(
                "Pivot",
                Constants.kCanBus,
                Constants.kPivotCanDeviceId,
                Constants.kPivotTalonConfig,
                Constants.kRotationalInertia,
                Constants.kLength,
                Constants.kMinAngle,
                Constants.kMaxAngle,
                Constants.kMinAngle,
                true,
                MotorType.KrakenX60,
                DCMotor.getKrakenX60Foc(1),
                Constants.kSensorToMechRatio),
            new EncoderIOCanCoderSim(
                Constants.kEncoderCanDeviceId, Constants.kCanBus, Constants.kEncoderConfig),
            new ServoIONone("Roller"),
            new DIONone());
  }

  @Logged private final EncoderIO mEncoder;
  @Logged private final ServoIO mPivot, mRoller;
  @Logged private final DIO mBeambreak;

  private final SysIdRoutine mSysIdRoutine;

  private Angle mTargetAngle = Degrees.zero();

  /**
   * Create new {@link Intake}
   *
   * @param pivot {@link ServoIO} Pivot servo rotating subsystem
   * @param encoder {@link EncoderIO} ABS Encoder measuring subsystem rotation
   * @param roller {@link ServoIO} Roller servo running subsystem speed
   * @param beambreak {@link DIO} Beambreak
   */
  protected Intake(ServoIO pivot, EncoderIO encoder, ServoIO roller, DIO beambreak) {
    mPivot = pivot;
    mEncoder = encoder;
    mRoller = roller;
    mBeambreak = beambreak;

    // Init SysId
    mSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Constants.kCharacterizationRampRate.in(Amps.per(Second))).per(Second),
                Volts.of(Constants.kCharacterizationStepSize.in(Amps)),
                null,
                (state) ->
                    Epilogue.getConfig()
                        .backend
                        .log("Characterization/Hood/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> mPivot.setTorqueCurrentSetpoint(Amps.of(voltage.in(Volts))),
                null,
                this));

    // Configure Sim
    if (mEncoder instanceof EncoderIOCanCoderSim simEncoder) {
      simEncoder.setRawAngleOverride(mPivot::getRawAngle);
    }

    // Publish characterization command in tuning mode
    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData("Run Intake SysId Characterization", runCharacterizationSequence());
    }
  }

  @Override
  public void periodic() {
    // Update hardware
    mPivot.periodic();
    mEncoder.periodic();
    mRoller.periodic();
  }

  // * GETTERS
  /**
   * Get Target Angular Position of subsystem
   *
   * @return {@link Angle}
   */
  @Logged(name = "Target Angular Position (rads)", importance = Importance.INFO)
  public Angle getTargetAngle() {
    return mTargetAngle;
  }

  /**
   * Get Angular Position of subsystem
   *
   * @return {@link Angle}
   */
  @Logged(name = "Angular Position (rads)", importance = Importance.INFO)
  public Angle getAngle() {
    return mEncoder.getAngle();
  }

  /**
   * Get Angular Velocity of subsystem
   *
   * @return {@link AngularVelocity}
   */
  @Logged(name = "Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return mPivot.getAngularVelocity();
  }

  /**
   * Get Angular Acceleration of subsystem
   *
   * @return {@link AngularAcceleration}
   */
  @Logged(name = "Angular Acceleration (rads per second per second)", importance = Importance.INFO)
  public AngularAcceleration getAngularAcceleration() {
    return mPivot.getAngularAcceleration();
  }

  /**
   * Check if subsystem is nearly at setpoint angular position
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Near Setpoint (bool)", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return MathUtil.isNear(
        mTargetAngle.in(Revolutions),
        getAngle().in(Revolutions),
        Constants.kEpsilon.in(Revolutions));
  }

  // * COMMANDS
  /**
   * Run SysId Characterization Routine for determining gains
   *
   * <p>Tests will run as follows: +Quasi, -Quasi, +Dyna, -Dyna
   *
   * <p>Each test will stop 5 degrees before the upper/lower angular position limits
   *
   * @return {@link Command}
   */
  public Command runCharacterizationSequence() {
    return Commands.sequence(
            mSysIdRoutine
                .quasistatic(Direction.kForward)
                .until(() -> getAngle().gte(Constants.kMaxAngle.minus(Degrees.of(5)))),
            Commands.waitSeconds(2),
            mSysIdRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> getAngle().gte(Constants.kMinAngle.plus(Degrees.of(5)))),
            Commands.waitSeconds(2),
            mSysIdRoutine
                .dynamic(Direction.kForward)
                .until(() -> getAngle().gte(Constants.kMaxAngle.minus(Degrees.of(5)))),
            Commands.waitSeconds(2),
            mSysIdRoutine
                .dynamic(Direction.kReverse)
                .until(() -> getAngle().gte(Constants.kMinAngle.plus(Degrees.of(5)))))
        .beforeStarting(() -> mTargetAngle = Revolutions.zero(), this)
        .withName("Intake Characterization");
  }

  /**
   * Request subsystem to stow (aka fold completely)
   *
   * @return {@link Command}
   */
  public Command stow() {
    return runSetpoint(Constants.kStowedAngle, Constants.kStowedSpeed).withName("Intake Stow");
  }

  /**
   * Request subsystem to intake (fold inwards and roll inwards)
   *
   * @return {@link Command}
   */
  public Command intake() {
    return runSetpoint(Constants.kDeployedAngle, Constants.kIntakingSpeed)
        .withName("Intake Deploy Intake");
  }

  /**
   * Request subsystem to outake (fold outwards and roll outwards)
   *
   * @return {@link Command}
   */
  public Command outake() {
    return runSetpoint(Constants.kDeployedAngle, Constants.kOutakingSpeed)
        .withName("Intake Deploy Outake");
  }

  /**
   * Request subsystem to follow specified angular position and speed setpoints
   *
   * @param angle {@link Angle} Angular position setpoint
   * @param speed {@link Voltage} Speed voltage
   * @return {@link Command}
   */
  private Command runSetpoint(Angle angle, Voltage speed) {
    return this.run(
        () -> {
          mTargetAngle =
              Revolutions.of(
                  MathUtil.clamp(
                      angle.in(Revolutions),
                      Constants.kMinAngle.in(Revolutions),
                      Constants.kMaxAngle.in(Revolutions)));
          mPivot.setTorqueMotionProfiledPositionSetpoint(mTargetAngle);
        });
  }
}
