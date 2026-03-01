// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/**
 * {@link SubsystemBase} Hood Subsystem
 *
 * <p>Subsystem for unfolding and folding the back of the shooter
 *
 * <p>Subsystem is powered by one Kraken x44
 *
 * <p>Subsystem is measured by one Throughbore CANcoder
 */
public class Hood extends SubsystemBase {
  /** Constants for the {@link Hood} */
  public class Constants {
    // * PHYSICAL CONSTANTS
    /** {@link Distance} 'Length' of Pivot System */
    public static final Distance kLength = Inches.of(7.510000);

    /** {@link MomentOfInertia} Rotational Inertia of Pivot System */
    public static final MomentOfInertia kRotationalInertia =
        KilogramSquareMeters.of(75.752248 * 0.0002926397);

    // * CONTROL CONSTANTS
    /** {@link Angle} Max allowed angular position error in subsystem */
    public static final Angle kEpsilon = Degrees.of(0.5);

    /** {@link Angle} Lower limit on angular position */
    public static final Angle kMinAngle = Degrees.of(14.703759);

    /** {@link Angle} upper limit on angular position */
    public static final Angle kMaxAngle = Degrees.of(45.812);

    /** {@link AngularVelocity} The maximum allowed angular velocity of subsystem */
    public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(35.0);

    /** {@link AngularAcceleration} The maximum allowed angular acceleration of subsystem */
    public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(15.0);

    // * HARDWARE CONSTANTS
    /** {@link Integer} CAN ID of abs encoder */
    public static final int kEncoderCanDeviceId = Matrix.kHoodEncoderId;

    /** {@link Angle} Angular Offset to the stowed angle of abs encoder */
    public static final Angle kEncoderAngularOffset = Revolutions.of(0.0); // TODO

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

    /** {@link Integer} CAN ID of servo */
    public static final int kServoCanDeviceId = Matrix.kHoodId;

    /** {@link Current} Stator current limit of servo */
    public static final Current kServoStatorCurrentLimit = Amps.of(40.0);

    /** {@link Double} Gear ratio between the servo rotor and the encoder shaft */
    public static final double kRotorToSensorRatio = 2.57142857143;

    /** {@link Double} Gear ratio between the encoder shaft and the mechanism pivot */
    public static final double kSensorToMechRatio = 10.83;

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
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(kEncoderCanDeviceId)
                    .withRotorToSensorRatio(kRotorToSensorRatio)
                    .withSensorToMechanismRatio(kSensorToMechRatio))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxVelocity)
                    .withMotionMagicAcceleration(kMaxAcceleration))
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKP(0.0)
                    .withKD(0.0)); // TODO Torque Current Control Gains (accelerating)

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
   * Create new {@link Hood}
   *
   * @return {@link Hood}
   */
  public static Hood create() {
    return (Robot.isReal())
        ? new Hood(
            new ServoIOTalonFx(
                "Servo",
                Constants.kCanBus,
                Constants.kServoCanDeviceId,
                Constants.kServoTalonConfig),
            new EncoderIOCanCoder(
                Constants.kEncoderCanDeviceId, Constants.kCanBus, Constants.kEncoderConfig))
        : new Hood(
            new ServoIOTalonFxPivotSim(
                "Servo",
                Constants.kCanBus,
                Constants.kServoCanDeviceId,
                Constants.kServoTalonConfig,
                Constants.kRotationalInertia,
                Constants.kLength,
                Constants.kMinAngle,
                Constants.kMaxAngle,
                Constants.kMinAngle,
                true,
                MotorType.KrakenX44,
                DCMotor.getKrakenX44Foc(1),
                Constants.kSensorToMechRatio * Constants.kRotorToSensorRatio),
            new EncoderIOCanCoder(
                Constants.kEncoderCanDeviceId, Constants.kCanBus, Constants.kEncoderConfig));
  }

  @Logged private final ServoIO mServo;
  @Logged private final EncoderIO mEncoder;

  private final SysIdRoutine mSysIdRoutine;

  private Angle mTargetAngle = Revolutions.zero();

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
                (voltage) -> mServo.setTorqueCurrentSetpoint(Amps.of(voltage.in(Volts))),
                null,
                this));

    // Publish characterization command in tuning mode
    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData("Run Hood SysId Characterization", runCharacterizationSequence());
    }
  }

  @Override
  public void periodic() {
    // Update all Hardware
    mServo.periodic();
    mEncoder.periodic();
  }

  // * GETTERS
  /**
   * Get Target Angular Position of subsystem
   *
   * @return {@link}
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
    return mServo.getAngle();
  }

  /**
   * Get Angular Velocity of subsystem
   *
   * @return {@link AngularVelocity}
   */
  @Logged(name = "Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return mServo.getAngularVelocity();
  }

  /**
   * Get Angular Acceleration of subsystem
   *
   * @return {@link AngularAcceleration}
   */
  @Logged(name = "Angular Acceleration (rads per second per second)", importance = Importance.INFO)
  public AngularAcceleration getAngularAcceleration() {
    return mServo.getAngularAcceleration();
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
            Commands.print("Quasi Forward"),
            mSysIdRoutine
                .quasistatic(Direction.kForward)
                .until(() -> getAngle().gt(Constants.kMaxAngle.minus(Degrees.of(10)))),
            Commands.print("Quasi Reverse"),
            mSysIdRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> getAngle().lt(Constants.kMinAngle.plus(Degrees.of(10)))),
            Commands.print("Dyna Forward"),
            mSysIdRoutine
                .dynamic(Direction.kForward)
                .until(() -> getAngle().gt(Constants.kMaxAngle.minus(Degrees.of(10)))),
            Commands.print("Dyna Reverse"),
            mSysIdRoutine
                .dynamic(Direction.kReverse)
                .until(() -> getAngle().lt(Constants.kMinAngle.plus(Degrees.of(10)))),
            Commands.print("Done!"))
        .withName("Hood Characterization");
  }

  /**
   * Request subsystem to stow (aka fold completely)
   *
   * @return {@link Command}
   */
  public Command stow() {
    return adjustToAngle(Constants.kMinAngle).withName("Hood Stow");
  }

  /**
   * Request subsystem to adjust to a desired angular position
   *
   * @param angle {@link Angle} Angular Position Setpoint
   * @return {@link Command}
   */
  public Command adjustToAngle(Angle angle) {
    return adjustToAngle(() -> angle).withName("Hood Adjust to Angle");
  }

  /**
   * Request subsystem to continiously adjust to a stream of angular position setpoints
   *
   * @param angle {@link Supplier}<{@link Angle}> Angular Position Setpoint
   * @return {@link Command}
   */
  public Command adjustToAngle(Supplier<Angle> angle) {
    return this.run(
            () -> {
              mTargetAngle =
                  Rotations.of(
                      MathUtil.clamp(
                          angle.get().in(Revolutions),
                          Constants.kMinAngle.in(Revolutions),
                          Constants.kMaxAngle.in(Revolutions)));

              mServo.setTorqueMotionProfiledPositionSetpoint(mTargetAngle);
            })
        .withName("Hood Adjust to Angle Continiously");
  }
}
