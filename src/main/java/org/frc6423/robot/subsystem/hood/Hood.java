// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.io.EncoderIO;
import org.frc6423.lib.io.EncoderIOCanCoder;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.util.NetworkTableUtil;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the hood subsystem
 *
 * <p>A {@link Hood} has a single pivoting component that folds and unfolds the back of the shooter
 * to launch fuel at different angles
 */
public class Hood extends SubsystemBase {
  /** {@link Hood} subsystem constants */
  public class Constants {
    // * CONTROLS CONSTANTS
    /** {@link Angle} representing the lower angular position limit */
    public static final Angle kMinAngle = Degrees.of(14.703759);

    /** {@link Angle} representing the higher angular position limit */
    public static final Angle kMaxAngle = Degrees.of(45.812);

    /** {@link Angle} representing the maximum allowed error in angular position */
    public static final Angle kEpsilon = Degrees.of(0.01);

    // * ENCODER HARDWARE CONSTANTS
    /** {@link Integer} representing the CAN ID of the hood encoder */
    public static final int kEncoderCanDeviceId = Matrix.kHoodEncoderId;

    /** {@link Angle} representing the angular offset of encoder */
    public static final Angle kEncoderAngularOffset = Rotations.of(0.0); // TODO

    /** {@link CANcoderConfiguration} representing the hardware config of encoder */
    public static final CANcoderConfiguration kEncoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(kEncoderAngularOffset));

    // * SERVO HARDWARE CONSTANTS
    /** {@link Integer} representing the CAN ID of servo */
    public static final int kServoCanDeviceId = Matrix.kHoodId;

    /** {@link Current} representing the stator current limit of servo */
    public static final Current kServoStatorCurrentLimit = Amps.of(20.0);

    /** {@link Double} representing the gear ratio between the servo rotor and the encoder shaft */
    public static final double kRotorToSensorRatio = 2.57142857143;

    /** {@link Double} representing the gear ratio between the encoder shaft and the hood */
    public static final double kSensorToMechRatio = 10.83;

    /** {@link TrapezoidProfile} representing the motion profile use to calculate hood setpoints */
    public static final TrapezoidProfile kServoMotionProfile =
        new TrapezoidProfile(new Constraints(3, 5));

    public static final TalonFXConfiguration kServoConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(kServoStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true));

    /** {@link Current} representing the Current gain acting against static friciton */
    public static Current kS = Amps.zero();

    /** {@link Current} representing the Current gain acting against gravity */
    public static Current kG = Amps.zero();

    /** {@link Current} representing the Current gain inducing acceleration */
    public static Current kA = Amps.zero();

    /** {@link Current} representing the Current gain driving the acceleration to zero */
    public static Current kP = Amps.zero();

    /** {@link Current} representing the Current gain driving the jerk to zero */
    public static Current kD = Amps.zero();

    /**
     * {@link Current} representing the rate to increase the current output of servo during
     * quasistatic characterization
     */
    public static final Current kQuasistaticCharacterizationRampRate = Amps.of(0.2);

    /**
     * {@link AngularVelocity} representing the maximum allowed velocity before ending quasistatic
     * characterization
     */
    public static final AngularVelocity kQuasistaticCharacterizationVelocityThreshold =
        DegreesPerSecond.of(0.2);

    /**
     * {@link Current} representing the constant current output of servo during dynamic
     * characterization
     */
    public static final Current kDynamicCharacterizationCurrent = Amps.of(2);

    /** {@link Time} representing the length of the dynamic characterization test */
    public static final Time kDynamicCharacterizationDuration = Seconds.of(2);

    /** {@link CANBus} representing bus CAN devices are on */
    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;
  }

  /**
   * Create new {@link Hood}
   *
   * @param coastOverride {@link BooleanSupplier} providing if coast mode should be enabled
   * @return {@link Hood}
   */
  public static Hood create(BooleanSupplier coastOverride) {
    // TODO sim
    return new Hood(
        new ServoIOTalonFx(
            "HoodServo", Constants.kCanBus, Constants.kServoCanDeviceId, Constants.kServoConfig),
        new EncoderIOCanCoder(
            Constants.kEncoderCanDeviceId, Constants.kCanBus, Constants.kEncoderConfig),
        coastOverride);
  }

  // * MEMBERS
  public final DoubleEntry kSTunable =
      NetworkTableUtil.createEntry("Characterization/Hood/KsAmps", 0.0);
  public final DoubleEntry kGTunable =
      NetworkTableUtil.createEntry("Characterization/Hood/KgAmps", 0.0);
  public final DoubleEntry kATunable =
      NetworkTableUtil.createEntry("Characterization/Hood/KaAmps", 0.0);
  public final DoubleEntry kPTunable =
      NetworkTableUtil.createEntry("Characterization/Hood/KpAmps", 0.0);
  public final DoubleEntry kDTunable =
      NetworkTableUtil.createEntry("Characterization/Hood/KdAmps", 0.0);

  @Logged private final ServoIO mServo;
  @Logged private final EncoderIO mEncoder;

  private final BooleanSupplier mCoastOverride;
  private boolean mIsCharacterizing = false;

  private Rotation2d mTargetAngle = Rotation2d.kZero;
  private State mSetpointProfileState = new State(0.0, 0.0);

  /**
   * Create new {@link Hood}
   *
   * @param servo {@link ServoIO} representing servo driving the system
   * @param encoder {@link EncoderIO} representing encoder measure the angle of system
   * @param coastOverride {@link BooleanSupplier} providing if coast mode should be enabled
   */
  private Hood(ServoIO servo, EncoderIO encoder, BooleanSupplier coastOverride) {
    mServo = servo;
    mEncoder = encoder;

    mCoastOverride = coastOverride;

    SmartDashboard.putData("Characterization/Hood/runStatic", runQuasistaticCharacterization(5));
  }

  @Override
  public void periodic() {
    mServo.periodic();
    mEncoder.periodic();

    // Accept values from tunables if tuning mode
    if (Flags.kTuningModeEnabled) {
      if (kSTunable.getAsDouble() != Constants.kS.in(Amps))
        Constants.kS = Amps.of(kSTunable.getAsDouble());

      if (kGTunable.getAsDouble() != Constants.kG.in(Amps))
        Constants.kG = Amps.of(kSTunable.getAsDouble());

      if (kATunable.getAsDouble() != Constants.kA.in(Amps))
        Constants.kA = Amps.of(kSTunable.getAsDouble());

      if (kPTunable.getAsDouble() != Constants.kP.in(Amps))
        Constants.kP = Amps.of(kSTunable.getAsDouble());

      if (kDTunable.getAsDouble() != Constants.kD.in(Amps))
        Constants.kD = Amps.of(kSTunable.getAsDouble());
    }

    // Check if setpoint should be applied
    boolean runProfile =
        DriverStation.isEnabled() && !mIsCharacterizing && !mCoastOverride.getAsBoolean();

    if (runProfile) {
      double previousVel = mSetpointProfileState.velocity;

      var targetState = new State(mTargetAngle.getRotations(), 0.0);
      mSetpointProfileState =
          Constants.kServoMotionProfile.calculate(0.02, getMotionProfileState(), targetState);

      var setpointAcceleration = (mSetpointProfileState.velocity - previousVel) / 0.02;

      var feedforward =
          Amps.zero()
              .plus(Constants.kS.times(Math.signum(mSetpointProfileState.velocity)))
              .plus(Constants.kG.times(getRotation2d().getCos()))
              .plus(Constants.kA.times(setpointAcceleration));

      var feedback =
          Amps.zero()
              .plus(
                  Constants.kP.times(
                      mSetpointProfileState.position - getRotation2d().getRotations()))
              .plus(
                  Constants.kD.times(
                      mSetpointProfileState.velocity
                          - getAngularVelocity().in(RotationsPerSecond)));

      mServo.setTorqueCurrentSetpoint(feedforward.plus(feedback));
    }
  }

  // * GETTERS
  /**
   * @return {@link Boolean} true when subsystem has adjust to target angle
   */
  public boolean adjusted() {
    return isNearSetpoint();
  }

  /**
   * @return {@link Boolean} true when subsystem is adjusting to target angle
   */
  public boolean adjusting() {
    return !isNearSetpoint();
  }

  /**
   * Check if angular position error of subsystem is less than epsilon of subsystem
   *
   * @return {@link Boolean}
   */
  @Logged(name = "isNearSetpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return isNearAngle(mTargetAngle);
  }

  /**
   * Check if error between subsysem & specified angular positions is less than epsilon of subsystem
   *
   * @param angle {@link Rotation2d} representing the angular position to check
   * @return {@link Boolean}
   */
  public boolean isNearAngle(Rotation2d angle) {
    return MathUtil.isNear(
        getTargetRotation2d().getRotations(),
        angle.getRotations(),
        Constants.kEpsilon.in(Rotations));
  }

  /**
   * @return {@link Rotation2d} representing the angular position of subsystem
   */
  @Logged(name = "Rotation2d", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return new Rotation2d(mEncoder.getAngle().div(Constants.kSensorToMechRatio))
        .plus(new Rotation2d(Constants.kMinAngle));
  }

  /**
   * @return {@link Rotation2d} representing the target anglular position subsystem is trying to get
   *     to
   */
  @Logged(name = "TargetRotation2d", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return mTargetAngle;
  }

  /**
   * @return {@link AngularVelocity} representing the angular velocity of subsystem
   */
  public AngularVelocity getAngularVelocity() {
    return mServo
        .getAngularVelocity()
        .div(Constants.kRotorToSensorRatio * Constants.kSensorToMechRatio);
  }

  /**
   * @return {@link State} representing the angular position and velocity of subsystem in revs
   */
  @Logged(name = "MotionProfileState", importance = Importance.INFO)
  public State getMotionProfileState() {
    return new State(getRotation2d().getRotations(), getAngularVelocity().in(RotationsPerSecond));
  }

  /**
   * @return {@link State} representing the setpoint angular position and velocity of subsystem in
   *     revs
   */
  @Logged(name = "SetpointMotionProfileState", importance = Importance.INFO)
  public State getSetpointMotionProfileState() {
    return mSetpointProfileState;
  }

  // * SETTERS
  /**
   * Set target anglular position
   *
   * @param angle {@link Rotation2d} representing the target position
   */
  private void setTargetAngle(Rotation2d angle) {
    mTargetAngle =
        Rotation2d.fromRotations(
            MathUtil.clamp(
                angle.getRotations(),
                Constants.kMinAngle.in(Rotations),
                Constants.kMaxAngle.in(Rotations)));
  }

  // * COMMANDS
  /**
   * Request subsystem to fold completely
   *
   * @return {@link Command}
   */
  public Command stow() {
    return adjustToAngle(new Rotation2d(Constants.kMinAngle));
  }

  /**
   * Request subsystem to adjust to angle
   *
   * @param angle {@link Rotation2d} representing angle
   * @return {@link Command}
   */
  public Command adjustToAngle(Rotation2d angle) {
    return adjustToAngle(() -> angle);
  }

  /**
   * Request subsystem to continiously adjust to the value of an angle stream
   *
   * @param angle {@link Supplier} of {@link Rotation2d} representing the angle to adjust to
   * @return {@link Command}
   */
  public Command adjustToAngle(Supplier<Rotation2d> angle) {
    return this.run(() -> setTargetAngle(angle.get()));
  }

  /**
   * Attempt to increase servo output current at a constant rate to find the current gain acting
   * against static fricition
   *
   * @return {@link Command}
   */
  public Command runQuasistaticCharacterization(int trials) {
    Timer timer = new Timer();
    final QuasistaticState state = new QuasistaticState();
    var logger = Epilogue.getConfig().backend;
    final ArrayList<Current> samples = new ArrayList<>();

    return this.startEnd(
            () -> {
              mIsCharacterizing = true;
              state.staticAmps = Amps.zero();
              timer.restart();
            },
            () -> {
              state.staticAmps =
                  state.staticAmps.plus(
                      Constants.kQuasistaticCharacterizationRampRate.times(timer.get()));

              mServo.setTorqueCurrentSetpoint(state.staticAmps);
              logger.log("Characterization/Hood/StaticAmps", state.staticAmps);
            })
        .until(
            () -> getAngularVelocity().gt(Constants.kQuasistaticCharacterizationVelocityThreshold))
        .andThen(() -> mServo.stop())
        .finallyDo(
            () -> {
              timer.stop();
              samples.add(state.staticAmps);
            })
        .repeatedly()
        .until(() -> samples.toArray().length == trials)
        .finallyDo(
            () -> {
              Current avg = Amps.zero();
              for (var sample : samples) {
                System.out.println("Sample 1 ~ " + sample.in(Amps) + " Amps");
                avg.plus(sample);
              }

              System.out.println("Avg ~ " + avg.div(trials) + " Amps");
            });
  }

  /** A {@link Current} wrapper class for quasistatic characterization */
  private class QuasistaticState {
    public Current staticAmps = Amps.zero();
  }

  /**
   * Attempt to apply a constant servo output current to find the current gain acting against
   * current gain inducing acceleration
   *
   * @return {@link Command}
   */
  public Command runDynamicCharacterizatoin() {
    Timer timer = new Timer();
    var logger = Epilogue.getConfig().backend;
    AngularVelocity velocity = RotationsPerSecond.zero();

    // TODO
    return Commands.none();
  }
}
