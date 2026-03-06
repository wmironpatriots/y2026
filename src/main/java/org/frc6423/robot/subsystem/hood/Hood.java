// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.frc6423.robot.subsystem.hood.HoodConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
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
@Deprecated
public class Hood extends SubsystemBase {

  /**
   * Create new {@link Hood}
   *
   * @return {@link Hood}
   */
  public static Hood create() {
    return (Robot.isReal())
        ? new Hood(
            new ServoIOTalonFx("Servo", kCanBus, kServoCanDeviceId, kServoTalonConfig),
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig))
        : new Hood(
            new ServoIOTalonFxPivotSim(
                "Servo",
                kCanBus,
                kServoCanDeviceId,
                kServoTalonConfig,
                kRotationalInertia,
                kLength,
                kMinAngle,
                kMaxAngle,
                kMinAngle,
                true,
                MotorType.KrakenX44,
                DCMotor.getKrakenX44Foc(1),
                kSensorToMechRatio * kRotorToSensorRatio),
            new EncoderIOCanCoder(kEncoderCanDeviceId, kCanBus, kEncoderConfig));
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
                Volts.of(kCharacterizationRampRate.in(Amps.per(Second))).per(Second),
                Volts.of(kCharacterizationStepSize.in(Amps)),
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
        mTargetAngle.in(Revolutions), getAngle().in(Revolutions), kEpsilon.in(Revolutions));
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
                .until(() -> getAngle().gt(kMaxAngle.minus(Degrees.of(10)))),
            Commands.print("Quasi Reverse"),
            mSysIdRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> getAngle().lt(kMinAngle.plus(Degrees.of(10)))),
            Commands.print("Dyna Forward"),
            mSysIdRoutine
                .dynamic(Direction.kForward)
                .until(() -> getAngle().gt(kMaxAngle.minus(Degrees.of(10)))),
            Commands.print("Dyna Reverse"),
            mSysIdRoutine
                .dynamic(Direction.kReverse)
                .until(() -> getAngle().lt(kMinAngle.plus(Degrees.of(10)))),
            Commands.print("Done!"))
        .withName("Hood Characterization");
  }

  /**
   * Request subsystem to stow (aka fold completely)
   *
   * @return {@link Command}
   */
  public Command stow() {
    return adjustToAngle(kMinAngle).withName("Hood Stow");
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
                          kMinAngle.in(Revolutions),
                          kMaxAngle.in(Revolutions)));

              mServo.setTorqueMotionProfiledPositionSetpoint(mTargetAngle);
            })
        .withName("Hood Adjust to Angle Continiously");
  }
}
