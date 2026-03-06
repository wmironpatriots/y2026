// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.frc6423.robot.subsystem.flywheel.FlywheelConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxFlywheelSim;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;

/**
 * {@link SubsystemBase} Interface for controlling the velocity of robot flywheel
 *
 * <p>Subsystem constants can be found in {@link FlywheelConstants}
 *
 * <p>To accelerate flywheel to a velocity for ejecting projectiles at a desired linear velocity,
 * use the {@link #accelerateToMuzzleVelocity(LinearVelocity)} method
 */
@Deprecated
public class Flywheel extends SubsystemBase {
  /**
   * Create new {@link Flywheel}
   *
   * @return {@link Flywheel}
   */
  public static Flywheel create() {
    return (Robot.isReal())
        ? new Flywheel(
            new ServoIOTalonFx("Left", kCanBus, kLeftCanDeviceId, kServoTalonConfig),
            new ServoIOTalonFx("Right", kCanBus, kRightCanDeviceId, kServoTalonConfig))
        : new Flywheel(
            new ServoIOTalonFxFlywheelSim(
                "Left",
                kCanBus,
                kLeftCanDeviceId,
                kServoTalonConfig,
                kRotationalInertia,
                MotorType.KrakenX60,
                DCMotor.getKrakenX60Foc(2),
                kSensorToMechanismRatio),
            new ServoIOTalonFx(
                "Right",
                kCanBus,
                kRightCanDeviceId,
                kServoTalonConfig)); // We can get away with one sim
  }

  // * HARDWARE MEMBERS
  @Logged private final ServoIO mLeft, mRight;

  // * CHARACTERIZATION/MEASUREMENT
  private final SysIdRoutine mSysIdRoutine;

  private AngularVelocity mTargetVelocity = RevolutionsPerSecond.zero();

  /**
   * Create new {@link Flywheel}
   *
   * @param left {@link ServoIO} Left servo connected to flywheel
   * @param right {@link ServoIO} Right servo connected to flywheel
   */
  private Flywheel(ServoIO left, ServoIO right) {
    // Init Hardware
    mLeft = left;
    mRight = right;

    // Set left servo as leader
    mRight.setLeader(mLeft, true);

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
                        .log("Characterization/Flywheel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> mLeft.setTorqueCurrentSetpoint(Amps.of(voltage.in(Volts))),
                null,
                this));

    // Publish characterization command in tuning mode
    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData("Run Flywheel SysId Characterization", runCharacterizationSequence());
    }
  }

  @Override
  public void periodic() {
    // Update all Hardware
    mLeft.periodic();
    mRight.periodic();

    // Switch between gain slots
    if (mTargetVelocity.gt(getAngularVelocity())) {
      mLeft.setGainsSlot(kGainSlotAccel);
    } else if (isNearSetpoint()) {
      mLeft.setGainsSlot(kGainSlotMaintain);
    } else if (mTargetVelocity.lt(getAngularVelocity())) {
      mLeft.setGainsSlot(kGainSlotDeaccel);
    }
  }

  // * GETTERS
  /**
   * Get angular position of flywheel
   *
   * @return {@link Angle}
   */
  @Logged(name = "Angular Position (rads)", importance = Importance.INFO)
  public Angle getAngle() {
    return mLeft.getAngle();
  }

  /**
   * Get Angular Velocity of flywheel
   *
   * @return {@link AngularVelocity}
   */
  @Logged(name = "Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return mLeft.getAngularVelocity();
  }

  /**
   * Get Angular Velocity Setpoint of flywheel
   *
   * @return {@link AngularVelocity}
   */
  @Logged(name = "Setpoint Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getAngularVelocitySetpoint() {
    return mTargetVelocity;
  }

  /**
   * Get muzzle velocity (aka approximated projectile velocity) of flywheel
   *
   * @return {@link LinearVelocity}
   */
  @Logged(name = "Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public LinearVelocity getMuzzleVelocity() {
    return MetersPerSecond.of(mTargetVelocity.in(RadiansPerSecond) * kRadius.in(Meters) * 0.5);
  }

  /**
   * Check if subsystem is nearly at setpoint angular velocity
   *
   * @return {@link Boolean}
   */
  @Logged(name = "is Near Setpoint (bool)", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return MathUtil.isNear(
        mTargetVelocity.in(RevolutionsPerSecond),
        getAngularVelocity().in(RevolutionsPerSecond),
        kEpsilon.in(RevolutionsPerSecond));
  }

  // * COMMANDS
  /**
   * Run SysId Characterization Routine for determining gains
   *
   * <p>Tests will run as follows: +Quasi, -Quasi, +Dyna, -Dyna
   *
   * <p>Tests will not start until flywheel has stopped running
   *
   * @return {@link Command}
   */
  public Command runCharacterizationSequence() {
    return Commands.sequence(
            mSysIdRoutine.quasistatic(Direction.kForward),
            Commands.waitUntil(() -> isNearSetpoint()),
            mSysIdRoutine.quasistatic(Direction.kReverse),
            Commands.waitUntil(() -> isNearSetpoint()),
            mSysIdRoutine.dynamic(Direction.kForward),
            Commands.waitUntil(() -> isNearSetpoint()),
            mSysIdRoutine.dynamic(Direction.kReverse))
        .beforeStarting(() -> mTargetVelocity = RevolutionsPerSecond.zero(), this)
        .withName("Flywheel Characterization");
  }

  /**
   * Request flywheel to coast
   *
   * <p>When coast, flywheel servos will not apply any output; flywheel will be allowed to spin down
   *
   * @return {@link Command}
   */
  public Command coast() {
    return this.run(() -> mLeft.stop()).withName("FlywheelCoast");
  }

  /**
   * Accelerate flywheel to angular velocity where projectiles will be ejcted at a desired linear
   * velocity
   *
   * @param velocity {@link LinearVelocity} Muzzle velocity setpoint
   * @return {@link Command}
   */
  public Command accelerateToMuzzleVelocity(LinearVelocity velocity) {
    return accelerateToMuzzleVelocity(() -> velocity);
  }

  /**
   * Accelerate flywheel to angular velocity where projectiles will be ejcted at a desired linear
   * velocity
   *
   * @param velocity {@link Supplier} of {@link LinearVelocity} Stream of muzzle velocity setpoints
   * @return {@link Command}
   */
  public Command accelerateToMuzzleVelocity(Supplier<LinearVelocity> velocity) {
    return this.run(
            () -> {
              // Projectile Velocity = (Flywheel Velocity) * Radius * 0.5
              mTargetVelocity =
                  RadiansPerSecond.of(
                      velocity.get().in(MetersPerSecond) / kRadius.in(Meters) * 2.0);

              mLeft.setTorqueMotionProfiledVelocitySetpoint(mTargetVelocity);
            })
        .withName("Flywheel Accelerate to Continously");
  }
}
