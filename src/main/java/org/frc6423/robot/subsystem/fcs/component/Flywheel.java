// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.fcs.component;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxFlywheelSim;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

public class Flywheel {
  // * ~~~~~~~~ STATIC CONSTANT MEMBERS ~~~~~~~~

  // Physical Constants

  /** {@link Distance} Radius of combined flywheel drum */
  public static final Distance kRadius = Inches.of(2);

  /** {@link MomentOfInertia} Rotational Inertia of flywheel system */
  public static final MomentOfInertia kRotationalInertia =
      KilogramSquareMeters.of(10.491008 * 0.0002926397);

  public static final double kEpsilonDebounceTime = 0.03;

  // Control Constants

  /** {@link Double} Max allowed error in subsystem angular velocity */
  public static final AngularVelocity kEpsilon = DegreesPerSecond.of(2);

  // Hardware Constants

  /** {@link Integer} CAN ID of the left flywheel servo */
  public static final int kLeftCanDeviceId = Matrix.kFlywheelLeftId;

  /** {@link Integer} CAN ID of the right flywheel servo */
  public static final int kRightCanDeviceId = Matrix.kFlywheelLeftId;

  /** {@link Current} Stator current limit of servos */
  public static final Current kServoStatorCurrentLimit = Amps.of(120);

  /** {@link Current} Supply current limit of servos */
  public static final Current kServoSupplyCurrentLimit = Amps.of(40.0);

  /** {@link Double} Gear ratio between the servo rotor and the flywheel shaft */
  public static final double kSensorToMechanismRatio = (28.0 / 16.0);

  /** {@link Integer} Gains slot used for accelerating flywheel */
  public static final int kGainSlotAccel = 0;

  /** {@link Integer} Gains slot used for maintaing flywheel velocity */
  public static final int kGainSlotMaintain = 1;

  /** {@link Integer} Gains slot used for deaccelerating flywheel */
  public static final int kGainSlotDeaccel = 2;

  /** {@link TalonFXConfiguration} Hardware config of the flywheel servos */
  public static final TalonFXConfiguration kServoTalonConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(kServoStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(kServoSupplyCurrentLimit)
                  .withSupplyCurrentLimitEnable(true))
          .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(9999.0))
          .withSlot0(
              new Slot0Configs()
                  .withKS(4.8691)
                  .withKV(0.10515)
                  .withKA(1.729)
                  .withKG(0.52863)
                  .withKP(0.26322)
                  .withKD(0.0)) // Torque Current Control Gains (accelerating)
          .withSlot1(
              new Slot1Configs()
                  .withKS(4.8691)
                  .withKV(0.10515)
                  .withKA(1.729)
                  .withKG(0.52863)
                  .withKP(0.26322)
                  .withKD(0.0)) // Torque Current Control Gains (maintaing)
          .withSlot2(
              new Slot2Configs()
                  .withKS(4.8691)
                  .withKV(0.10515)
                  .withKA(1.729)
                  .withKG(0.52863)
                  .withKP(0.26322)
                  .withKD(0.0)); // Torque Current Control Gains (deaccelerating)

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  // Characterization Constants

  /** {@link Velocity} of {@link CurrentUnit} Rate at which current output ramps up at in Quasi */
  public static final Velocity<CurrentUnit> kCharacterizationRampRate = Amps.of(35.0).per(Second);

  /** {@link Current} Current step size for Dyna */
  public static final Current kCharacterizationStepSize = Amps.of(15.0);

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

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  @Logged private final ServoIO mLeft, mRight;

  private AngularVelocity mTargetVelocity = RevolutionsPerSecond.zero();

  private final Debouncer mIsAtSetpoint = new Debouncer(kEpsilonDebounceTime);

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
  }

  public void periodic() {
    // Update all hardware
    mLeft.periodic();
    mRight.periodic();

    mIsAtSetpoint.calculate(
        MathUtil.isNear(
            mTargetVelocity.in(RadiansPerSecond),
            getAngularVelocity().in(RadiansPerSecond),
            kEpsilon.in(RadiansPerSecond)));

    // Switch between gain slots
    if (mTargetVelocity.gt(getAngularVelocity())) {
      mLeft.setGainsSlot(kGainSlotAccel);
    } else if (isHoldingSetpoint()) {
      mLeft.setGainsSlot(kGainSlotMaintain);
    } else if (mTargetVelocity.lt(getAngularVelocity())) {
      mLeft.setGainsSlot(kGainSlotDeaccel);
    }
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  @Logged(name = "Holding Setpoint (bool)", importance = Importance.INFO)
  public boolean isHoldingSetpoint() {
    return mIsAtSetpoint.calculate(
        MathUtil.isNear(
            getTargetAngularVelocity().in(RadiansPerSecond),
            getAngularVelocity().in(RadiansPerSecond),
            kEpsilon.in(RadiansPerSecond)));
  }

  @Logged(name = "Approximated Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public LinearVelocity getApproximatedMuzzleVelocity() {
    return MetersPerSecond.of(getAngularVelocity().in(RadiansPerSecond) * kRadius.in(Meters) * 0.5);
  }

  @Logged(name = "Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return mLeft.getAngularVelocity();
  }

  @Logged(name = "Target Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public LinearVelocity getTargetMuzzleVelocity() {
    return MetersPerSecond.of(
        getTargetAngularVelocity().in(RadiansPerSecond) * kRadius.in(Meters) * 0.5);
  }

  @Logged(name = "Target Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getTargetAngularVelocity() {
    return mTargetVelocity;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  public void coast() {
    mLeft.stop();
  }

  public void setTargetMuzzleVelocity(LinearVelocity velocity) {
    runTargetMuzzleVelocity(() -> velocity);
  }

  public void runTargetMuzzleVelocity(Supplier<LinearVelocity> velocitySupplier) {
    // Target Velocity = ((Target Muzzle Velocity) / Radius) * 2.0
    mTargetVelocity =
        RadiansPerSecond.of(velocitySupplier.get().in(MetersPerSecond) / kRadius.in(Meters) * 2.0);

    mLeft.setTorqueMotionProfiledVelocitySetpoint(mTargetVelocity);
  }
}
