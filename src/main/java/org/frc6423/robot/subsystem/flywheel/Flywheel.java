// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
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
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;

/**
 * {@link SubsystemBase} Flywheel Subsystem
 *
 * <p>Subsystem is powered by two Kraken x60s
 */
public class Flywheel extends SubsystemBase {
  /** Constants for the {@link Flywheel} */
  public class Constants {
    // * PHYSICAL CONSTANTS
    /** {@link MomentOfInertia} Rotational Inertia of flywheel system */
    public static final MomentOfInertia kRotationalInertia =
        KilogramSquareMeters.of(10.491008 * 0.0002926397);

    // * CONTROL CONSTANTS
    /** {@link Double} Max allowed percent error in subsystem velocity */
    public static final double kEpsilon = 0.01;

    // * HARDWARE CONSTANTS
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
    private static final TalonFXConfiguration kServoTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
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
                    .withKS(4.7248)
                    .withKV(0.10759)
                    .withKA(3.6145)
                    .withKP(0.23165)
                    .withKD(0.0)) // Torque Current Control Gains (accelerating)
            .withSlot1(
                new Slot1Configs()
                    .withKS(4.7248)
                    .withKV(0.10759)
                    .withKA(3.6145)
                    .withKP(0.23165)
                    .withKD(0.0)) // TODO Torque Current Control Gains (maintaing)
            .withSlot2(
                new Slot2Configs()
                    .withKS(4.7248)
                    .withKV(0.10759)
                    .withKA(3.6145)
                    .withKP(0.23165)
                    .withKD(0.0)); // TODO Torque Current Control Gains (deaccelerating)

    /** {@link CANBus} CAN bus devices are on */
    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    // * CHARACTERIZATON
    /** {@link Velocity} of {@link CurrentUnit} Rate at which current output ramps up at in Quasi */
    public static final Velocity<CurrentUnit> kCharacterizationRampRate = Amps.of(35.0).per(Second);

    /** {@link Current} Current step size for Dyna */
    public static final Current kCharacterizationStepSize = Amps.of(15.0);
  }

  /**
   * Create new {@link Flywheel}
   *
   * @return {@link Flywheel}
   */
  public static Flywheel create() {
    return (Robot.isReal())
        ? new Flywheel(
            new ServoIOTalonFx(
                "Left", Constants.kCanBus, Constants.kLeftCanDeviceId, Constants.kServoTalonConfig),
            new ServoIOTalonFx(
                "Right",
                Constants.kCanBus,
                Constants.kRightCanDeviceId,
                Constants.kServoTalonConfig))
        : new Flywheel(
            new ServoIOTalonFxFlywheelSim(
                "Left",
                Constants.kCanBus,
                Constants.kLeftCanDeviceId,
                Constants.kServoTalonConfig,
                Constants.kRotationalInertia,
                MotorType.KrakenX60,
                DCMotor.getKrakenX60Foc(2),
                Constants.kSensorToMechanismRatio),
            new ServoIOTalonFx(
                "Right",
                Constants.kCanBus,
                Constants.kRightCanDeviceId,
                Constants.kServoTalonConfig)); // We can get away with one sim
  }

  @Logged private final ServoIO mLeft, mRight;

  private final SysIdRoutine mSysIdRoutine;

  private final LinearFilter mCurrentFilter;

  private AngularVelocity mTargetVelocity = RevolutionsPerSecond.zero();

  /**
   * Create new {@link Flywheel}
   *
   * @param left {@link ServoIO} Left flywheel servo
   * @param right {@link ServoIO} Right flywheel servo
   */
  public Flywheel(ServoIO left, ServoIO right) {
    // Init Hardware
    mLeft = left;
    mRight = right;

    mRight.setLeader(mLeft, true);

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
                        .log("Characterization/Flywheel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> mLeft.setTorqueCurrentSetpoint(Amps.of(voltage.in(Volts))),
                null,
                this));

    // Publish characterization command in tuning mode
    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData("Run Flywheel SysId Characterization", runCharacterizationSequence());
    }

    // TODO idk if high pass is the correct filter to use here tbh, should test with moving avg as
    // well
    // Init current filter
    mCurrentFilter = LinearFilter.highPass(0.1, 0.02);
  }

  @Override
  public void periodic() {
    // Update all Hardware
    mLeft.periodic();
    mRight.periodic();

    // Update Current Filter
    mCurrentFilter.calculate(mLeft.getSupplyCurrent().in(Amps));

    // Switch between gain slots
    if (mTargetVelocity.gt(getAngularVelocity())) {
      mLeft.setGainsSlot(Constants.kGainSlotAccel);
    } else if (isNearSetpoint()) {
      mLeft.setGainsSlot(Constants.kGainSlotMaintain);
    } else if (mTargetVelocity.lt(getAngularVelocity())) {
      mLeft.setGainsSlot(Constants.kGainSlotDeaccel);
    }
  }

  // * GETTERS
  /**
   * Get the supply current of left flywheel servo with linear filter applied (High Pass)
   *
   * <p>TODO docs if filter type changes
   *
   * @return {@link Current}
   */
  @Logged(name = "Filtered Supply Current (amps)", importance = Importance.INFO)
  public Current getFilteredCurrent() {
    return Amps.of(mCurrentFilter.calculate(mLeft.getSupplyCurrent().in(Amps)));
  }

  /**
   * Get Angular Position of subsystem
   *
   * @return {@link Angle}
   */
  @Logged(name = "Angular Position (rads)", importance = Importance.INFO)
  public Angle getAngle() {
    return mLeft.getAngle();
  }

  /**
   * Get Angular Velocity of subsystem
   *
   * @return {@link AngularVelocity}
   */
  @Logged(name = "Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return mLeft.getAngularVelocity();
  }

  /**
   * Get Setpoint Angular Velocity of subsystem
   *
   * @return {@link AngularVelocity}
   */
  @Logged(name = "Setpoint Angular Velocity (rads per second)", importance = Importance.INFO)
  public AngularVelocity getAngularVelocitySetpoint() {
    return mTargetVelocity;
  }

  /**
   * Check if subsystem is nearly at setpoint angular velocity
   *
   * @return {@link Boolean}
   */
  @Logged(name = "is Near Setpoint (bool)", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return getAngularVelocity().in(RadiansPerSecond) / mTargetVelocity.in(RadiansPerSecond)
        > Constants.kEpsilon;
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
   * Request subsystem to coast
   *
   * <p>When coast, flywheel servos will not apply any output; flywheel will be allowed to spin down
   *
   * @return {@link Command}
   */
  public Command coast() {
    return this.run(() -> mLeft.stop()).withName("FlywheelCoast");
  }

  /**
   * TODO wip storage command
   *
   * <p>This command should store surplus supply current
   *
   * <p>Make command public once finished
   *
   * @return {@link Command}
   */
  protected Command store() {
    return this.startRun(
        () -> mLeft.stop(),
        () -> {
          if (getFilteredCurrent().gt(Amps.zero())) {}
        });
  }

  /**
   * Request subsystem to accelerate to desired angular velocity
   *
   * @param velocity {@link AngularVelocity} Angular Velocity setpoint for subsystem to accelerate
   *     to
   * @return {@link Command}
   */
  public Command accelerateToVelocity(AngularVelocity velocity) {
    return accelerateToVelocity(() -> velocity).withName("Flywheel Accelerate to");
  }

  /**
   * Request subsystem to continiously accelerate to a stream of angular velocity setpoints
   *
   * @param velocity {@link Supplier}<{@link AngularVelocity}> Stream of angular velocity setpoints
   * @return {@link Command}
   */
  public Command accelerateToVelocity(Supplier<AngularVelocity> velocity) {
    return this.run(
            () -> {
              mTargetVelocity = velocity.get();
              mLeft.setTorqueMotionProfiledVelocitySetpoint(mTargetVelocity);
            })
        .withName("Flywheel Accelerate to Continously");
  }
}
