// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the flywheel subsystem
 *
 * <p>The {@link Flywheel} has two servos that drive it
 */
public class Flywheel extends SubsystemBase {
  /** {@Flywheel} subsystem constants */
  public class Constants {
    // * CONTROL CONSTANTS
    /** {@link Double} representing the maximum allowed percent error in angular velocity */
    public static final double kEpsilon = 0.01;

    // * HARDWARE CONSTANTS
    /** {@link Integer} representing the CAN ID of the left flywheel servo */
    private static final int kLeftCanDeviceId = Matrix.kFlywheelLeftId;

    /** {@link Integer} representing the CAN ID of the right flywheel servo */
    private static final int kRightCanDeviceId = Matrix.kFlywheelLeftId;

    /** {@link Current} representing the stator current limit of servos */
    public static final Current kServoStatorCurrentLimit = Amps.of(120);

    /** {@link Current} representing the supply current limit of servos */
    public static final Current kServoSupplyCurrentLimit = Amps.of(40.0);

    /** {@link Double} representing the gear ratio between the servo rotor and the flywheel shaft */
    public static double kRotorToSensorRatio = (28.0 / 16.0);

    /** {@link TalonFXConfiguration} representing the hardware config of the flywheel servos */
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
            .withSlot0(new Slot0Configs().withKS(0.0).withKA(0.0).withKP(0.0).withKD(0.0));

    /** {@link CANBus} representing bus CAN devices are on */
    public static final CANBus kCanBus = Matrix.kSubsystemCanBus;
  }

  /**
   * Create new {@link Flywheel}
   *
   * @return {@link Flywheel}
   */
  public static Flywheel create() {
    // TODO sim
    return new Flywheel(
        new ServoIOTalonFx(
            "Left", Constants.kCanBus, Constants.kLeftCanDeviceId, Constants.kServoTalonConfig),
        new ServoIOTalonFx(
            "Right", Constants.kCanBus, Constants.kRightCanDeviceId, Constants.kServoTalonConfig));
  }

  @Logged private final ServoIO mLeft, mRight;

  private AngularVelocity mTargetVelocity = RevolutionsPerSecond.zero();

  private final SysIdRoutine mCharacterization;

  /**
   * Create new {@link Flywheel}
   *
   * @param left {@link ServoIO} representing the left servo spinning flywheel
   * @param right {@link ServoIO} representing the right servo spinning flywheel
   * @param coastOverride {@link Boolean}
   */
  public Flywheel(ServoIO left, ServoIO right) {
    // Init hardware
    mLeft = left;
    mRight = right;

    mRight.setLeader(mLeft, true);

    // Init characterization routines
    // TODO idk how well these will work for torque current control
    mCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Epilogue.getConfig().backend.log("Telemetry/Flywheel/SysID State", state)),
            new SysIdRoutine.Mechanism(
                (voltage) -> mLeft.setTorqueCurrentSetpoint(Amps.of(voltage.in(Volts))),
                null,
                this));

    if (Flags.kTuningModeEnabled) {
      SmartDashboard.putData(
          "Quasistatic Forward", mCharacterization.quasistatic(Direction.kForward));
      SmartDashboard.putData(
          "Quasistatic Reverse", mCharacterization.quasistatic(Direction.kReverse));

      SmartDashboard.putData("Dynamic Forward", mCharacterization.dynamic(Direction.kForward));
      SmartDashboard.putData("Dynamic Reverse", mCharacterization.dynamic(Direction.kReverse));
    }
  }

  @Override
  public void periodic() {
    mLeft.periodic();
    mRight.periodic();

    mLeft.setTorqueMotionProfiledVelocitySetpoint(mTargetVelocity);
  }

  // * GETTERS
  /**
   * @return {@link AngularVelocity} representing the angular velocity of flywheel
   */
  @Logged(name = "Angular Velocity", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return mLeft.getAngularVelocity();
  }

  /**
   * @return {@link AngularVelocity} representing the setpoint angular velocity of flywheel
   */
  @Logged(name = "Setpoint Angular Velocity", importance = Importance.INFO)
  public AngularVelocity getAngularVelocitySetpoint() {
    return mTargetVelocity;
  }

  /**
   * @return true when the percent error from the setpoint angular velocity is less than the epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return getAngularVelocity().in(RadiansPerSecond) / mTargetVelocity.in(RadiansPerSecond)
        > Constants.kEpsilon;
  }

  // * COMMANDS
  /**
   * Request subsystem to stop applying output and coast
   *
   * @return {@link Command}
   */
  public Command coast() {
    return this.run(() -> mLeft.stop());
  }

  /**
   * Request subsystem to accelerate to a specified angular velocity
   *
   * @param velocity {@link AngularVelocity} representing the velocity to accelerate to
   * @return {@link Command}
   */
  public Command accelerateTo(AngularVelocity velocity) {
    return accelerateToVelocity(() -> velocity);
  }

  /**
   * Request subsystem to continiously accelerate to the angular velocity from a stream
   *
   * @param velocity {@link Suppler} of {@link AngularVelocity} representing the velocity to
   *     accelerate to
   * @return {@link Command}
   */
  public Command accelerateToVelocity(Supplier<AngularVelocity> velocity) {
    return this.run(() -> mTargetVelocity = velocity.get());
  }
}
