// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the flywheel subsystem
 *
 * <p>The {@link Flywheel} has two servos that drive it
 */
public class Flywheel extends SubsystemBase {
  /** {@Flywheel} subsystem constants */
  public class Constants {
    /** {@link CANBus} representing the bus devices are connected to */
    private static final CANBus kCanBus = Matrix.kSubsystemCanBus;

    /** {@link Integer} representing the left servo's CAN ID on CANBUS */
    private static final int kLeftCanDeviceId = Matrix.kFlywheelLeftId;

    /** {@link Integer} representing the right servo's CAN ID on CANBUS */
    private static final int kRightCanDeviceId = Matrix.kFlywheelLeftId;

    /** {@link TalonFXConfiguration} representing the hardware config of the servo */
    private static final TalonFXConfiguration kServoTalonConfig =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs().withKS(0.0).withKA(0.0).withKP(0.0).withKD(0.0));

    // TODO
    public static final double kEpsilon = 0.01;
  }

  private final ServoIO mLeft, mRight;

  private State mState = State.COASTING;

  private AngularVelocity mVelocitySetpoint = RevolutionsPerSecond.zero();
  private final double mEpsilon;

  private final SysIdRoutine mCharacterization;

  /**
   * Create new {@link Flywheel}
   *
   * @param left {@link ServoIO} representing the left servo spinning flywheel
   * @param right {@link ServoIO} representing the right servo spinning flywheel
   * @param epsilon {@link Double} representing the largest acceptable amount of percent error of
   *     angular velocity from setpoint velocity
   */
  public Flywheel(ServoIO left, ServoIO right, double epsilon) {
    mLeft = left;
    mRight = right;
    mEpsilon = epsilon;

    mRight.setLeader(mLeft, true);

    mCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Epilogue.getConfig().backend.log("Telemetry/Flywheel/SysID State", state)),
            new SysIdRoutine.Mechanism(
                (voltage) -> mLeft.setVoltageSetpoint(voltage, true), null, this));

    SmartDashboard.putData(
        "Quasistatic Forward", mCharacterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Quasistatic Reverse", mCharacterization.quasistatic(Direction.kReverse));

    SmartDashboard.putData("Dynamic Forward", mCharacterization.dynamic(Direction.kForward));
    SmartDashboard.putData("Dynamic Reverse", mCharacterization.dynamic(Direction.kReverse));
  }

  @Override
  public void periodic() {
    mLeft.periodic();
    mRight.periodic();

    switch (mState) {
      case COASTING:
        mLeft.stop();
        break;
      case ACCELERATING:
        mLeft.setVoltageVelocitySetpoint(mVelocitySetpoint, true, 0);

        if (isNearSetpoint()) {
          mState = State.CRUISING;
        }
        break;
      case CRUISING:
        mLeft.setVoltageVelocitySetpoint(mVelocitySetpoint, true, 0);
        break;
    }
  }

  /**
   * @return {@link State} representing the current mode of being subsystem is in
   */
  @Logged(name = "State", importance = Importance.INFO)
  public State getState() {
    return mState;
  }

  /**
   * @return {@link AngularVelocity} representing the angular velocity of flywheel
   */
  @Logged(name = "Angular Velocity", importance = Importance.INFO)
  public AngularVelocity getAngularVelocity() {
    return null;
  }

  /**
   * @return {@link AngularVelocity} representing the setpoint angular velocity of flywheel
   */
  @Logged(name = "Setpoint Angular Velocity", importance = Importance.INFO)
  public AngularVelocity getAngularVelocitySetpoint() {
    return mVelocitySetpoint;
  }

  /**
   * @return true when the percent error from the setpoint angular velocity is less than the epsilon
   */
  @Logged(name = "is Near Setpoint", importance = Importance.INFO)
  public boolean isNearSetpoint() {
    return getAngularVelocity().in(RadiansPerSecond) / mVelocitySetpoint.in(RadiansPerSecond)
        > Constants.kEpsilon;
  }

  public void accelerateTo(AngularVelocity velocity) {
    mVelocitySetpoint = velocity;
    mState = State.ACCELERATING;
  }

  public void coast() {
    mState = State.COASTING;
  }

  /** Represents a mode of being the {@link Flywheel} subsystem can be in */
  public static enum State {
    /** {@link State} where the {@link Flywheel} is freely spinning with no forces applied */
    COASTING,
    /** {@link State} where the {@link Flywheel} is accelerating to a specified setpoint */
    ACCELERATING,
    /** {@link State} where the {@link Flywheel} is spinning at a specified setpoint */
    CRUISING
  }
}
