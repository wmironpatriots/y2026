// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.robot.Constants.Matrix;

/**
 * {@link SubsystemBase} extension representing the feeder subsystem
 *
 * <p>
 * This subsystem's only component is a roller
 *
 * <p>
 * The feeder will only spin towards the shooter
 */
public class Feeder extends SubsystemBase {
    /** {@link Feeder} subsystem constants */
    public class Constants {
        /** {@link CANBus} representing the bus devices are connected to */
        private static final CANBus kCanBus = Matrix.kSubsystemCanBus;

        /** {@link Integer} representing the servo's CAN ID on CANBUS */
        private static final int kServoCanDeviceId = Matrix.kFeederId;

        /**
         * {@link TalonFXConfiguration} representing the hardware config of the servo
         */
        private static final TalonFXConfiguration kServoTalonConfig = new TalonFXConfiguration()
                .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(20.0))
                                .withStatorCurrentLimitEnable(true));

        /** {@link Voltage} representing the feeding speed */
        private static final Voltage kFeedingSpeed = Volts.of(9);

        /**
         * {@link Time} representing the period between pulses when feeder is in pulse
         * mode
         */
        private static final double kPulsePeriodSeconds = 0.2;
    }

    @Logged
    private final ServoIO mServo;

    private State mState = State.STOPPED;

    /**
     * Create new {@link Feeder}
     *
     * @return {@link Feeder}
     */
    public static Feeder create() {
        // TODO sim
        return new Feeder(
                new ServoIOTalonFx(
                        "FeederServo",
                        Constants.kCanBus,
                        Constants.kServoCanDeviceId,
                        Constants.kServoTalonConfig));
    }

    /**
     * Create new {@link Feeder}
     *
     * @param servo {@link ServoIO} representing roller servo
     */
    protected Feeder(ServoIO servo) {
        mServo = servo;
    }

    @Override
    public void periodic() {
        mServo.periodic();

    @Override
    public void periodic() {
        // Update hardware
        mServo.periodic();

        // Run state logic
        switch (mState) {
            case STOPPED:
                mServo.stop();
                break;

            case RUNNING:
                setSpeed(Constants.kFeedingSpeed);
                break;

            case PULSING:
                boolean run = (Timer.getFPGATimestamp() % Constants.kPulsePeriodSeconds)
                        / Constants.kPulsePeriodSeconds > 0.5;
                if (run)
                    setSpeed(Constants.kFeedingSpeed);
                else
                    mServo.stop();
                break;
        }
    }

    /**
     * Set servo speed
     *
     * @param speed {@link Voltage} representing desired feeder speed
     */
    private void setSpeed(Voltage speed) {
        mServo.setVoltageSetpoint(speed, true);
    }

    /**
     * @return {@link State} representing the current mode of being subsystem is in
     */
    @Logged(name = "State", importance = Importance.INFO)
    public State getState() {
        return mState;
    }

    /**
     * Attempt to stop feeder
     *
     * @return {@link Command}
     */
    public Command stop() {
        return Commands.none();
    }

    /**
     * Attempt to run feeder
     *
     * @return {@link Command}
     */
    public Command run() {
        return Commands.none();
    }

    /**
     * Attempt to start pulsing feeder
     *
     * @return {@link Command}
     */
    public Command pulse() {
        return Commands.none();
    }

    /** Represents a mode of being the {@link Feeder} subsystem can be in */
    public static enum State {
        /** {@link State} where the {@link Feeder} is not running */
        STOPPED,
        /** {@link State} where the {@link Feeder} is running */
        RUNNING,
        /** {@link State} where the {@link Feeder} is periodically pulsing */
        PULSING
    }
}
