// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive.component;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.util.Queue;
import org.frc6423.robot.subsystem.drive.PhoneixOdometryThread;

/** Interface for interacting with {@link Pigeon2} gyro hardware */
public class GyroIOPigeon2 extends GyroIO {
  private final Pigeon2 mPigeon;

  private final BaseStatusSignal mYawSignal;

  private final Queue<Double> mYawPositionQueue;
  private final Queue<Double> mYawTimestampQueue;

  public GyroIOPigeon2(int canDeviceId, CANBus canBus, Pigeon2Configuration config) {
    mPigeon = new Pigeon2(canDeviceId, canBus);
    mPigeon.getConfigurator().apply(config);
    mPigeon.getConfigurator().setYaw(0.0);

    mYawSignal = mPigeon.getYaw();
    mYawSignal.setUpdateFrequency(PhoneixOdometryThread.kFrequencyHz);

    // TODO do we need to specify the freq?
    mPigeon.optimizeBusUtilization();

    mYawTimestampQueue = PhoneixOdometryThread.getInstance().makeTimestampQueue();
    mYawPositionQueue = PhoneixOdometryThread.getInstance().registerSignal(mPigeon.getYaw());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(mYawSignal);
  }

  @Override
  public double[] getYawRotationsRads() {
    var result = mYawPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    mYawPositionQueue.clear();

    return result;
  }

  @Override
  public double[] getYawTimestampsSec() {
    var result = mYawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    mYawTimestampQueue.clear();

    return result;
  }
}
