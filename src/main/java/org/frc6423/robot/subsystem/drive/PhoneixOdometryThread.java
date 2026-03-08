// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.frc6423.robot.Constants.Flags;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoneixOdometryThread extends Thread {
  private static PhoneixOdometryThread kInstance;

  /**
   * Get {@link PhoneixOdometryThread} singleton instance
   *
   * @return {@link PhoneixOdometryThread}
   */
  public static PhoneixOdometryThread getInstance() {
    if (kInstance == null) kInstance = new PhoneixOdometryThread();
    return kInstance;
  }

  public static final double kFrequencyHz = 250.0;

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private final Lock mLock = new ReentrantLock();
  private BaseStatusSignal[] mSignals = new BaseStatusSignal[0];
  private final List<Queue<Double>> mPhoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> mTimestampQueues = new ArrayList<>();

  private static boolean mIsCanFd = Flags.kDriveConstants.getCANBus().isNetworkFD();

  protected PhoneixOdometryThread() {
    setName("OdometryThread");
    setDaemon(true);
  }

  @Override
  public synchronized void start() {
    if (mTimestampQueues.size() > 0) super.start();
    ;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  /**
   * Register a Phoneix6 {@link BaseStatusSignal} to be read by thread
   *
   * @param signal {@link BaseStatusSignal} Phoneix6 signal to register
   * @return {@link Queue} of {@link Double}
   */
  public Queue<Double> registerSignal(BaseStatusSignal signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    mLock.lock();
    Drive.kLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[mSignals.length + 1];
      System.arraycopy(mSignals, 0, newSignals, 0, mSignals.length);
      newSignals[mSignals.length] = signal;
      mSignals = newSignals;
      mPhoenixQueues.add(queue);
    } finally {
      Drive.kLock.unlock();
      mLock.unlock();
    }
    return queue;
  }

  /**
   * Get a {@link Queue} returning timestamp values for each sampe
   *
   * @return {@link Queue} of {@link Double}
   */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.kLock.lock();
    try {
      mTimestampQueues.add(queue);
    } finally {
      Drive.kLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    // ! Real-Time thread priority
    // @see https://blogs.oracle.com/linux/task-priority
    // @see
    // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#real-time-thread-priority
    Threads.setCurrentThreadPriority(true, 1);

    while (true) {
      // Wait for updates from all BaseStatusSignals
      mLock.lock();
      try {
        if (mIsCanFd && mSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / kFrequencyHz, mSignals);
        } else {
          // "waitForAll" does not support blocking on multiple signals with a bus
          // that is not CAN FD, regardless of Pro licensing. No reasoning for this
          // behavior is provided by the documentation.
          Thread.sleep((long) (1000.0 / kFrequencyHz));
          if (mSignals.length > 0) BaseStatusSignal.refreshAll(mSignals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        mLock.unlock();
      }

      // Save new data to queues
      Drive.kLock.lock();
      try {
        // Sample timestamp is current FPGA time minus average CAN latency
        //     Default timestamps from Phoenix are NOT compatible with
        //     FPGA timestamps, this solution is imperfect but close
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (var signal : mSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (mSignals.length > 0) {
          timestamp -= totalLatency / mSignals.length;
        }

        // Add new samples to queues
        for (int i = 0; i < mSignals.length; i++) {
          mPhoenixQueues.get(i).offer(mSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < mTimestampQueues.size(); i++) {
          mTimestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.kLock.unlock();
      }
    }
  }
}
