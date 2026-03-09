// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.lang.reflect.Array;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystem.vision.CameraIO;

// TODO document
public class RobotState {
  private static RobotState kInstance;

  /**
   * Get {@link RobotState} singleton instance
   *
   * @return {@link RobotState}
   */
  public static RobotState getInstance() {
    if (kInstance == null) kInstance = new RobotState();
    return kInstance;
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  private static final double kPoseBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> kOdometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private Pose2d mOdometryPose = Pose2d.kZero;
  private Pose2d mEstimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> mPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(kPoseBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private final SwerveDriveKinematics mKinematics = Flags.kDriveConstants.getKinematics();
  private SwerveModulePosition[] mPreviousWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d mYawOffset = Rotation2d.kZero;

  private ChassisSpeeds mSpeeds = new ChassisSpeeds();

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(kOdometryStateStdDevs.get(i, 0), 2));
    }
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Get estimated position of robot from odometry/vision
   *
   * @return {@link Pose2d}
   */
  public Pose2d getEstimatedPosition() {
    return mEstimatedPose;
  }

  /**
   * Get estimated speeds of robot (field relative)
   *
   * @return {@link ChassisSpeeds}
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        getChassisSpeeds(), getEstimatedPosition().getRotation());
  }

  /**
   * Get estimated speeds of robot
   *
   * @return {@link ChassisSpeeds}
   */
  public ChassisSpeeds getChassisSpeeds() {
    return mSpeeds;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  /**
   * Reset estimated position to a specified position on the fiel
   *
   * <p>The method will automatically account for the gyro's rotation
   *
   * @param pose {@link Pose2d} Field position to reset to
   */
  public void resetPose(Pose2d pose) {
    mYawOffset = pose.getRotation().minus(mOdometryPose.getRotation().minus(mYawOffset));
    mEstimatedPose = pose;
    mOdometryPose = pose;
    mPoseBuffer.clear();
  }

  /**
   * Set measured speeds of robot
   *
   * @param speeds {@link ChassisSpeeds} Speeds of drivetrain robot relative
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    mSpeeds = speeds;
  }

  /**
   * Modify position estimates based on an odometry sample
   *
   * @param measurement {@link OdometryMeasurement} Sample
   */
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    // Calculate diff in pose
    var twist = mKinematics.toTwist2d(mPreviousWheelPositions, measurement.wheelPositions());
    mPreviousWheelPositions = measurement.wheelPositions();

    // Modify the odometry pose
    var previousOdoPose = mOdometryPose;
    mOdometryPose = mOdometryPose.exp(twist);

    // Override with gyro if connected
    measurement.gyroAngle.ifPresent(
        (yaw) -> {
          Rotation2d angle = yaw.plus(mYawOffset);
          mOdometryPose = new Pose2d(mOdometryPose.getTranslation(), angle);
        });

    // Add position measurement to buffer
    mPoseBuffer.addSample(measurement.timestampSeconds(), mOdometryPose);

    // Calculate diff in position from last odometry pose and modify estimated pose
    var finalTwist = previousOdoPose.log(mOdometryPose);
    mEstimatedPose = mEstimatedPose.exp(finalTwist);
  }

  /**
   * Modify position estimate based on a camera estimated position
   *
   * @param estimate {@link VisionEstimate} Position estimate
   */
  public void addVisionEstimate(VisionEstimate estimate) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (mPoseBuffer.getInternalBuffer().lastKey() - kPoseBufferSizeSec
          > estimate.timestampSeconds()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = mPoseBuffer.getSample(estimate.timestampSeconds());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), mOdometryPose);
    var odometryToSampleTransform = new Transform2d(mOdometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = mEstimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = estimate.stdDevs.get(i, 0) * estimate.stdDevs.get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, estimate.positionEstimate);
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    mEstimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  // * ~~~~~~~~ STRUCTS ~~~~~~~~

  /**
   * Measurement of drivetrain encoders to estimate position
   *
   * @param wheelPositions {@link Array} of {@link SwerveModulePosition} Position of wheels
   * @param gyroAngle {@link Optional} {@link Rotation2d} Measurement can supply a gyro angle
   *     measurement to override angle derived from odometry math
   * @param timestampSeconds {@link Double} FPAG timestamp when the measurment was taken
   */
  public record OdometryMeasurement(
      double timestampSeconds,
      SwerveModulePosition[] wheelPositions,
      Optional<Rotation2d> gyroAngle) {}

  /**
   * Position estimate calculated by a {@link CameraIO}
   *
   * @param timestampSeconds {@link Double} FPAG timemstamp approximating when the robot was at this
   *     position
   * @param positionEstimate {@link Pose2d} Position estimation in 2-Dimensional Space (x, y)
   * @param stdDevs {@link Matrix} of length {@link N3} Standard Deviations of estimation in the x,
   *     y, & z directions
   */
  public record VisionEstimate(
      double timestampSeconds, Pose2d positionEstimate, Matrix<N3, N1> stdDevs) {}
}
