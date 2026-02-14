// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.frc6423.lib.util.CoordUtil;
import org.frc6423.lib.util.Tracer;
import org.frc6423.robot.Constants.Flags;

/** A class for tracking the robot on the field */
public class RobotState {
  /** {@link Matrix} representing the standard deviations of vision in the x, y, z, yaw */
  private static final Matrix<N4, N1> kPoseEstimateStdevs = VecBuilder.fill(0.6, 0.6, 0.07, 0.0);

  /** Represents how long a pose estimate should remain in the buffer */
  public static final double kBufferDurationSec = 1.5;

  private final SwerveDriveKinematics mKinematics;
  private SwerveModulePosition[] mPreviousSwerveModulePoses;

  private Pose3d mPreviousOdoPose = new Pose3d();
  private Pose3d mOdoPose = new Pose3d();
  private Pose3d mEstPose = new Pose3d();
  private final TimeInterpolatableBuffer<Pose3d> mOdoPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferDurationSec);

  private Rotation2d mOffset = Rotation2d.kZero;

  private final Field2d mF2d = new Field2d();

  /** Create new RobotState */
  public RobotState() {
    var constants = Flags.kDriveConstants;

    mPreviousSwerveModulePoses =
        new SwerveModulePosition[constants.getModuleDisplacements().length];
    for (int i = 0; i < mPreviousSwerveModulePoses.length; i++) {
      mPreviousSwerveModulePoses[i] = new SwerveModulePosition();
    }

    mKinematics = constants.getKinematics();

    SmartDashboard.putData(mF2d);
  }

  /**
   * @return {@link Rotation2d} representing the estimated robot rotation in 2d space (yaw)
   */
  @Logged(name = "Estimated Robot Rotation (2d)", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return getPose2d().getRotation();
  }

  /**
   * @return {@link Rotation3d} representing the estimated robot rotation in 3d space (yaw, pitch,
   *     roll)
   */
  @Logged(name = "Estimated Robot Rotation (3d)", importance = Importance.INFO)
  public Rotation3d getRotation3d() {
    return getPose3d().getRotation();
  }

  /**
   * @return {@link Pose2d} representing the estimated robot position in 2d space (x, y)
   */
  @Logged(name = "Estimated Robot Pose (2d)", importance = Importance.INFO)
  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  /**
   * @return {@link Pose3d} representing the estimated robot position in 3d space (x, y, z)
   */
  @Logged(name = "Estimated Robot Pose (3d)", importance = Importance.INFO)
  public Pose3d getPose3d() {
    return mEstPose;
  }

  /**
   * Reset estimated robot position to specified position
   *
   * @param pose {@link Pose2d} to reset estimated position to
   */
  public void resetPose(Pose2d pose) {
    // Account for gyro
    mOffset = pose.getRotation().minus(mOdoPose.getRotation().toRotation2d().minus(mOffset));

    mEstPose = new Pose3d(pose);
    mOdoPose = new Pose3d(pose);
    mOdoPoseBuffer.clear();
  }

  /**
   * Adjust estimated position based off of swerve drive odometry measurements
   *
   * @param sample {@link OdometrySample} representing an odometry measurement
   */
  public void addOdometryMeasurement(OdometrySample sample) {
    Tracer.traceFunc(
        "RecordOdometryMeasurement",
        () -> {
          // Calculate the change in distance of swerve module poses and apply to odometry pose
          Twist3d odoPoseTwist =
              CoordUtil.toTwist3d(
                  mKinematics.toTwist2d(mPreviousSwerveModulePoses, sample.swerveModulePoses()));
          mPreviousSwerveModulePoses = sample.swerveModulePoses;
          mOdoPose = mOdoPose.exp(odoPoseTwist);

          // Utilize gyro measurements if present
          sample.gyroRotation3d.ifPresent(r -> mOdoPose = new Pose3d(mOdoPose.getTranslation(), r));

          // Add odometry sample of specified timestamp to odo buffer
          mOdoPoseBuffer.addSample(kBufferDurationSec, mOdoPose);

          // Calculate change in distance between odometry positions and apply to estimated pose
          Twist3d estPoseTwist = mPreviousOdoPose.log(mOdoPose);
          mEstPose.exp(estPoseTwist);
          mF2d.setRobotPose(mEstPose.toPose2d());
        });
  }

  /**
   * Adjust estimated position based off of vision position measurement
   *
   * @param measurements {@link VisionSample} {@link Array} representing new vision position
   *     estimations
   */
  public void addVisionMeasurement(VisionSample... measurements) {
    for (var measurement : measurements) {
      Tracer.traceFunc(
          "RecordVisionMeasurement",
          () -> {
            // exit if sample is too old or there are no recent odometry samples
            if (mOdoPoseBuffer.getInternalBuffer().isEmpty()
                || mOdoPoseBuffer.getInternalBuffer().lastKey() - kBufferDurationSec
                    > measurement.timestampSeconds) {
              return;
            }

            // Get odo sample at timestamp; exit if nonexistent
            var odoSample = mOdoPoseBuffer.getSample(measurement.timestampSeconds);
            if (odoSample.isEmpty()) {
              return;
            }

            var odoToSample = new Transform3d(mOdoPose, odoSample.get());

            var estPoseAtTimestamp = mEstPose.plus(odoToSample);

            Matrix<N6, N6> visionK = new Matrix<>(Nat.N6(), Nat.N6());

            var r = new double[3];

            // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
            // and C = I. See WPIMath/algorithms.md.
            for (int row = 0; row < 4; ++row) {
              if (kPoseEstimateStdevs.get(row, 0) == 0.0) {
                visionK.set(row, row, 0.0);
              } else {
                visionK.set(
                    row,
                    row,
                    kPoseEstimateStdevs.get(row, 0)
                        / (kPoseEstimateStdevs.get(row, 0)
                            + Math.sqrt(kPoseEstimateStdevs.get(row, 0) * r[row])));
              }
            }
            // Fill in the gains for the other components of the rotation vector
            double angle_gain = visionK.get(3, 3);
            visionK.set(4, 4, angle_gain);
            visionK.set(5, 5, angle_gain);

            var transform = new Transform3d(estPoseAtTimestamp, measurement.pose3dMeasurement);

            // Step 5: We should not trust the transform entirely, so instead we scale this
            // transform by a
            // Kalman
            // gain matrix representing how much we trust vision measurements compared to our
            // current pose.
            var transformTimesK =
                visionK.times(
                    VecBuilder.fill(
                        transform.getX(),
                        transform.getY(),
                        transform.getZ(),
                        transform.getRotation().getX(),
                        transform.getRotation().getY(),
                        transform.getRotation().getZ()));

            // Step 6: Convert back to Transform3d.
            var scaledTransform =
                new Transform3d(
                    transformTimesK.get(0, 0),
                    transformTimesK.get(1, 0),
                    transformTimesK.get(2, 0),
                    new Rotation3d(
                        transformTimesK.get(3, 0),
                        transformTimesK.get(4, 0),
                        transformTimesK.get(5, 0)));

            mEstPose = estPoseAtTimestamp.plus(scaledTransform).plus(transform.inverse());
            mF2d.setRobotPose(mEstPose.toPose2d());
          });
    }
  }

  /**
   * Represents a swerve drive position measurement using encoders
   *
   * @param timestampSeconds the timestamp when the measurement was taken
   * @param swerveModulePoses {@link SwerveModulePosition} array representing the measured positions
   *     of swerve modules
   * @param gyroRotation3d {@link Rotation3d} representing the orientation of gyro in 3D space
   */

  /**
   * Represents position measurements of swerve modules & gyro at a specific timestamp
   *
   * @param timestampSeconds the timestamp representing when the sample was recorded
   * @param swerveModulePoses {@link SwerveModulePosition} {@link Array} representing the measured
   *     positions of modules
   * @param gyroRotation3d {@link Rotation3d} representing the orientation of gyro in 3D space (yaw,
   *     pitch, roll)
   */
  public record OdometrySample(
      double timestampSeconds,
      SwerveModulePosition[] swerveModulePoses,
      Optional<Rotation3d> gyroRotation3d) {}

  /**
   * Represents position measurement of cameras
   *
   * @param timestampSeconds timestamp representing when the sample was recorded
   * @param pose3d {@link Pose3d} representing the estimated position in 3d space (x, y, z)
   * @param stdevsMatrix {@link Matrix} of 3x1 dimension representing standard deviations of pose
   *     estimation
   */
  public record VisionSample(
      double timestampSeconds, Pose3d pose3dMeasurement, Matrix<N3, N1> measurementStdevs) {}
}
