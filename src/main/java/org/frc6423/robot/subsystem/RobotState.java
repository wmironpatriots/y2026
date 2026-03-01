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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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
import org.frc6423.lib.util.Tracer;
import org.frc6423.robot.Constants.Flags;

/** A singleton that tracks the robot's estimated position */
public class RobotState {
  /** {@link RobotState} internal constants */
  private static class Constants {
    /** {@link Matrix} representing expected position estimate error */
    private static final Matrix<N4, N1> kPoseEstimateStdevs = VecBuilder.fill(0.6, 0.6, 0.07, 0.0);

    /** {@link Double} representing how long odometry estimated position remain in buffer */
    private static final double kBufferDuration = 1.5;
  }

  private Pose3d mPreviousOdoPose = new Pose3d();
  private Pose3d mOdoPose = new Pose3d();
  private Pose3d mEstPose = new Pose3d();
  private final TimeInterpolatableBuffer<Pose3d> mOdoPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(Constants.kBufferDuration);

  private final Field2d mF2d = new Field2d();

  private Rotation2d mOffset = Rotation2d.kZero;

  private final SwerveDriveKinematics mKinematics;
  private SwerveModulePosition[] mPreviousSwerveModulePoses;

  /** Create new {@link RobotState} */
  public RobotState() {
    mPreviousSwerveModulePoses = new SwerveModulePosition[4];
    for (int i = 0; i < mPreviousSwerveModulePoses.length; i++) {
      mPreviousSwerveModulePoses[i] = new SwerveModulePosition();
    }

    mKinematics = Flags.kRobotType.mDriveConstants.getKinematics();

    SmartDashboard.putData(mF2d);
  }

  /**
   * @return {@link Rotation3d} representing the estimated robot rotation in 3d space
   */
  public Rotation3d getRotation3d() {
    return getPose3d().getRotation();
  }

  /**
   * @return {@link Pose3d} representing the estimated robot position in 3d space
   */
  public Pose3d getPose3d() {
    return mEstPose;
  }

  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  public Translation2d getTranslation2d() {
    return getPose2d().getTranslation();
  }

  public void resetPose(Pose2d pose) {
    mOffset = pose.getRotation().minus(mOdoPose.getRotation().toRotation2d().minus(mOffset));
    mEstPose = new Pose3d(pose);
    mOdoPose = new Pose3d(pose);
    mOdoPoseBuffer.clear();
  }

  public void addOdometryMeasurement(OdometryMeasurement sample) {
    Tracer.traceFunc(
        "RecordOdometryMeasurement",
        () -> {
          // Calculate the change in distance of swerve module poses and apply to odometry pose
          Twist3d odoPoseTwist =
              toTwist3d(
                  mKinematics.toTwist2d(mPreviousSwerveModulePoses, sample.swerveModulePoses()));
          mPreviousSwerveModulePoses = sample.swerveModulePoses;
          mOdoPose = mOdoPose.exp(odoPoseTwist);

          // Utilize gyro measurements if present
          sample.gyroRotation3d.ifPresent(
              r ->
                  mOdoPose =
                      new Pose3d(mOdoPose.getTranslation(), r.plus(new Rotation3d(mOffset))));

          // Add odometry sample of specified timestamp to odo buffer
          mOdoPoseBuffer.addSample(Constants.kBufferDuration, mOdoPose);

          // Calculate change in distance between odometry positions and apply to estimated pose
          Twist3d estPoseTwist = mPreviousOdoPose.log(mOdoPose);
          mEstPose = mEstPose.exp(estPoseTwist);
          mF2d.setRobotPose(mEstPose.toPose2d());
        });
  }

  public void addVisionMeasurement(VisionMeasurement... measurements) {
    for (var measurement : measurements) {
      Tracer.traceFunc(
          "RecordVisionMeasurement",
          () -> {
            // exit if sample is too old or there are no recent odometry samples
            if (mOdoPoseBuffer.getInternalBuffer().isEmpty()
                || mOdoPoseBuffer.getInternalBuffer().lastKey() - Constants.kBufferDuration
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
              if (Constants.kPoseEstimateStdevs.get(row, 0) == 0.0) {
                visionK.set(row, row, 0.0);
              } else {
                visionK.set(
                    row,
                    row,
                    Constants.kPoseEstimateStdevs.get(row, 0)
                        / (Constants.kPoseEstimateStdevs.get(row, 0)
                            + Math.sqrt(Constants.kPoseEstimateStdevs.get(row, 0) * r[row])));
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
   * Convert a specified {@link Twist2d} to a {@link Twist3d}
   *
   * @param twist2d {@link Twist2d} to convert
   * @return {@link Twist3d}
   */
  public static Twist3d toTwist3d(Twist2d twist2d) {
    return new Twist3d(twist2d.dx, twist2d.dy, 0.0, 0.0, 0.0, twist2d.dtheta);
  }

  /**
   * Represents a swerve drive position measurement using encoders
   *
   * @param timestampSeconds the timestamp when the measurement was taken
   * @param swerveModulePoses {@link SwerveModulePosition} array representing the measured positions
   *     of swerve modules
   * @param gyroRotation3d {@link Rotation3d} representing the orientation of gyro in 3D space
   */
  public record OdometryMeasurement(
      double timestampSeconds,
      SwerveModulePosition[] swerveModulePoses,
      Optional<Rotation3d> gyroRotation3d) {}

  /**
   * Represents a vision position estimation in 3d space
   *
   * @param timestampSeconds timestamp estimation was measured at
   * @param pose3d {@link Pose3d} representing the estimated position in 3d space
   * @param stdevsMatrix {@link Matrix} of 3x1 dimension representing standard deviations of pose
   *     estimation
   */
  public record VisionMeasurement(
      double timestampSeconds, Pose3d pose3dMeasurement, Matrix<N3, N1> measurementStdevs) {}
}
