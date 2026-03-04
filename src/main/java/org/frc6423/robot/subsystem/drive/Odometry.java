// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import org.frc6423.lib.util.GeometryUtil;
import org.frc6423.lib.util.Tracer;

// TODO cleanup
/**
 * A class for calculating the position of the robot based on odometry inputs (drive encoders,
 * vision localization systems, etc)
 *
 * <p>To feed drive encoder measurements for calculations, use {@link
 * #addEncoderMeasurement(EncoderMeasurement)}
 *
 * <p>To feed vision position estimates for calculations, use {@link
 * #addVisionMeasurement(VisionMeasurement...)}
 */
public class Odometry {
  // * GEOMETRY MEMBERS
  private Pose3d mPreviousOdoPose = new Pose3d();
  private Pose3d mOdoPose = new Pose3d();
  private Pose3d mEstPose = new Pose3d();
  private final double mEstimateBufferSize;
  private final TimeInterpolatableBuffer<Pose3d> mOdoPoseBuffer;

  private Rotation2d mGyroOffset = Rotation2d.kZero;

  private final Matrix<N4, N1> mEstimateStdevs;

  // * KINEMATICS MEMBERS
  private final SwerveDriveKinematics mKinematics;
  private SwerveModulePosition[] mPreviousSwerveModulePoses;

  // * PUBLISHER/VISUALIZER MEMBERS
  private final Field2d mF2d = new Field2d();

  /**
   * Create new {@link Odometry}
   *
   * @param kinematics {@link SwerveDriveKinematics} Kinematics model to use for calculations
   * @param positionEstimateStdevs {@link Matrix} of length {@link N4} Standard deviations of the
   *     pose estimate (x position in meters, y position in meters, z position in meters, and angle
   *     in radians). Increase these numbers to trust your state estimate less.
   * @param odometryBufferSizeSeconds {@link Double} How long odometry estimations should last in
   *     the buffer
   */
  public Odometry(
      SwerveDriveKinematics kinematics,
      Matrix<N4, N1> positionEstimateStdevs,
      double odometryBufferSizeSeconds) {
    mKinematics = kinematics;
    mPreviousSwerveModulePoses = new SwerveModulePosition[4];
    for (int i = 0; i < mPreviousSwerveModulePoses.length; i++) {
      mPreviousSwerveModulePoses[i] = new SwerveModulePosition();
    }

    mEstimateBufferSize = odometryBufferSizeSeconds;
    mOdoPoseBuffer = TimeInterpolatableBuffer.createBuffer(odometryBufferSizeSeconds);

    mEstimateStdevs = positionEstimateStdevs;
    SmartDashboard.putData(mF2d);
  }

  /** Update visualizers */
  public void update() {
    // Update visualizers
    mF2d.setRobotPose(getPose2d());
  }

  /**
   * Get the estimated rotation of the robot in 2-Dimensional Space
   *
   * @return {@link Rotation2d}
   */
  public Rotation2d getRotation2d() {
    return getPose2d().getRotation();
  }

  /**
   * Get the estimated displacement of robot from the origin in 2-Dimensional Space
   *
   * @return {@link Translation2d}
   */
  public Translation2d getTranslation2d() {
    return getPose2d().getTranslation();
  }

  /**
   * Get the estimated position of robot in 2-Dimensional Space
   *
   * @return {@link Pose2d}
   */
  public Pose2d getPose2d() {
    return getPose3d().toPose2d();
  }

  /**
   * Get the estimated rotation of robot in 3-Dimensional Space
   *
   * @return {@link Rotation3d}
   */
  public Rotation3d getRotation3d() {
    return getPose3d().getRotation();
  }

  /**
   * Get the esimated position of robot in 3-Dimensional Space
   *
   * @return {@link Pose3d}
   */
  public Pose3d getPose3d() {
    return mEstPose;
  }

  /**
   * Reset odometry position to a specified point
   *
   * @param pose {@link Pose2d} The position to reset to
   */
  public void resetPose(Pose2d pose) {
    mGyroOffset =
        pose.getRotation().minus(mOdoPose.getRotation().toRotation2d().minus(mGyroOffset));
    mEstPose = new Pose3d(pose);
    mOdoPose = new Pose3d(pose);
    mOdoPoseBuffer.clear();
  }

  public void addEncoderMeasurement(EncoderMeasurement sample) {
    Tracer.traceFunc(
        "RecordOdometryMeasurement",
        () -> {
          // Calculate the change in distance of swerve module poses and apply to odometry pose
          Twist3d odoPoseTwist =
              GeometryUtil.toTwist3d(
                  mKinematics.toTwist2d(mPreviousSwerveModulePoses, sample.swerveModulePoses()));
          mPreviousSwerveModulePoses = sample.swerveModulePoses;
          mOdoPose = mOdoPose.exp(odoPoseTwist);

          // Utilize gyro measurements if present
          sample.gyroRotation3d.ifPresent(
              r ->
                  mOdoPose =
                      new Pose3d(mOdoPose.getTranslation(), r.plus(new Rotation3d(mGyroOffset))));

          // Add odometry sample of specified timestamp to odo buffer
          mOdoPoseBuffer.addSample(sample.timestampSeconds, mOdoPose);

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
                || mOdoPoseBuffer.getInternalBuffer().lastKey() - mEstimateBufferSize
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
              if (mEstimateStdevs.get(row, 0) == 0.0) {
                visionK.set(row, row, 0.0);
              } else {
                visionK.set(
                    row,
                    row,
                    mEstimateStdevs.get(row, 0)
                        / (mEstimateStdevs.get(row, 0)
                            + Math.sqrt(mEstimateStdevs.get(row, 0) * r[row])));
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

  public record EncoderMeasurement(
      double timestampSeconds,
      SwerveModulePosition[] swerveModulePoses,
      Optional<Rotation3d> gyroRotation3d) {}

  public record VisionMeasurement(
      double timestampSeconds, Pose3d pose3dMeasurement, Matrix<N3, N1> measurementStdevs) {}
}
