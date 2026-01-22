// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
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

import java.util.Optional;

import org.frc6423.lib.util.Tracer;

/** 
 * A class for keeping track of:
 * 
 * </p> * The estimated position of robot on field
 * </p> * The 
 */
public class RobotState {
  // * CONSTANTS
  public static final Matrix<N4, N1> kPoseEstimateStdevs = VecBuilder.fill(0.6, 0.6, 0.07, 0.0);

  public static final double kBufferDuration = 1.5;

  public static RobotState kInstance;

  /**
   * @return {@link RobotState} singleton instance
   */
  public static RobotState getInstace() {
    if (kInstance == null) kInstance = new RobotState();

    return kInstance;
  }

  private Pose3d mPreviousOdoPose;
  private Pose3d mOdoPose;
  private Pose3d mEstPose;
  private final TimeInterpolatableBuffer<Pose3d> mOdoPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferDuration);

  private final SwerveDriveKinematics mKinematics;
  private SwerveModulePosition[] mPreviousSwerveModulePoses;

  /** Create new {@link RobotState} */
  private RobotState() {
    // TODO values get from drive constants
    var centerToEdge = Inches.of(10);
    mKinematics =
        new SwerveDriveKinematics(
            new Translation2d[] {
              new Translation2d(centerToEdge, centerToEdge).times(-1),
              new Translation2d(centerToEdge, centerToEdge),
              new Translation2d(centerToEdge, centerToEdge),
              new Translation2d(centerToEdge, centerToEdge),
            });

    mPreviousSwerveModulePoses = new SwerveModulePosition[4];
  }

  /**
   * @return {@link Rotation3d} representing the estimated robot rotation in 3d space
   */
  @Logged(name = "Rotation3d", importance = Importance.INFO)
  public Rotation3d getRotation3d() {
    return getPose3d().getRotation();
  }

  /**
   * @return {@link Pose3d} representing the estimated robot position in 3d space
   */
  @Logged(name = "Pose3d", importance = Importance.INFO)
  public Pose3d getPose3d() {
    return mEstPose;
  }

  public void addOdometryMeasurement(OdometryMeasurement sample) {
    Tracer.traceFunc("RecordOdometryMeasurement", () -> {
      // Calculate the change in distance of swerve module poses and apply to odometry pose
      Twist3d odoPoseTwist =
          toTwist3d(mKinematics.toTwist2d(mPreviousSwerveModulePoses, sample.swerveModulePoses()));
      mPreviousSwerveModulePoses = sample.swerveModulePoses;
      mOdoPose = mOdoPose.exp(odoPoseTwist);

      // Utilize gyro measurements if present
      sample.gyroRotation3d.ifPresent(r -> mOdoPose = new Pose3d(mOdoPose.getTranslation(), r));

      // Add odometry sample of specified timestamp to odo buffer
      mOdoPoseBuffer.addSample(kBufferDuration, mOdoPose);

      // Calculate change in distance between odometry positions and apply to estimated pose
      Twist3d estPoseTwist = mPreviousOdoPose.log(mOdoPose);
      mEstPose.exp(estPoseTwist);
    });
  }

  public void addVisionMeasurement(VisionMeasurement... measurements) {
    for (var measurement : measurements) {
      Tracer.traceFunc("RecordVisionMeasurement", () -> {
        // exit if sample is too old or there are no recent odometry samples
        if (mOdoPoseBuffer.getInternalBuffer().isEmpty()
            || mOdoPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration
                > measurement.timestampeSeconds) {
          return;
        }

        // Get odo sample at timestamp; exit if nonexistant
        var odoSample = mOdoPoseBuffer.getSample(measurement.timestampeSeconds);
        if (odoSample.isEmpty()) {
          return;
        }

        var odoToSample = new Transform3d(mOdoPose, odoSample.get());

        var estPoseAtTimestamp = mEstPose.plus(odoToSample);

        Matrix<N6, N6> visionK = new Matrix<>(Nat.N6(), Nat.N6());

        var r = new double[3];

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
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

        // Step 5: We should not trust the transform entirely, so instead we scale this transform by a
        // Kalman
        // gain matrix representing how much we trust vision measurements compared to our current pose.
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
                    transformTimesK.get(3, 0), transformTimesK.get(4, 0), transformTimesK.get(5, 0)));

        mEstPose = estPoseAtTimestamp.plus(scaledTransform).plus(transform.inverse());
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

  public record OdometryMeasurement(
      double timestampSeconds,
      SwerveModulePosition[] swerveModulePoses,
      Optional<Rotation3d> gyroRotation3d) {}

  /**
   * TODO redo
   * Represents a vision position estimation in 3d space
   *
   * @param timestampSeconds timestamp estimation was measured at
   * @param pose3d {@link Pose3d} representing the estimated position in 3d space
   * @param stdevsMatrix {@link Matrix} of 3x1 dimension representing standard deviations of pose
   *     estimation
   */
  public record VisionMeasurement(
      double timestampeSeconds, Pose3d pose3dMeasurement, Matrix<N3, N1> measurementStdevs) {}
}
