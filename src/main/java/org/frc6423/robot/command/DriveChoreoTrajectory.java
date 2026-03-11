// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.command;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystem.RobotState;
import org.frc6423.robot.subsystem.drive.DriveSubsystem;

/** Static {@link Command}s for driving drivetrain with autonomous controls */
public class DriveChoreoTrajectory extends Command {
  private static final RobotState kRobotState = RobotState.getInstance();

  private final Timer mTimer = new Timer();
  private final Supplier<Pose2d> mPoseSupplier = kRobotState::getEstimatedPosition;
  private final Supplier<ChassisSpeeds> mSpeedsSupplier = kRobotState::getChassisSpeeds;
  private final DriveSubsystem mDrive;
  private final Trajectory<SwerveSample> mTrajectory;
  private final boolean mMirror;

  private final Pose2d[] mTrajectoryPoses;

  /**
   * Follows a raw choreo trajectory until exeeding estimated time of completion or setpoints
   *
   * @param drive {@link Drive} Drive subsystem to run setpoints
   * @param trajectory {@link Trajectory} of {@link SwerveSample} Trajectory for command to follow
   * @param mirror {@link Boolean} Whether trajectory should be flipped to account for alliance
   */
  public DriveChoreoTrajectory(
      DriveSubsystem drive, Trajectory<SwerveSample> trajectory, boolean mirror) {
    mDrive = drive;
    mTrajectory = trajectory;
    mMirror = mirror;

    mTrajectoryPoses = trajectory.getPoses();
    addRequirements(mDrive);
  }

  @Override
  public void initialize() {
    mTimer.restart();

    DriveFeedbackControllers.reset();
  }

  @Override
  public void execute() {
    // Get current state
    var pose = mPoseSupplier.get();
    var speeds = mSpeedsSupplier.get();

    // Get trajectory sample
    Optional<SwerveSample> sample = mTrajectory.sampleAt(mTimer.get(), mMirror);

    sample.ifPresentOrElse(
        (s) -> {
          // Derive setpoint
          var setpointSpeedsFeedforward = s.getChassisSpeeds();
          var setpointSpeedsFeedback =
              new ChassisSpeeds(
                  DriveFeedbackControllers.calculateTranslationalX(mDrive, pose.getX(), s.x),
                  DriveFeedbackControllers.calculateTranslationalY(mDrive, pose.getY(), s.y),
                  DriveFeedbackControllers.calculateAngular(
                      mDrive, pose.getRotation().getRadians(), s.heading));

          var setpointSpeeds = setpointSpeedsFeedforward.plus(setpointSpeedsFeedback);
          SwerveModuleState[] setpointStates =
              Flags.kDriveConstants.getKinematics().toSwerveModuleStates(setpointSpeeds);

          var xForces = s.moduleForcesX();
          var yForces = s.moduleForcesY();

          double[] wheelTorquesNm = new double[setpointStates.length];
          for (int i = 0; i < setpointStates.length; i++) {
            // Get setpoint angle
            var angle = setpointStates[i].angle;

            // Calculate desired force vector accounting for chassis orientation
            var forceVec =
                new Translation2d(xForces[i], yForces[i])
                    .rotateBy(Rotation2d.fromRadians(s.heading).unaryMinus())
                    .toVector();
            var forceUVec = VecBuilder.fill(angle.getCos(), angle.getSin());

            // Convert desired force vector into wheel torque
            wheelTorquesNm[i] =
                forceVec.dot(forceUVec) * Flags.kDriveConstants.getWheelRadiusMeters();
          }

          // Send setpoints
          mDrive.setSetpointWheelStates(setpointStates, wheelTorquesNm);
        },
        () -> {
          end(false);
        });
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(mTrajectory.getTotalTime());
  }

  /**
   * Get array of positions representing the trajectory
   *
   * @return {@link Array} of {@link Pose2d}
   */
  public Pose2d[] getTrajectory() {
    return mTrajectoryPoses;
  }
}
