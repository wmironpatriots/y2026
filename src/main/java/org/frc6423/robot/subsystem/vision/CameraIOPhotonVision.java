// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import static org.frc6423.robot.subsystem.vision.CameraConstants.kDistanceFactor;
import static org.frc6423.robot.subsystem.vision.CameraConstants.kFieldLayout;
import static org.frc6423.robot.subsystem.vision.CameraConstants.kInfiniteDevs;
import static org.frc6423.robot.subsystem.vision.CameraConstants.kPointBlankStdevs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.Optional;
import org.frc6423.robot.RobotState.VisionMeasurement;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

// TODO sim implmentation
public class CameraIOPhotonVision extends CameraIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator pvPoseEstimator;

  public CameraIOPhotonVision(String cameraId, Transform3d cameraDisplacementWrtRobot) {
    this.camera = new PhotonCamera(cameraId);
    this.pvPoseEstimator = new PhotonPoseEstimator(kFieldLayout, cameraDisplacementWrtRobot);
  }

  @Override
  protected void periodic() {}

  @Override
  public VisionMeasurement[] getUnreadVisionMeasurements() {
    ArrayList<VisionMeasurement> measurements = new ArrayList<>();
    var results = camera.getAllUnreadResults();

    for (var result : results) {
      var pvEstPose = getEstimatedRobotPose(result);

      pvEstPose.ifPresent(
          e ->
              measurements.add(
                  new VisionMeasurement(
                      e.timestampSeconds, e.estimatedPose, getMeasurementStdevs(e))));
    }

    return measurements.toArray(VisionMeasurement[]::new);
  }

  protected Optional<EstimatedRobotPose> getEstimatedRobotPose(PhotonPipelineResult result) {
    // Return empty if there are no new targets
    if (result.getTargets().isEmpty()) {
      return Optional.empty();
    }

    return pvPoseEstimator.estimateCoprocMultiTagPose(result);
  }

  /**
   * Calculates the Standard Deviations of a estimated robot pose
   *
   * @param estimation {@link EstimatedRobotPose} estimated pose to calculate stdevs for
   * @return {@link Matrix} of {@link N3}, {@link N1} dimensions
   */
  public static Matrix<N3, N1> getMeasurementStdevs(EstimatedRobotPose estimation) {
    double sumDistance = 0;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      sumDistance +=
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    }
    double avgDistance = sumDistance / estimation.targetsUsed.size();

    var deviation = kPointBlankStdevs.times(Math.max(avgDistance, 0.0) * kDistanceFactor);
    if (estimation.targetsUsed.size() == 1) {
      deviation = deviation.times(3);
    }
    if (estimation.targetsUsed.size() == 1 && estimation.targetsUsed.get(0).poseAmbiguity > 0.15) {
      return kInfiniteDevs;
    }
    // Reject if estimated pose is in the air or ground
    if (Math.abs(estimation.estimatedPose.getZ()) > 0.125) {
      return kInfiniteDevs;
    }

    return deviation;
  }
}
