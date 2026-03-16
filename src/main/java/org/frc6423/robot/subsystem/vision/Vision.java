// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  public static class CameraConfig {
    public final String cameraName;
    public final Transform3d robotToCam;

    public CameraConfig(String cameraName, Transform3d robotToCam) {
      this.cameraName = cameraName;
      this.robotToCam = robotToCam;
    }
  }

  private static class CameraContext {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private CameraContext(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
      this.camera = camera;
      this.poseEstimator = poseEstimator;
    }
  }

  private final List<CameraContext> cameras;
  private List<EstimatedRobotPose> latestEstimates = List.of();
  private Optional<EstimatedRobotPose> latestBestEstimate = Optional.empty();

  public Vision(String cameraName, Transform3d robotToCam) {
    this(new CameraConfig(cameraName, robotToCam));
  }

  public Vision(CameraConfig... configs) {
    if (configs == null || configs.length == 0) {
      throw new IllegalArgumentException("At least one camera config is required.");
    }

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    List<CameraContext> cameraContexts = new ArrayList<>(configs.length);
    Arrays.stream(configs)
        .filter(config -> config != null)
        .forEach(
            config -> {
              PhotonCamera camera = new PhotonCamera(config.cameraName);
              PhotonPoseEstimator poseEstimator =
                  new PhotonPoseEstimator(
                      layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.robotToCam);
              poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
              cameraContexts.add(new CameraContext(camera, poseEstimator));
            });

    if (cameraContexts.isEmpty()) {
      throw new IllegalArgumentException("At least one valid camera config is required.");
    }

    cameras = Collections.unmodifiableList(cameraContexts);
  }

  @Override
  public void periodic() {
    List<EstimatedRobotPose> estimates = new ArrayList<>();

    for (CameraContext context : cameras) {
      PhotonPipelineResult result = context.camera.getLatestResult();
      if (!result.hasTargets()) {
        continue;
      }

      context
          .poseEstimator
          .update(result)
          .filter(est -> isEstimateUsable(est, result))
          .ifPresent(estimates::add);
    }

    latestEstimates = Collections.unmodifiableList(estimates);
    latestBestEstimate =
        latestEstimates.stream().max(Comparator.comparingDouble(this::estimateScore));
  }

  public Optional<EstimatedRobotPose> getLatestPoseEstimate() {
    return latestBestEstimate;
  }

  public List<EstimatedRobotPose> getLatestPoseEstimates() {
    return latestEstimates;
  }

  public boolean isConnected() {
    return cameras.stream().anyMatch(context -> context.camera.isConnected());
  }

  private boolean isEstimateUsable(EstimatedRobotPose est, PhotonPipelineResult result) {
    // add thresholds and stuff here later
    return true;
  }

  private double estimateScore(EstimatedRobotPose estimate) {
    if (estimate.targetsUsed == null || estimate.targetsUsed.isEmpty()) {
      return 0.0;
    }

    double ambiguitySum = 0.0;
    int ambiguityCount = 0;

    for (PhotonTrackedTarget target : estimate.targetsUsed) {
      double ambiguity = target.getPoseAmbiguity();
      if (ambiguity >= 0.0) {
        ambiguitySum += ambiguity;
        ambiguityCount++;
      }
    }

    double avgAmbiguity = ambiguityCount == 0 ? 1.0 : ambiguitySum / ambiguityCount;
    return estimate.targetsUsed.size() * 10.0 - avgAmbiguity;
  }
}
