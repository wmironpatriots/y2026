// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public static class CameraConfig {
        public final String cameraName;
        public final Transform3d robotToCam;

        public CameraConfig(String cameraName, Transform3d robotToCam) {
            this.cameraName = cameraName;
            this.robotToCam = robotToCam;
        }
    }

    private static class CameraBundle {
        private final PhotonCamera camera;
        private final PhotonPoseEstimator poseEstimator;

        private CameraBundle(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
            this.camera = camera;
            this.poseEstimator = poseEstimator;
        }
    }

    private final List<CameraBundle> cameras;
    private List<EstimatedRobotPose> latestEstimates = List.of();
   
    public Vision(String cameraName, Transform3d robotToCam) {
        this(new CameraConfig(cameraName, robotToCam));
    }

    public Vision(CameraConfig... configs) {

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        List<CameraBundle> cameraBundles = new ArrayList<>(configs.length);
        Arrays.stream(configs)
            .filter(config -> config != null)
            .forEach(config -> {
                PhotonCamera camera = new PhotonCamera(config.cameraName);
                PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
                    layout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    config.robotToCam
                );
                poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                cameraBundles.add(new CameraBundle(camera, poseEstimator));
            });

        cameras = Collections.unmodifiableList(cameraBundles);
    }

    @Override
    public void periodic() {
        List<EstimatedRobotPose> estimates = new ArrayList<>();

        for (CameraBundle camera : cameras) {
            PhotonPipelineResult result = camera.camera.getLatestResult();
            if (!result.hasTargets()) {
                continue;
            }

            camera.poseEstimator.update(result)
                .filter(est -> isEstimateUsable(est, result))
                .ifPresent(estimates::add);
        }

        latestEstimates = Collections.unmodifiableList(estimates);
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
}

