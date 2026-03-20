// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  /**
   * Static Factory for automatically configuring and creating a {@link VisionSubsystem}
   *
   * @return {@link VisionSubsystem}
   */
  public static VisionSubsystem create() {
    return new VisionSubsystem(kBessie, kBeatrice, kBelinda);
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  public static final CameraConfig kBessie =
      new CameraConfig(
          "bessie",
          new Transform3d(
              new Translation3d(-0.1908, 0, 0.3609),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-10.0), Degrees.of(180))));
  // public static final CameraConfig kElsie =
  //    new CameraConfig(
  //        "elsie",
  //        new Transform3d(
  //            new Translation3d(0.2006, -0.16, 0.5094),
  //            new Rotation3d(Degrees.of(0.0), Degrees.of(-15.0), Degrees.of(0))));
  public static final CameraConfig kBeatrice =
      new CameraConfig(
          "beatrice",
          new Transform3d(
              new Translation3d(0.2006, -0.3114, 0.5094),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-10.0), Degrees.of(-75.0))));
  public static final CameraConfig kBelinda =
      new CameraConfig(
          "belinda",
          new Transform3d(
              new Translation3d(0.2006, 0.3114, 0.5094),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-10.0), Degrees.of(75.0))));

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

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private final List<CameraBundle> cameras;
  private List<EstimatedRobotPose> latestEstimates = List.of();
  private List<Matrix<N3, N1>> curStdDevs;

  // public Vision(String cameraName, Transform3d robotToCam) {
  //    this(new CameraConfig(cameraName, robotToCam));
  // }

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.6, 0.6, 1155);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 1155);
  public static final Matrix<N3, N1> SUPERTRUST_TAG_STD_DEVS = VecBuilder.fill(0.001, 0.001, 0.001);
  private static AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public VisionSubsystem(CameraConfig... configs) {
    List<CameraBundle> cameraBundles = new ArrayList<>(configs.length);
    Arrays.stream(configs)
        .filter(config -> config != null)
        .forEach(
            config -> {
              PhotonCamera camera = new PhotonCamera(config.cameraName);
              PhotonPoseEstimator poseEstimator =
                  new PhotonPoseEstimator(
                      layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.robotToCam);
              poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
              cameraBundles.add(new CameraBundle(camera, poseEstimator));
            });

    cameras = Collections.unmodifiableList(cameraBundles);
  }

  @Override
  public void periodic() {
    List<EstimatedRobotPose> estimates = new ArrayList<>();
    List<Matrix<N3, N1>> stdDevs = new ArrayList<>();
    for (CameraBundle bundle : cameras) {
      var unreadResults = bundle.camera.getAllUnreadResults();

      for (PhotonPipelineResult res : unreadResults) {
        res.targets = res.targets.stream().filter(t -> t.poseAmbiguity < 0.2).toList();
        res.multitagResult = res.multitagResult.filter(r -> r.estimatedPose.ambiguity < 0.2);

        if (!res.hasTargets()) {
          continue;
        }
        bundle
            .poseEstimator
            .update(res)
            .filter(est -> isEstimateUsable(est, res))
            .ifPresent(
                (est) -> {
                  estimates.add(est);
                  stdDevs.add(estimationStdDevs(est.estimatedPose.toPose2d(), res));
                });

        // if (!res.hasTargets()) {
        //   continue;
        // }
        // bundle
        //     .poseEstimator
        //     .update(res)
        //     .filter(est -> isEstimateUsable(est, res))
        //     .ifPresent(
        //         (est) -> {
        //           estimates.add(est);
        //           stdDevs.add(estimationStdDevs(est.estimatedPose.toPose2d(), res));
        //         });
      }
    }

    latestEstimates = Collections.unmodifiableList(estimates);
    curStdDevs = Collections.unmodifiableList(stdDevs);
  }

  public Matrix<N3, N1> estimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
    var estStdDevs = SINGLE_TAG_STD_DEVS;
    var targets = pipelineResult.getTargets();
    double avgDist = 0;
    double avgWeight = 0;
    for (var tgt : targets) {
      var tagPose = layout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      avgWeight += 1;
    }
    if (targets.isEmpty()) return estStdDevs;

    avgDist /= targets.size();
    avgWeight /= targets.size();

    // Decrease std devs if multiple targets are visibleX
    if (targets.size() > 1) estStdDevs = MULTIPLE_TAG_STD_DEVS;
    // Increase std devs based on (average) distance
    if (targets.size() == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return VecBuilder.fill(0.0, 0.0, 0.0);
    // return estStdDevs.times(avgWeight);
  }

  public List<EstimatedRobotPose> getLatestPoseEstimates() {
    return latestEstimates;
  }

  public List<Matrix<N3, N1>> getEstimationStdDevs() {
    return curStdDevs;
  }

  public boolean isConnected() {
    return cameras.stream().anyMatch(context -> context.camera.isConnected());
  }

  private boolean isEstimateUsable(EstimatedRobotPose est, PhotonPipelineResult result) {
    // add thresholds and stuff here later
    return true;
  }
}
