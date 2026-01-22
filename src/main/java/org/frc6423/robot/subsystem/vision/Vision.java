package org.frc6423.robot.subsystem.vision;

import java.util.ArrayList;
import java.util.Arrays;

import org.frc6423.robot.RobotState;
import org.frc6423.robot.RobotState.VisionMeasurement;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final CameraIO[] cameras;

    public Vision(CameraIO... cameras) {
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        ArrayList<VisionMeasurement> poseEstimates = new ArrayList<>();
        for (CameraIO camera : cameras) {
            Arrays.stream(camera.getUnreadVisionMeasurements()).forEach(p -> poseEstimates.add(p));
        }

        RobotState.getInstace().addVisionMeasurement(poseEstimates.toArray(VisionMeasurement[]::new));
    }
}
