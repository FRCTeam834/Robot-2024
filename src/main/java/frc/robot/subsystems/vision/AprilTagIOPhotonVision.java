package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;


public class AprilTagIOPhotonVision implements AprilTagIO {
    private final PhotonCamera camera;
    private final String name;
    private final PhotonPoseEstimator odometry;

    /**
     * Implements PhotonVision camera
     *
     * @param name Name of the camera.
     * @param robotToCamera Location of the camera on the robot (from center, positive x towards the arm, positive y to the left, and positive angle is counterclockwise.
     */
    public AprilTagIOPhotonVision(String name, Transform3d robotToCamera) {
        this.name = name;

        camera = new PhotonCamera(name);
        odometry = new PhotonPoseEstimator(loadFieldLayout(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        odometry.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public boolean isConnected () {
        return camera.isConnected();
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            inputs.poseEstimate = null;
            return;
        }
        if (result.getBestTarget().getPoseAmbiguity() > 0.2) {
            inputs.poseEstimate = null;
            return;
        }
        Optional<EstimatedRobotPose> currentPose = odometry.update(result);

        // If target found update pose
        if (currentPose.isPresent()) {
            inputs.poseEstimate = currentPose.get().estimatedPose;
            inputs.lastTimestamp = result.getTimestampSeconds();
        } else {
            // No pose estimate if there is no target
            inputs.poseEstimate = null;
        }
    }

    @Override
    public String getName() {
        return this.name;
    }

    private static AprilTagFieldLayout loadFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }
}