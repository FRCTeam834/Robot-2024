package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


public class AprilTagIOPhotonVision implements AprilTagIO {
    private final PhotonCamera camera;
    private final String name;
    private final PhotonPoseEstimator poseEstimator;

    public static double fieldLength = Units.inchesToMeters(651.223);
    public static double fieldWidth = Units.inchesToMeters(323.277);

    public static int[] allowedIDs = { 3, 4, 7, 8 };

    /**
     * Implements PhotonVision camera
     *
     * @param name Name of the camera.
     * @param robotToCamera Location of the camera on the robot (from center, positive x towards the arm, positive y to the left, and positive angle is counterclockwise.
     */
    public AprilTagIOPhotonVision(String name, Transform3d robotToCamera) {
        this.name = name;

        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(loadFieldLayout(), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
        
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public boolean isConnected () {
        return camera.isConnected();
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        inputs.poseEstimate = null;
        var result = camera.getLatestResult();

        // Not multi-tag
        if (!result.hasTargets() || result.targets.size() < 2) return;

        Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update(result);

        if (!poseEstimatorResult.isPresent()) return;

        Pose3d estimatedPose = poseEstimatorResult.get().estimatedPose;
        // Estimated pose is outside of the field
        if (isPoseInsane(estimatedPose)) return;

        double distanceSum = 0.0;
        for (int i = 0; i < result.targets.size(); i++) {
            var target = result.targets.get(i);
            if (target.getPoseAmbiguity() > 0.2) return;
            distanceSum += target
                .getBestCameraToTarget()
                .getTranslation()
                .getDistance(new Translation3d(0, 0, 0));
        }

        // Average distance to tags
        double averageDistance = distanceSum / result.targets.size();

        inputs.poseEstimate = estimatedPose;
        inputs.lastTimestamp = result.getTimestampSeconds();
        inputs.averageDistance = averageDistance;
    }

    /**
     * Checks if pose is insane value (outside field)
     * @param pose
     * @return
     */
    public boolean isPoseInsane (Pose3d pose) {
        double xymargin = Units.feetToMeters(1);
        double zmargin = Units.feetToMeters(2);
        return pose.getX() < -xymargin
            || pose.getX() > fieldLength + xymargin
            || pose.getY() < -xymargin
            || pose.getY() > fieldWidth + xymargin
            || pose.getZ() < -zmargin
            || pose.getZ() > zmargin;
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