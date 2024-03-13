package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO {
    class AprilTagIOInputs {
        // public Pose3d poseEstimate = new Pose3d();
        // public double lastTimestamp = 0;
        // public boolean isMultiTag = false;
        // public double averageDistance = 0;
        public double yawToSpeaker = 0.0;
        public double pitchToTag = 0.0;
        public double distance = 0;
        public boolean hasTarget = false;
    }

    default void updateInputs(AprilTagIOInputs inputs) {}

    default boolean isConnected () { return false; }

    default String getName() {
        return "";
    }
}
