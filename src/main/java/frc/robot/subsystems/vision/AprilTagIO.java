package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO {
    class AprilTagIOInputs {
        public Pose3d poseEstimate = new Pose3d();
        public double lastTimestamp = 0;
    }

    default void updateInputs(AprilTagIOInputs inputs) {}

    default String getName() {
        return "";
    }
}
