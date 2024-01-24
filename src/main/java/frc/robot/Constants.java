package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
    public static final RobotMode robotMode = RobotMode.SIMULATION;

    public static enum RobotMode {
        COMPETITION,
        DEVELOPMENT,
        SIMULATION
    }

    public static final class VisionConstants {
        public static final Transform3d[] ROBOT_TO_CAMERAS = {new Transform3d(0.5, 0.0, 0.5, new Rotation3d(0, 0, 0)), new Transform3d(0,0, 0, new Rotation3d(0, 0, 0)), new Transform3d(0,0, 0, new Rotation3d(0, 0, 0))};
        
    }
}
