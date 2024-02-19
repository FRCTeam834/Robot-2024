package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.AprilTagIOPhotonVision;
import frc.robot.subsystems.vision.NoteDetectionIOPhotonVision;

public class Constants {
    public static final RobotMode robotMode = RobotMode.DEVELOPMENT;

    public static enum RobotMode {
        COMPETITION,
        DEVELOPMENT,
        SIMULATION
    }

    public static AprilTagIOPhotonVision[] aprilTagCameras = {
        new AprilTagIOPhotonVision(
            "CameraFront",
            new Transform3d(0.0, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, 0.0))
        ),
        new AprilTagIOPhotonVision(
            "CameraRight",
            new Transform3d(0.0, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, 0.0))
        ),
        new AprilTagIOPhotonVision(
            "CameraLeft",
            new Transform3d(0.0, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, 0.0))
        )
    };

    public static NoteDetectionIOPhotonVision noteDetectionCamera = new NoteDetectionIOPhotonVision("Poly");

    public static PathConstraints AMP_LINEUP_CONSTRAINTS = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
}