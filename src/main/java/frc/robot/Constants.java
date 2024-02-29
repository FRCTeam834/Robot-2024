package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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
            new Transform3d(Units.inchesToMeters(8), Units.inchesToMeters(9), Units.inchesToMeters(16.5),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(10), Units.degreesToRadians(180)))
        ),
        new AprilTagIOPhotonVision(
            "CameraRight",
            new Transform3d(Units.inchesToMeters(14.25), Units.inchesToMeters(10), Units.inchesToMeters(15),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(10), 0.0))
        ),
        new AprilTagIOPhotonVision(
            "CameraLeft",
            new Transform3d(0.0, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, 0.0))
        )
    };

    public static NoteDetectionIOPhotonVision noteDetectionCamera = new NoteDetectionIOPhotonVision("Poly");

    public static PathConstraints AMP_LINEUP_CONSTRAINTS = new PathConstraints(
        0.5, 0.5, // set to low values for testing
        Units.degreesToRadians(360), Units.degreesToRadians(360));
}