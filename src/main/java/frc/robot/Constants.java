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
            "LongRangeCamera",
            new Transform3d(Units.inchesToMeters(-7.25), Units.inchesToMeters(-7.96875), Units.inchesToMeters(16.125),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-10), Units.degreesToRadians(180)))
        ),
        new AprilTagIOPhotonVision(
            "CameraRight",
            new Transform3d(Units.inchesToMeters(5), Units.inchesToMeters(9.5), Units.inchesToMeters(15),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(10), Units.degreesToRadians(0)))
        ),
        new AprilTagIOPhotonVision(
            "CameraLeft",
            new Transform3d(0.0, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, 0.0))
        )
    };

    public static NoteDetectionIOPhotonVision noteDetectionCamera = new NoteDetectionIOPhotonVision("Poly");

    public static PathConstraints AMP_LINEUP_CONSTRAINTS = new PathConstraints(
        1, 1, // set to low values for testing
        Units.degreesToRadians(360), Units.degreesToRadians(360));
}