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

    public static NoteDetectionIOPhotonVision noteDetectionCamera = new NoteDetectionIOPhotonVision("Poly");

    public static PathConstraints AMP_LINEUP_CONSTRAINTS = new PathConstraints(
        1, 1, // set to low values for testing
        Units.degreesToRadians(360), Units.degreesToRadians(360));
}