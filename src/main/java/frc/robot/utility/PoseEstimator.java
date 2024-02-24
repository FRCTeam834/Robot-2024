package frc.robot.utility;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.vision.Vision;

public class PoseEstimator extends SubsystemBase {
    private final Swerve swerve;
    private final Vision vision;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d poseEstimateField = new Field2d();

    private double distanceToSpeaker = 0.0;

    public PoseEstimator (Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        poseEstimator = new SwerveDrivePoseEstimator(
            swerve.getKinematics(),
            swerve.getYaw(),
            swerve.getModulePositions(),
            new Pose2d()
        );
        SmartDashboard.putData("PoseEstimate", poseEstimateField);
    }

    public Pose2d getEstimatedPose () {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * 
     * Reset odometry to a known pose
     * @param pose
     * 
      */
    public void resetPose(Pose2d pose){
        swerve.resetYaw(pose.getRotation().getDegrees());
        poseEstimator.resetPosition(pose.getRotation(), swerve.getModulePositions(), pose); 
    }

    public Translation2d getSpeakerLocation () {
        /** BLUE SPEAKER LOCATION */
         Translation2d speakerLocation = new Translation2d(0.0, 0.0);

        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    /** RED SPEAKER LOCATION */
                    speakerLocation = new Translation2d(0.0, 0.0);
                }
              }
        return speakerLocation;
    }

    public double getRotationToSpeaker () {
        Translation2d speakerLocation = getSpeakerLocation();
        
        Pose2d currentPose = getEstimatedPose();
        double desiredRotationRad = Math.atan2(speakerLocation.getY() - currentPose.getY(), speakerLocation.getX() - currentPose.getX());
        double error = currentPose.getRotation().getRadians() - desiredRotationRad;

        return error;
    }

    public double getDistanceToSpeaker () {
        return distanceToSpeaker;
    }

    @Override
    public void periodic () {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerve.getYaw(), swerve.getModulePositions());

        AprilTagIOInputs[] visionInputs = vision.getInputs();
        for (int i = 0; i < visionInputs.length; i++) {
            if (visionInputs[i].poseEstimate == null) continue;
            poseEstimator.addVisionMeasurement(
                visionInputs[i].poseEstimate.toPose2d(),
                visionInputs[i].lastTimestamp
            );
        }

        Translation2d speakerLocation = getSpeakerLocation();
         Pose2d currentPose = getEstimatedPose();
        distanceToSpeaker = Math.sqrt(
            Math.pow(currentPose.getX() - speakerLocation.getX(), 2) +
            Math.pow(currentPose.getY() - speakerLocation.getY(), 2));

        if (Constants.robotMode == RobotMode.DEVELOPMENT) {
            poseEstimateField.setRobotPose(getEstimatedPose());
        }
    }
}
