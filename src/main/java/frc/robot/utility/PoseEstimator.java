package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
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

    private static final double visionXYstddev = 0.1;
    private static final double visionTHETAstddev = 1;

    public PoseEstimator (Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        poseEstimator = new SwerveDrivePoseEstimator(
            swerve.getKinematics(),
            swerve.getYaw(),
            swerve.getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, 0.001),
            VecBuilder.fill(visionXYstddev, visionXYstddev, visionTHETAstddev)
        );
        SmartDashboard.putData("PoseEstimate", poseEstimateField);
        SmartDashboard.putData(this);
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
         Translation2d speakerLocation = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    /** RED SPEAKER LOCATION */
                    speakerLocation = new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));
                }
              }
        return speakerLocation;
    }

    public double getRotationToSpeaker () {
        Translation2d speakerLocation = getSpeakerLocation();
        
        Pose2d currentPose = getEstimatedPose();
        double currentAngle = currentPose.getRotation().getRadians() + Units.degreesToRadians(180);
        double desiredRotationRad = Math.atan2(speakerLocation.getY() - currentPose.getY(), speakerLocation.getX() - currentPose.getX());
        double error = MathUtil.angleModulus(currentAngle - desiredRotationRad);

        return error;
    }

    
    public double calculateDistanceToSpeakerInTime (double time) {
        Translation2d speakerLocation = getSpeakerLocation();
        Pose2d currentPose = getEstimatedPose();
        Transform2d adjustForFuture = new Transform2d();
        if (time > 0) {
            // assume no rotation
            adjustForFuture = new Transform2d(
                swerve.getCurrentChassisSpeeds().vxMetersPerSecond * time,
                swerve.getCurrentChassisSpeeds().vyMetersPerSecond * time,
                new Rotation2d()
            );
        }
        currentPose.transformBy(adjustForFuture);

        return Math.sqrt(
            Math.pow(currentPose.getX() - speakerLocation.getX(), 2) +
            Math.pow(currentPose.getY() - speakerLocation.getY(), 2));
    }
    

    
    public double getDistanceToSpeaker () {
        return calculateDistanceToSpeakerInTime(0.0);
    }
    

    @Override
    public void periodic () {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerve.getYaw(), swerve.getModulePositions());

        AprilTagIOInputs[] visionInputs = vision.getInputs();
        for (int i = 0; i < visionInputs.length; i++) {
            if (visionInputs[i].poseEstimate == null) continue;
            poseEstimator.addVisionMeasurement(
                visionInputs[i].poseEstimate.toPose2d(),
                visionInputs[i].lastTimestamp,
                VecBuilder.fill(
                    visionXYstddev * Math.pow(visionInputs[i].averageDistance, 2),
                    visionXYstddev * Math.pow(visionInputs[i].averageDistance, 2),
                    visionTHETAstddev // dont care we never trust this
                )
            );
        }

        if (Constants.robotMode == RobotMode.DEVELOPMENT) {
            poseEstimateField.setRobotPose(getEstimatedPose());
        }
    }

    @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("Pose Estimator");

    builder.addDoubleProperty("ErrorToSpeaker", this::getRotationToSpeaker, null);
    builder.addDoubleProperty("DistanceToSpeaker", this::getDistanceToSpeaker, null);
  }
}
