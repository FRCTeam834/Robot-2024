package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionOLD extends SubsystemBase {

  private final SwerveDrivePoseEstimator swervePoseEstimator;

  private final PhotonCamera camFront;
  private final PhotonCamera camRight;
  private final PhotonCamera camLeft;
  // private final PhotonCamera camColored; 

  private final PhotonPoseEstimator posFront;
  private final PhotonPoseEstimator posRight;
  private final PhotonPoseEstimator posLeft;
  //private final PhotonPoseEstimator posColored;

  private Optional<EstimatedRobotPose> estimatedPoseFront;
  private Optional<EstimatedRobotPose> estimatedPoseRight;
  private Optional<EstimatedRobotPose> estimatedPoseLeft;


  
  private static AprilTagFieldLayout loadFieldLayout () {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  public VisionOLD() {
    camFront = new PhotonCamera("cameraFront");
    camRight = new PhotonCamera("cameraRight");
    camLeft = new PhotonCamera("cameraLeft");
    //camColored = new PhotonCamera("cameraColored");

    posFront = new PhotonPoseEstimator(VisionOLD.loadFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camFront, new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
    posRight = new PhotonPoseEstimator(VisionOLD.loadFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camRight, new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
    posLeft = new PhotonPoseEstimator(VisionOLD.loadFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camLeft, new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));

  }

  // public void setReferencePose (Pose2d pose) {
  //   photonPoseEstimator.setReferencePose(pose);
  // }

  public Optional<EstimatedRobotPose>[] getEstimatedPose (){
    Optional<EstimatedRobotPose>[] poseEstimators = {estimatedPoseFront, estimatedPoseRight, estimatedPoseLeft}
    return poseEstimators;
  }


  @Override
  public void periodic() {
    estimatedPoseFront = posFront.update();
    estimatedPoseRight = posRight.update();
    estimatedPoseLeft = posLeft.update();

    if(estimatedPoseFront.isPresent()) {        
      swervePoseEstimator.addVisionMeasurement(estimatedPoseFront.get().estimatedPose.toPose2d(), estimatedPoseFront.get().timestampSeconds);
    }

    if (estimatedPoseLeft.isPresent()) {
      swervePoseEstimator.addVisionMeasurement(estimatedPoseLeft.get().estimatedPose.toPose2d(), estimatedPoseLeft.get().timestampSeconds);
    }

    if (estimatedPoseRight.isPresent()) {
      swervePoseEstimator.addVisionMeasurement(estimatedPoseRight.get().estimatedPose.toPose2d(), estimatedPoseRight.get().timestampSeconds);
    }

    // swervePoseEstimator.update();
  }
}
