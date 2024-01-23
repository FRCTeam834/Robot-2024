// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionIOPhotonVision implements VisionIO {
    private PhotonCamera camFront;
    private PhotonPoseEstimator poseFront;
    private Optional<EstimatedRobotPose> estimatedPoseFront;
  
    private static AprilTagFieldLayout loadFieldLayout () {
      try {
        return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      } catch (IOException e) {
        e.printStackTrace();
        return null;
      }
    }
  
    public VisionIOPhotonVision() {
      camFront = new PhotonCamera("cameraFront");
      poseFront = new PhotonPoseEstimator(VisionIOPhotonVision.loadFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camFront, new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));

    }

    @Override
    public void updateInputs(VisionIOInputs inputs){
      
    }

}




  

