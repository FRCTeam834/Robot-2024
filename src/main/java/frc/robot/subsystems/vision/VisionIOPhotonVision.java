// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.Constants;

/** Add your docs here. */
public class VisionIOPhotonVision implements VisionIO {
    private ArrayList<PhotonCamera> cams;
    private ArrayList<PhotonPoseEstimator> robotToCams;
    // private PhotonPoseEstimator[] robotToCams = new PhotonPoseEstimator[3];
    private static Optional<EstimatedRobotPose> estimatedPoseFront;
  
    private static AprilTagFieldLayout loadFieldLayout () {
      try {
        return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
        e.printStackTrace();
        return null;
      }
    }
  
    public VisionIOPhotonVision() {
      Collections.addAll(cams, new PhotonCamera("cameraFront"), new PhotonCamera("cameraRight"), new PhotonCamera("cameraLeft"));
      
      for (int i = 0; i < cams.size(); i++){
        robotToCams.add(new PhotonPoseEstimator(VisionIOPhotonVision.loadFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cams.get(i), Constants.VisionConstants.ROBOT_TO_CAMERAS[i]));
      }
      
    }


    // @Override
    // public void updateInputs(VisionIOInputs inputs){}

    public Pose2d getEstimatedGlobalPose2d(){
      return estimatedPoseFront.get().estimatedPose.toPose2d();
    }

    public Optional<EstimatedRobotPose> updateEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      poseFront.setReferencePose(prevEstimatedRobotPose);
      estimatedPoseFront = poseFront.update();
      return estimatedPoseFront;
    }
}




  

