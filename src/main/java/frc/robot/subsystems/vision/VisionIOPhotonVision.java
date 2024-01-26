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
import frc.robot.Constants;

/** Add your docs here. */
public class VisionIOPhotonVision implements VisionIO {
  private PhotonCamera[] cams = { new PhotonCamera("cameraFront"), new PhotonCamera("cameraRight"),
      new PhotonCamera("cameraLeft") };
  private PhotonPoseEstimator[] camPoseEstimators = {
      new PhotonPoseEstimator(VisionIOPhotonVision.loadFieldLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cams[0], Constants.VisionConstants.ROBOT_TO_FRONT_CAMERAS),
      new PhotonPoseEstimator(VisionIOPhotonVision.loadFieldLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cams[1], Constants.VisionConstants.ROBOT_TO_RIGHT_CAMERAS),
      new PhotonPoseEstimator(VisionIOPhotonVision.loadFieldLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cams[2], Constants.VisionConstants.ROBOT_TO_LEFT_CAMERAS) };

  private static Optional<EstimatedRobotPose> estimatedPoseFront;

  private static AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  public VisionIOPhotonVision() {
    for(PhotonPoseEstimator cams : camPoseEstimators) {
      cams.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  public PhotonPoseEstimator[] getPhotonPoseEstimators(){
    return camPoseEstimators;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator poseEstimator) {
    return poseEstimator.update();
  }
}