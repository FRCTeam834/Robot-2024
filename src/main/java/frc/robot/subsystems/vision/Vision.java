// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private static AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  private PhotonCamera[] cams = { new PhotonCamera("cameraFront"), new PhotonCamera("cameraRight"),
      new PhotonCamera("cameraLeft") };

  private PhotonCamera camColored = new PhotonCamera("cameraColored");

  private PhotonPoseEstimator[] camPoseEstimators = {
      new PhotonPoseEstimator(Vision.loadFieldLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cams[0], Constants.VisionConstants.ROBOT_TO_FRONT_CAMERAS),
      new PhotonPoseEstimator(Vision.loadFieldLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cams[1], Constants.VisionConstants.ROBOT_TO_RIGHT_CAMERAS),
      new PhotonPoseEstimator(Vision.loadFieldLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cams[2], Constants.VisionConstants.ROBOT_TO_LEFT_CAMERAS) };

  private static Optional<EstimatedRobotPose> estimatedPoseFront;
  private static Optional<EstimatedRobotPose> estimatedPoseRight;
  private static Optional<EstimatedRobotPose> estimatedPoseLeft;

  // private final VisionIO io;

  /** Creates a new vision. */
  public Vision() {
    for (PhotonPoseEstimator cams : camPoseEstimators) {
      cams.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }


  }

  public PhotonPoseEstimator[] getPhotonPoseEstimators(){
    return camPoseEstimators;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator poseEstimator) {
    return poseEstimator.update();
  }

  public Double getYawFromNote(){
    var result = camColored.getLatestResult();
    if (result.hasTargets() == true){
      PhotonTrackedTarget target = result.getBestTarget();
      return target.getYaw();
    }
    return null;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    estimatedPoseFront = camPoseEstimators[0].update();
    estimatedPoseRight = camPoseEstimators[1].update();
    estimatedPoseLeft = camPoseEstimators[2].update();

    Logger.recordOutput("EstimatedPoses", new Pose2d[] {estimatedPoseFront.get().estimatedPose.toPose2d(), estimatedPoseRight.get().estimatedPose.toPose2d(), estimatedPoseLeft.get().estimatedPose.toPose2d()});

    // for (PhotonPoseEstimator poseEstimator : io.getPhotonPoseEstimators()){
    // io.getEstimatedGlobalPose(poseEstimator);
    // }
  }
}
