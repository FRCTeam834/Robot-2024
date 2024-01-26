// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final VisionIO io;

  /** Creates a new vision. */
  public Vision(VisionIO io) {
    this.io = io; 
  }


  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    for (PhotonPoseEstimator poseEstimator : io.getPhotonPoseEstimators()){
      io.getEstimatedGlobalPose(poseEstimator);
    }
  }
}
