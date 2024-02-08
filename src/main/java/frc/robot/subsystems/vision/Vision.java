// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;


public class Vision extends SubsystemBase {
  private final NoteDetectionIO polychromeCamera;
  private final AprilTagIO[] cameras;
  
  // private final AprilTagIOInputsAutoLogged[] aprilInputs;
  private final AprilTagIOInputs[] aprilInputs;
  // private final NoteDetectionIOInputsAutoLogged noteInputs;

  private final List<Vision.PoseAndTimestamp> results = new ArrayList<>();

  /** Creates a new Vision. */
  public Vision(AprilTagIO[] cameras, NoteDetectionIO polychromeCamera) {
    this.polychromeCamera = polychromeCamera;
    this.cameras = cameras;
    aprilInputs = new AprilTagIOInputs[cameras.length];
    // noteInputs = new NoteDetectionIOInputsAutoLogged();

    for (int i = 0; i < cameras.length; i++) {
      aprilInputs[i] = new AprilTagIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    polychromeCamera.updateInputs();
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(aprilInputs[i]);
      results.add(new PoseAndTimestamp(aprilInputs[i].poseEstimate3d.toPose2d(), aprilInputs[i].timestamp));
    }
    Logger.recordOutput("VisionOdometry", results.get(0).getPose());
  }

    /**
     * Returns the last recorded pose
     */
    public List<Vision.PoseAndTimestamp> getVisionOdometry() {
      return results;
  }

    /**
     * Inner class to record a pose and its timestamp
     */
    public static class PoseAndTimestamp {
        Pose2d pose;
        double timestamp;

        public PoseAndTimestamp(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }
}
