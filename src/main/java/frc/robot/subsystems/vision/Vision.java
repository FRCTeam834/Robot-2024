// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.vision.NoteDetectionIO.NoteDetectionIOInputs;


public class Vision extends SubsystemBase {
  private final NoteDetectionIO polychromeCamera;
  private final AprilTagIO[] cameras;
  
  private final AprilTagIOInputs[] aprilInputs;
  private final NoteDetectionIOInputs noteInputs = new NoteDetectionIOInputs();

  public Vision(AprilTagIO[] cameras, NoteDetectionIO polychromeCamera) {
    this.polychromeCamera = polychromeCamera;
    this.cameras = cameras;
    aprilInputs = new AprilTagIOInputs[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      aprilInputs[i] = new AprilTagIOInputs();
    }
  }

  @Override
  public void periodic() {
    polychromeCamera.updateInputs(noteInputs);
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(aprilInputs[i]);
    }
  }

  public AprilTagIOInputs[] getInputs () {
    return aprilInputs;
  }
}