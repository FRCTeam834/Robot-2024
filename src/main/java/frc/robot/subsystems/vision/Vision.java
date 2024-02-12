// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
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

    SmartDashboard.putData(this);
  }

  public AprilTagIOInputs[] getInputs () {
    return aprilInputs;
  }

  public Double getRotationToNode () {
    return polychromeCamera.getRotationToNote();
  }

  public double getRotationToNoteTelemetry () {
    Double rotation = getRotationToNode();
    if (rotation == null) return 0.0;
    return rotation.doubleValue();
  }

  @Override
  public void periodic() {
    polychromeCamera.updateInputs(noteInputs);
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(aprilInputs[i]);
    }
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("Vision");
    builder.addDoubleProperty("AngleToNote", this::getRotationToNoteTelemetry, null);
  }
}