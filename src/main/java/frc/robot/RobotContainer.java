// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.VisionHandler;
import frc.robot.subsystems.vision.AprilTagIO;
import frc.robot.subsystems.vision.AprilTagIOPhotonVision;
import frc.robot.subsystems.vision.NoteDetectionIOPhotonVision;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  private final Vision vision = new Vision(new AprilTagIO[] {new AprilTagIOPhotonVision("CameraFront", new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))} , new NoteDetectionIOPhotonVision("Poly"));
  private final Command visionHandler = new VisionHandler(vision);

  private final LoggedDashboardChooser<Command> autonChooser = new LoggedDashboardChooser<>("Pick Auton");
  public RobotContainer() {
    vision.setDefaultCommand(visionHandler);
    autonChooser.addDefaultOption("Do Nothing", new InstantCommand());
    configureBindings();

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
