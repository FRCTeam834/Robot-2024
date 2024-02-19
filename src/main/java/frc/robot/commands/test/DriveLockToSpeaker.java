// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.PoseEstimator;

public class DriveLockToSpeaker extends Command {
  private final Swerve driveTrain;
  private final PoseEstimator poseEstimator;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;

  private final PIDController alignController;
  
  private double rotationToNote;
  private double PIDOutput;
  
  public DriveLockToSpeaker(Swerve driveTrain, PoseEstimator poseEstimator, DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;

    alignController = new PIDController(0.5, 0, 0);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d speakerLocation = new Pose2d(0, 5, new Rotation2d(0)); // random value change to actual
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      speakerLocation = new Pose2d(51, 5, new Rotation2d(Math.PI));
    }

    Pose2d currentPose = poseEstimator.getEstimatedPose();
    double desiredRotationRad = Math.atan2(speakerLocation.getY() - currentPose.getY(), speakerLocation.getX() - currentPose.getX());
    double error = currentPose.getRotation().getRadians() - desiredRotationRad;
    
      PIDOutput = alignController.calculate(error);

      driveTrain.drive(
       vxSupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
       vySupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
       MathUtil.clamp(PIDOutput, -Swerve.maxSteerSpeed.get(), Swerve.maxSteerSpeed.get()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
