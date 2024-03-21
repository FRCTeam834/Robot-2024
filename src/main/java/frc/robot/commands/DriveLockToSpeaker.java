// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.PoseEstimator;

public class DriveLockToSpeaker extends Command {
  private final Swerve driveTrain;
  private final Vision vision;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final DoubleSupplier omegaSupplier;

  private final PIDController alignController;
  private double PIDOutput;
  private double speedMultiplier;
  private final LinearFilter angleAverage = LinearFilter.movingAverage(3);
  
  public DriveLockToSpeaker(Swerve driveTrain, Vision vision, DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier, double speedMultipler) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.omegaSupplier = omegaSupplier;
    this.speedMultiplier = speedMultipler;

    alignController = new PIDController(5, 0, 0);
    alignController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleAverage.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = vision.getInputs()[0].hasTarget;
    if (hasTarget) {
      double error = -vision.getInputs()[0].yawToSpeaker;
      error = angleAverage.calculate(error);
    
      PIDOutput = alignController.calculate(error);
      PIDOutput = MathUtil.clamp(PIDOutput, -1, 1);
      if (Math.abs(error) < Units.degreesToRadians(0.1)) PIDOutput = 0.0;
      driveTrain.drive(
      vxSupplier.getAsDouble() * Swerve.maxTranslationSpeed.get() * speedMultiplier, 
      vySupplier.getAsDouble() * Swerve.maxTranslationSpeed.get() * speedMultiplier, 
      PIDOutput);
    } else {
      PIDOutput = omegaSupplier.getAsDouble() * Swerve.maxSteerSpeed.get();
      driveTrain.drive(
      vxSupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
      vySupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
      PIDOutput);
    }

      
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
