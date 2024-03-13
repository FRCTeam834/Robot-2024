// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class LockOnAprilTag extends Command {
  private final Shooter shooter;
  private final Vision vision;  
  private final Indexer indexer;
  private final LinearFilter shooterAngleAverage = LinearFilter.movingAverage(3);
  private final LinearFilter distanceAverage = LinearFilter.movingAverage(3);

  public LockOnAprilTag(Shooter shooter, Indexer indexer, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;
    this.indexer = indexer;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterAngleAverage.reset();
    distanceAverage.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!vision.getInputs()[0].hasTarget) return;
    double distance = distanceAverage.calculate(vision.getInputs()[0].distance);
    double currentAngle = shooter.getCurrentPivotAngle();
    double error = vision.getInputs()[0].pitchToTag;

    error = shooterAngleAverage.calculate(error);
    shooter.setDesiredPivotAngle(currentAngle + error);
    shooter.setDesiredRollerSpeeds(shooter.getShooterSpeedForDistance(distance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!indexer.noteDetectedIntakeSide() && !indexer.noteDetectedShooterSide()) {
      shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
      shooter.setDesiredPivotAngle(0.95);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
