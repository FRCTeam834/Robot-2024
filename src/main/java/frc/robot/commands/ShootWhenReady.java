// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.PoseEstimator;

public class ShootWhenReady extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final PoseEstimator poseEstimator;
  private boolean hasBegunFeed = false;
  
  public ShootWhenReady(Shooter shooter, Indexer indexer, PoseEstimator poseEstimator) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.poseEstimator = poseEstimator;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasBegunFeed = false;
    indexer.setSetpoint(Indexer.Setpoint.STOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setDesiredPivotAngle(shooter.getPivotAngleForDistance(poseEstimator.getDistanceToSpeaker()));
    shooter.setDesiredRollerSpeeds(shooter.getShooterSpeedForDistance(poseEstimator.getDistanceToSpeaker()));
    // indexer has alrdy begun feeding into shooter, too late to stop it
    if (hasBegunFeed) return;
    // tolerances
    if (Math.abs(poseEstimator.getRotationToSpeaker()) > Units.degreesToRadians(5)) return;
    if (!shooter.atSetpoint(poseEstimator.getDistanceToSpeaker())) return;

    hasBegunFeed = true;

    indexer.setSetpoint(Indexer.Setpoint.FEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
    shooter.setDesiredPivotAngle(shooter.getIntakePivotAngle());
    indexer.setSetpoint(Indexer.Setpoint.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !indexer.hasNote();
  }
}
