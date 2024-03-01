// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.PoseEstimator;

public class LockOnSpeaker extends Command {
  /** Creates a new LockOnSpeaker. */
  private final Shooter shooter;
  private final PoseEstimator poseEstimator;

  private static final double lookAheadTime = 0.2;

  public LockOnSpeaker(Shooter shooter, PoseEstimator poseEstimator) {
    this.shooter = shooter;
    this.poseEstimator = poseEstimator;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = poseEstimator.calculateDistanceToSpeakerInTime(lookAheadTime);
    shooter.setDesiredPivotAngle(shooter.getPivotAngleForDistance(distance));
    shooter.setDesiredRollerSpeeds(shooter.getShooterSpeedForDistance(distance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    shooter.setDesiredPivotAngle(0.95);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
