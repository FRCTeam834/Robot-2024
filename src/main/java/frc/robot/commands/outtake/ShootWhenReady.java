// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.PoseEstimator;

/**
 * Indexer feeds note into shooter once tolerances are met
 * For teleop use
 */
public class ShootWhenReady extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final PoseEstimator poseEstimator;
  private int confidenceTicks;

  public ShootWhenReady(Indexer indexer, Shooter shooter, PoseEstimator poseEstimator) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.poseEstimator = poseEstimator;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    confidenceTicks = 3;
    indexer.setSetpoint(Indexer.Setpoint.STOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Shooter is at setpoint angle and speeds
    if (!shooter.atSetpoint(poseEstimator.getDistanceToSpeaker())) {
      confidenceTicks = Math.min(confidenceTicks, confidenceTicks + 1);
      return;
    };
    // Robot is pointed at speaker
    if (Math.abs(poseEstimator.getRotationToSpeaker()) > Units.degreesToRadians(2)) {
      confidenceTicks = Math.min(confidenceTicks, confidenceTicks + 1);
      return;
    }
    // confidence ticks make sure we are within tolerance for some time and not by chance
    if (--confidenceTicks > 0) return;

    indexer.setSetpoint(Indexer.Setpoint.FEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setSetpoint(Indexer.Setpoint.STOP);
    if (!indexer.noteDetectedIntakeSide()) {
      shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !indexer.noteDetectedShooterSide() && !indexer.noteDetectedIntakeSide();
  }
}
