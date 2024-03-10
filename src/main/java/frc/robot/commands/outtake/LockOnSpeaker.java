// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.PoseEstimator;

/**
 * Makes shooter continously go to setpoint for speaker shot
 * Useful to reduce wait time before shooting
 */

public class LockOnSpeaker extends Command {
  /** Creates a new LockOnSpeaker. */
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

  private static final double lookAheadTime = 0.1;

  public LockOnSpeaker(Shooter shooter, Indexer indexer, Vision vision) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //System.out.println("distance: " + poseEstimator.calculateDistanceToSpeakerInTime(lookAheadTime));
    // Look ahead a little so shooter isn't lagging behind
    // look ahead time should be about the response time of the system
    double distance = vision.getInputs()[0].distance;
    shooter.setDesiredPivotAngle(shooter.getPivotAngleForDistance(distance));
    shooter.setDesiredRollerSpeeds(shooter.getShooterSpeedForDistance(distance));
  }

  @Override
  public void end(boolean interrupted) {
    if (!indexer.noteDetectedIntakeSide() && !indexer.noteDetectedShooterSide()) {
      shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
      shooter.setDesiredPivotAngle(0.95);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
