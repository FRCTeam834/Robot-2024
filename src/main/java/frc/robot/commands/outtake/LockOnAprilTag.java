// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  private final PIDController alignController;

  private static InterpolatingDoubleTreeMap shooterOffsetTable = new InterpolatingDoubleTreeMap();

  static {
    // (key: distance) (value: offset rad)

    // sample values
    shooterOffsetTable.put(1.0, 0.5);
    shooterOffsetTable.put(4.0, 0.1);
    shooterOffsetTable.put(7.0, 0.05);
  }

  public LockOnAprilTag(Shooter shooter, Indexer indexer, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;
    this.indexer = indexer;

    alignController = new PIDController(2, 0, 0);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    shooterAngleAverage.reset();
    distanceAverage.reset();
    alignController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Low 0.17, High 1.1
    double currentAngle = shooter.getCurrentPivotAngle();

    if (!vision.getInputs()[0].hasTarget) {
      if (vision.getInputs()[1].hasTarget) {
        //top cam found tag so move shooter up 
        if (currentAngle > 1) {
          shooter.stop();
          return;
        }
        shooter.setPivotVoltage(-1);
      } else if (vision.getInputs()[2].hasTarget) {
        //bottom cam found tag so so move shooter down
        if (currentAngle < 0.18) {
          shooter.stop();
          return;
        }
        shooter.setPivotVoltage(1);
      } else {
        shooter.setDesiredPivotAngle(0.95);
      }
      return;
    }

    double distance = distanceAverage.calculate(vision.getInputs()[0].distance);
    double error = vision.getInputs()[0].pitchToTag + shooterOffsetTable.get(distance);
    error = shooterAngleAverage.calculate(error);

    shooter.setPivotVoltage(-alignController.calculate(error));
    shooter.setDesiredRollerSpeeds(4000); //for now
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
