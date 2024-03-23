// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class alignShooterToTag extends Command {

  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;
  private final LinearFilter shooterAngleAverage = LinearFilter.movingAverage(10);
  private final PIDController alignController;

  public alignShooterToTag(Shooter shooter, Indexer indexer, Vision vision) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;

    alignController = new PIDController(10, 0, 0);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    shooter.setDesiredPivotAngle(0.7);
    shooterAngleAverage.reset();
  }

  @Override
  public void execute() {
    if (indexer.hasNote() && vision.getInputs()[0].hasTarget) {
      double error = vision.getInputs()[0].pitchToTag - Units.degreesToRadians(5);
      error = shooterAngleAverage.calculate(error);


      double voltage = -alignController.calculate(error);
      voltage += (Math.signum(voltage) * 0.02);
      shooter.setPivotVoltage(voltage);
    } else if (!indexer.hasNote()) {
      shooter.setDesiredPivotAngle(0.92);
    } else {
      shooter.setDesiredPivotAngle(0.6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
