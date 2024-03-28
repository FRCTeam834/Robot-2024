// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
  private final Timer timer = new Timer();

  public alignShooterToTag(Shooter shooter, Indexer indexer, Vision vision) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;

    alignController = new PIDController(10, 0, 0); // last kp 6

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    //shooter.setDesiredPivotAngle(0.6);
    timer.reset();
    timer.stop();
    //shooterAngleAverage.reset();
  }

  @Override
  public void execute() {
    if (vision.getInputs()[0].hasTarget) {
      timer.reset();
      timer.stop();
    } else if (!vision.getInputs()[0].hasTarget) {
      timer.start();
      //if (indexer.hasNote()) {
        // idle spin
        //shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
      //}
    }

    if (indexer.hasNote() && !timer.hasElapsed(0.25)) {
      double error = vision.getInputs()[0].pitchToTag;
      //error = shooterAngleAverage.calculate(error);


      double voltage = -alignController.calculate(error);
      voltage += (Math.signum(voltage) * 0.05);
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
