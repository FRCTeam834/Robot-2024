// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.shooter.Shooter;

public class DPadControlShooter extends Command {

  private final Shooter shooter;

  private double desiredAngle;

  public DPadControlShooter(Shooter shooter) {
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = shooter.getCurrentPivotAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = shooter.getCurrentPivotAngle();
    // LOW 0.17 HIGH 1.1
    if (OI.isDPadUpPressed() && currentAngle < 1) {
      desiredAngle += 0.05;
    }
    if (OI.isDPadDownPressed() && currentAngle > 0.2) {
      desiredAngle -= 0.05;
    }
    
    shooter.setDesiredPivotAngle(desiredAngle);
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
