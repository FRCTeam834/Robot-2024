// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;

public class TempShooter extends Command {
  public final CommandXboxController controller = new CommandXboxController(0);
  public Shooter shooter;
  public double desiredSpeeds = 0.0;
  
  public TempShooter(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredSpeeds += (controller.getLeftY() * 10);
    if(desiredSpeeds >= 5000){
      desiredSpeeds = 5000;
    } 
    if (desiredSpeeds <= 0) {
      desiredSpeeds = 0;
    }
    shooter.setDesiredRollerSpeeds(desiredSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
