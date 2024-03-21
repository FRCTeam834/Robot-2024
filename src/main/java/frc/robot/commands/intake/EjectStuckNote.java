// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class EjectStuckNote extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  
  public EjectStuckNote(Intake intake, Indexer indexer, Shooter shooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;

    addRequirements(intake, indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter.setDesiredPivotAngle(0.15);
    //shooter.setDesiredRollerSpeeds(-1000);
    intake.setVoltage(-12);
    //indexer.setVoltage(-12);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    //indexer.stop();
    //shooter.setDesiredPivotAngle(0.85);
    //shooter.setDesiredRollerSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
