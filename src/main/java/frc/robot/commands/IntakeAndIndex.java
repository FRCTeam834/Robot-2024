// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class IntakeAndIndex extends Command {
  /** Creates a new IntakeAndIndex. */
  private final Intake intake;
  private final Indexer indexer;
  
  public IntakeAndIndex(Intake intake, Indexer indexer) {
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!indexer.noteDetectedBack() && !indexer.noteDetectedFront()) {
      intake.setSetpoint(Intake.Setpoint.SLURP);
      indexer.setSetpoint(Indexer.Setpoint.FAST);
    } else if (!indexer.noteDetectedFront()) {
      intake.setSetpoint(Intake.Setpoint.BURP);
      indexer.setSetpoint(Indexer.Setpoint.SLOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.hasNote();
  }
}
