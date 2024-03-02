// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class WiggleIndexer extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private boolean firstWiggle;
  private Timer wiggleTimer = new Timer();

  //
  private final double wiggleTime = 0.4;


  /** Creates a new WiggleIndexer. */
  public WiggleIndexer(Intake intake, Indexer indexer) {
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstWiggle = true;
    wiggleTimer.reset();
    wiggleTimer.stop();
    intake.setSetpoint(Intake.Setpoint.SLOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer.noteDetectedShooterSide()) {
      // Since note is up against shooter
      // if first wiggle push note out faster
      if (firstWiggle) {
        indexer.setVoltage(-4);
      } else {
        indexer.setVoltage(-1);
      }
    } else {
      firstWiggle = false;
      wiggleTimer.start();
      indexer.setVoltage(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when note 
    return wiggleTimer.hasElapsed(wiggleTime) && indexer.noteDetectedShooterSide();
  }
}