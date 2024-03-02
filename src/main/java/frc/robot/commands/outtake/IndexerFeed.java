// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

/**
 * Indexer feed note into shooter without condition
 * Used for auton or manual control
 */
public class IndexerFeed extends Command {
  /** Creates a new IndexerFeed. */
  private final Indexer indexer;

  public IndexerFeed(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setSetpoint(Indexer.Setpoint.FEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setSetpoint(Indexer.Setpoint.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
