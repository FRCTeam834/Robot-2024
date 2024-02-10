package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.intake.Intake;

public class RunIndexerAndIntake extends Command {
  private boolean holdingNote = false;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Intake intake;
  private final Indexer indexer;

  /** Creates a new DriveWithJoysticks. */
  public RunIndexerAndIntake(Intake intake, Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.startIntake();
    indexer.startIndexer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!inputs.noteIsDetectedBack && !inputs.noteIsDetectedFront){
      indexer.startIndexer();
    }
    if (!inputs.noteIsDetectedFront){
      indexer.slowIndexer();
    }
    if (inputs.noteIsDetectedFront) {
      indexer.stopIndexer();
      intake.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return holdingNote;
  }    
}
