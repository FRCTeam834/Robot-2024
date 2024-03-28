// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utility.LEDs;
import frc.robot.utility.LEDs.Colors;

public class WiggleIndexer extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private final LEDs leds;
  private boolean firstWiggle;
  private Timer wiggleTimer = new Timer();

  //
  private final double wiggleTime = 0.5;


  /** Creates a new WiggleIndexer. */
  public WiggleIndexer(Intake intake, Indexer indexer, LEDs leds) {
    this.intake = intake;
    this.indexer = indexer;
    this.leds = leds;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstWiggle = true;
    wiggleTimer.reset();
    wiggleTimer.stop();
    intake.stop();
    //intake.setSetpoint(Intake.Setpoint.SLOW);
    // leds.setColorForTime(Colors.RED, 2.0);
    //leds.setColorForTime(Colors.CONFETTI, 10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer.noteDetectedShooterSide()) {
      // Since note is up against shooter
      // if first wiggle push note out faster
      if (firstWiggle) {
        indexer.setVoltage(-12);
        //leds.setColorForTime(Colors.STROBEBLUE, 1.5);
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
    //leds.cancelColorForTime();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when note 
    return (wiggleTimer.hasElapsed(wiggleTime) && indexer.noteDetectedShooterSide()) || !indexer.hasNote();
  }
}