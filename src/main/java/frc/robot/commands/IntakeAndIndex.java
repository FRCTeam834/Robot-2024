// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;


public class IntakeAndIndex extends Command {
  /** Creates a new IntakeAndIndex. */
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private Timer timer;
  private Timer intakeTimer;
  private Timer stopTimer;
  
  public IntakeAndIndex(Intake intake, Indexer indexer, Shooter shooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setDesiredPivotAngle(0.85);
    timer = new Timer();
    timer.reset();
    timer.stop();
    intakeTimer = new Timer();
    intakeTimer.reset();
    intakeTimer.stop();
    stopTimer = new Timer();
    stopTimer.reset();
    stopTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println(intakeTimer.get());

    if (intakeTimer.hasElapsed(0.1)){
      System.out.println("intakeTimer");
        intake.setSetpoint(Intake.Setpoint.STOP);
        indexer.setSetpoint(Indexer.Setpoint.SLOW);
        timer.start();
        intakeTimer.stop();
        intakeTimer.reset();
      }

    if (timer.hasElapsed(0.05)) {
      shooter.setDesiredPivotAngle(0.6);
      if (indexer.noteDetectedBack()) {
        indexer.setVoltage(-1);
      } else if (!indexer.noteDetectedBack()) {
        indexer.setVoltage(1);
      }
    } else if (!indexer.noteDetectedBack() && !indexer.noteDetectedFront()) {
      intake.setSetpoint(Intake.Setpoint.FAST);
      indexer.setSetpoint(Indexer.Setpoint.FAST);
    } else if (indexer.noteDetectedBack()) {
      intake.setSetpoint(Intake.Setpoint.FAST);
      indexer.setSetpoint(Indexer.Setpoint.SLOW);
      intakeTimer.start();
    }

    if (indexer.noteDetectedBack()) {
      stopTimer.start();
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
    return false;//(timer.hasElapsed(3));
  }
}
