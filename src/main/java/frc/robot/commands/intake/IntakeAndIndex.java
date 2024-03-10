// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.LEDs;
import frc.robot.utility.LEDs.Colors;

public class IntakeAndIndex extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final LEDs leds;
  private final Timer stopTimer = new Timer();
  private final Timer slowIndexerTimer = new Timer();
  
  public IntakeAndIndex(Intake intake, Indexer indexer, Shooter shooter, LEDs leds) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.leds = leds;

    addRequirements(intake, indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopTimer.reset();
    stopTimer.stop();
    slowIndexerTimer.reset();
    slowIndexerTimer.stop();
    shooter.setDesiredPivotAngle(0.92);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // NO note detected
    if (!indexer.noteDetectedIntakeSide() && !indexer.noteDetectedShooterSide()) {
      leds.setColorForTime(Colors.RED, 10.0);
      intake.setSetpoint(Intake.Setpoint.FAST);
      indexer.setSetpoint(Indexer.Setpoint.FAST);
    } else if (indexer.noteDetectedIntakeSide()) {
      leds.setColorForTime(Colors.STROBEBLUE, 1.0);
      intake.setSetpoint(Intake.Setpoint.SLOW);
      indexer.setSetpoint(Indexer.Setpoint.SLOW);
      shooter.setDesiredPivotAngle(0.6);
      stopTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.cancelColorForTime();
    intake.stop();
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.noteDetectedShooterSide();
  }
}
