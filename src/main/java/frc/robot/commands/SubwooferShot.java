// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class SubwooferShot extends Command {
  /** Creates a new SubwooferShot. */
  private final Shooter shooter;
  private final Indexer indexer;
  private boolean fed = false;
  Timer a = new Timer();
  public SubwooferShot(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(shooter, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fed = false;
    a.reset();
    a.start();
    shooter.setDesiredPivotAngle(0.3);
    shooter.setDesiredRollerSpeeds(5000);
    indexer.setSetpoint(Indexer.Setpoint.STOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (a.get() < 1) return;
    if (!shooter.atSetpoint(0.0)) return;
    fed = true;;
    indexer.setSetpoint(Indexer.Setpoint.FEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
    indexer.setSetpoint(Indexer.Setpoint.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !indexer.hasNote();
  }
}
