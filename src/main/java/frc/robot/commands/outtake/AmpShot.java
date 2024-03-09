// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Hardcoded shot into amp
 */
public class AmpShot extends Command {
  /** Creates a new SubwooferShot. */
  private final Shooter shooter;
  private final Indexer indexer;

  public AmpShot(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(shooter, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setDesiredPivotAngle(0.8);
    shooter.setDesiredRollerSpeeds(2450);
    indexer.setSetpoint(Indexer.Setpoint.STOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooter.atDesiredSetpoint(Units.degreesToRadians(0.5), 50)) return;
    indexer.setSetpoint(Indexer.Setpoint.FEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!indexer.hasNote()) {
      shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
      shooter.setDesiredPivotAngle(0.15);
      indexer.setSetpoint(Indexer.Setpoint.STOP);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
