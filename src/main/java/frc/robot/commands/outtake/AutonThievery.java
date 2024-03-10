// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutonThievery extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Timer timer = new Timer();
  /** Creates a new AutonThievery. */
  public AutonThievery(Shooter shooter, Indexer indexer, Intake intake) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    addRequirements(shooter, indexer, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.setDesiredPivotAngle(0.92);
    shooter.setDesiredRollerSpeeds(2000.0);
    indexer.setSetpoint(Indexer.Setpoint.FAST);
    intake.setSetpoint(Intake.Setpoint.FAST);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(15.3);
  }
}
