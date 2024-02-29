// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deflector.Deflector;

public class DeflectorToScoringPosition extends Command {
  /** Creates a new DeflectorToScoringPosition. */
  public Deflector deflector;
  private final DigitalInput limitSwitch = new DigitalInput(0);
  private final Timer timer = new Timer();
  private boolean hasHitSwitch = false;
  
  public DeflectorToScoringPosition(Deflector deflector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.deflector = deflector;
    addRequirements(deflector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasHitSwitch = false;
    timer.reset();
    timer.stop();
    deflector.setVoltage(6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!limitSwitch.get()) {
      hasHitSwitch = true;
      timer.start();
    }
    if (hasHitSwitch) {
      deflector.setVoltage(2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    deflector.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(0.2)) {
      return true;
    }

    return false;
  }
}