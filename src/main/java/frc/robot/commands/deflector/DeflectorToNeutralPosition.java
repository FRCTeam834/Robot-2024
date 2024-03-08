// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.deflector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deflector.Deflector;

public class DeflectorToNeutralPosition extends Command {
  /** Creates a new DeflectorToNeutralPosition. */
  public Deflector deflector;
  //!Timer code might not be good
  public Timer timer = new Timer();
  //!Limit switch code
  //private final DigitalInput limitSwitch = new DigitalInput(1);
  public DeflectorToNeutralPosition(Deflector deflector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.deflector = deflector;
    addRequirements(deflector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    deflector.setVoltage(-8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    deflector.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //!If needed, uncomment this code
    /*
     * if (limitSwitch.get()) {
     *  return true;
     * }
     * 
    */
    if (timer.get() > 0.3) {
      return true;
    }
    return false;
  }
}