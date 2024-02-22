// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.deflector.Deflector;
import frc.robot.subsystems.deflector.DeflectorIOSparkMax;

public class tempDeflector extends Command {
  /** Creates a new tempDeflector. */
  public final CommandXboxController deflectorController = new CommandXboxController(0);
  public Deflector deflector;
  public Trigger aButton = deflectorController.a();
  public Trigger bButton = deflectorController.b();

  //public double desiredAngle;
  public tempDeflector(Deflector deflector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.deflector = deflector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (aButton.getAsBoolean()) {
      deflector.goToScoringPosition();
    } else if (bButton.getAsBoolean()) {
      deflector.goToNeutralPosition();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    deflector.goToNeutralPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
