// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

import java.util.function.DoubleSupplier;

public class ClimbWithJoysticks extends Command {
  private final Climber climber;
  private final DoubleSupplier rightYSupplier;
  private final DoubleSupplier leftYSupplier;

  /**
   * @param rightYSupplier - Supplies [-1, 1]
   * @param leftYSupplier - Supplies [-1, 1]
   */

  /** Creates a new ClimbWithJoysticks. */
  public ClimbWithJoysticks(Climber climber, DoubleSupplier rightYSupplier, DoubleSupplier leftYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.rightYSupplier = rightYSupplier;
    this.leftYSupplier = leftYSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setArmSpeeds(rightYSupplier.getAsDouble(), leftYSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setArmSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}