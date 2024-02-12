// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;

public class DriveWithSpeeds extends Command {
  private final Swerve driveTrain;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final DoubleSupplier omegaSupplier;

  /**
   * 
   * @param driveTrain
   * @param vxSupplier - Supplies [-1, 1]
   * @param vySupplier - Supplies [-1, 1]
   * @param omegaSupplier - Supplies [-1, 1]
   */
  public DriveWithSpeeds(
    Swerve driveTrain,
    DoubleSupplier vxSupplier,
    DoubleSupplier vySupplier,
    DoubleSupplier omegaSupplier
  ) {
    this.driveTrain = driveTrain;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.omegaSupplier = omegaSupplier;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(
      vxSupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(),
      vySupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(),
      omegaSupplier.getAsDouble() * Swerve.maxSteerSpeed.get()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
