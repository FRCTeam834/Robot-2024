// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.TunableNumber;

public class DriveWithNoteAlign extends Command {
  private final Swerve driveTrain;
  private final Vision vision;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier rightJoystickTrigger;

  private static final TunableNumber alignkP = new TunableNumber("Commands/DriveWithNoteAlignkP");
  private final PIDController alignController;
  
  private double rotationToNote;
  private double PIDOutput;

  static {
    alignkP.initDefault(0.6);
  }
  
  public DriveWithNoteAlign(Swerve driveTrain, Vision vision, DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier, BooleanSupplier rightJoystickTrigger) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.omegaSupplier = omegaSupplier;
    this.rightJoystickTrigger = rightJoystickTrigger;

    alignController = new PIDController(alignkP.get(), 0, 0);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationToNote = vision.getRotationToNoteTelemetry();
    if (rightJoystickTrigger.getAsBoolean() && rotationToNote != 0.0) {
      PIDOutput = alignController.calculate(new Rotation2d(rotationToNote).getRadians());

      driveTrain.drive(
       vxSupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
       vySupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
       MathUtil.clamp(PIDOutput, -Swerve.maxSteerSpeed.get(), Swerve.maxSteerSpeed.get()));
    } else {
      driveTrain.drive(
       vxSupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
       vySupplier.getAsDouble() * Swerve.maxTranslationSpeed.get(), 
       omegaSupplier.getAsDouble() * Swerve.maxSteerSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
