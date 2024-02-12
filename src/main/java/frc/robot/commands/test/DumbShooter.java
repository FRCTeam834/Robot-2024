// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class DumbShooter extends Command {
  private final Shooter shooter;
  private final DoubleSupplier pivotAngleSupplier;
  private final DoubleSupplier speedDeltaSupplier;

  private final static double minPivotAngle = Units.degreesToRadians(-20);
  private final static double maxPivotAngle = Units.degreesToRadians(-70);

  private final static double speedGainPerSecond = 100 * 0.02; // 100 rpm/s
  private double rollerSpeed = 0.0;

  public DumbShooter(
    Shooter shooter,
    DoubleSupplier pivotAngleSupplier,
    DoubleSupplier speedDeltaSupplier
  ) {
    this.shooter = shooter;
    this.pivotAngleSupplier = pivotAngleSupplier;
    this.speedDeltaSupplier = speedDeltaSupplier;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turn supplied angle from [-1, 1] to [0, 1]
    double normalized = (pivotAngleSupplier.getAsDouble() + 1) / 2;
    // Turn [0, 1] to [min, max]
    double mappedAngle = minPivotAngle + 
      (maxPivotAngle - minPivotAngle) * normalized;

    shooter.setDesiredPivotAngle(mappedAngle);

    rollerSpeed += speedDeltaSupplier.getAsDouble() * speedGainPerSecond;
    shooter.setDesiredRollerSpeeds(rollerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
