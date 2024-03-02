// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class DumbShooter extends Command {
  private final Shooter shooter;
  private final DoubleSupplier pivotAngleSupplier;
  private final DoubleSupplier speedDeltaSupplier;

  private final static double minPivotAngle = 0.16;
  private final static double maxPivotAngle = 1.1;

  private final static double speedGainPerSecond = 1000 * 0.02; // 1000 rpm/s
  private final static double angleGainPerSecond = Units.degreesToRadians(10) * 0.02; // 10 deg/s
  private double pivotAngle = minPivotAngle;
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
  public void initialize() {
    //pivotAngle = minPivotAngle;
    rollerSpeed = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotAngle += pivotAngleSupplier.getAsDouble() * angleGainPerSecond;
    pivotAngle = MathUtil.clamp(pivotAngle, minPivotAngle, maxPivotAngle);

    shooter.setDesiredPivotAngle(pivotAngle);

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
