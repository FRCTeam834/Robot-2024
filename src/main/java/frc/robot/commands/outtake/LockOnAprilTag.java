// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class LockOnAprilTag extends Command {
  private final Shooter shooter;
  private final Vision vision;  
  private final Indexer indexer;
  private final LinearFilter shooterAngleAverage = LinearFilter.movingAverage(3);
  private final LinearFilter distanceAverage = LinearFilter.movingAverage(3);
  private final PIDController alignController;

  private static InterpolatingDoubleTreeMap shooterOffsetTable = new InterpolatingDoubleTreeMap();

  static {
    // (key: shooter angle rad) (value: offset rad)

    // sample values
    shooterOffsetTable.put(0.65, Units.degreesToRadians(-0.48));
  }


  public LockOnAprilTag(Shooter shooter, Indexer indexer, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;
    this.indexer = indexer;

    alignController = new PIDController(10, 0, 0);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    shooterAngleAverage.reset();
    distanceAverage.reset();
    alignController.reset();
    shooter.setDesiredPivotAngle(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!vision.getInputs()[0].hasTarget) return;
    double error = vision.getInputs()[0].pitchToTag;
    error = shooterAngleAverage.calculate(error);

    double voltage = -alignController.calculate(error);
    voltage += (Math.signum(voltage) * 0.03);

    System.out.println(error + "V: " + voltage);
    shooter.setPivotVoltage(voltage);
    shooter.setDesiredRollerSpeeds(5000);
    //shooter.setDesiredRollerSpeeds(shooter.getShooterSpeedForDistance(distance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!indexer.noteDetectedIntakeSide() && !indexer.noteDetectedShooterSide()) {
      // shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
      // shooter.setDesiredPivotAngle(0.95);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
