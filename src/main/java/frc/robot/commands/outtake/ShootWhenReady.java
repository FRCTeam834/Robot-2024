// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.LEDs;
import frc.robot.utility.PoseEstimator;
import frc.robot.utility.LEDs.Colors;

/**
 * Indexer feeds note into shooter once tolerances are met
 * For teleop use
 */
public class ShootWhenReady extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;
  private final LEDs leds;
  private int confidenceTicks;
  private final LinearFilter angleAverage = LinearFilter.movingAverage(3);
  private final LinearFilter distanceAverage = LinearFilter.movingAverage(3);
  private final LinearFilter shooterAngleAverage = LinearFilter.movingAverage(10);

  private static final InterpolatingDoubleTreeMap rollerSpeedsTable = new InterpolatingDoubleTreeMap();

  static {
    // (shooter angle rad, shooter rollers rpm)
    rollerSpeedsTable.put(1.0, 4000.0);
    rollerSpeedsTable.put(0.6, 5000.0);
    rollerSpeedsTable.put(0.2, 5500.0);
  }

  public ShootWhenReady(Indexer indexer, Shooter shooter, Vision vision, LEDs leds) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.vision = vision;
    this.leds = leds;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    confidenceTicks = 3;
    indexer.setSetpoint(Indexer.Setpoint.STOP);
    angleAverage.reset();
    distanceAverage.reset();
    shooterAngleAverage.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setDesiredRollerSpeeds(5500);
    if (vision.getInputs()[0].hasTarget == false) {
      leds.setColorForTime(Colors.STROBERED, 0.06);
      return;
    }  else {
      leds.setColorForTime(Colors.STROBEWHITE, 0.06);
    }
    double angle = angleAverage.calculate(vision.getInputs()[0].yawToSpeaker);
    //double distance = distanceAverage.calculate(vision.getInputs()[0].distance);
    double pitch = vision.getInputs()[0].pitchToTag;

    if (!vision.getInputs()[0].hasTarget) return;
    if (Math.abs(pitch) > Units.degreesToRadians(1.25)) {
      //System.out.println("pitch: " + Math.abs(pitch));
      return;
    }
    if (!shooter.atDesiredRollerSetpoint(50)) return;
    
    // Robot is pointed at speaker
    if (Math.abs(angle) > Units.degreesToRadians(3)) {
      //confidenceTicks = Math.min(confidenceTicks, confidenceTicks + 1);
      //System.out.println("yaw: " + Math.abs(angle));
      return;
    }
    // confidence ticks make sure we are within tolerance for some time and not by chance
    //if (--confidenceTicks > 0) return;

    indexer.setSetpoint(Indexer.Setpoint.FEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setSetpoint(Indexer.Setpoint.STOP);
    if (!indexer.noteDetectedIntakeSide()) {
      shooter.setDesiredRollerSpeeds(shooter.getIdleShooterSpeed());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !indexer.noteDetectedShooterSide() && !indexer.noteDetectedIntakeSide();
  }
}
