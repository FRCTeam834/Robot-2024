// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.outtake.ShootWhenReadyNoYaw;
import frc.robot.commands.outtake.alignShooterToTag;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonVisionShot extends ParallelRaceGroup {

  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;
  private final LEDs leds;

  public AutonVisionShot(Shooter shooter, Indexer indexer, Vision vision, LEDs leds) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;
    this.leds = leds;
  
    addCommands(
      new ShootWhenReadyNoYaw(indexer, shooter, vision, leds),
      new alignShooterToTag(shooter, indexer, vision));
  }
}
