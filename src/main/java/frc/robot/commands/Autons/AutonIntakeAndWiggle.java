// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeAndIndex;
import frc.robot.commands.intake.WiggleIndexer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonIntakeAndWiggle extends SequentialCommandGroup {

  private final Shooter shooter;
  private final Indexer indexer;
  private final LEDs leds;
  private final Intake intake;
  
  /** Creates a new AutonIntakeAndWiggle. */
  public AutonIntakeAndWiggle(Shooter shooter, Indexer indexer, Intake intake, LEDs leds) {
    
    this.shooter = shooter;
    this.indexer = indexer;
    this.leds = leds;
    this.intake = intake;

    addCommands(new IntakeAndIndex(intake, indexer, shooter, leds),
                new WiggleIndexer(intake, indexer, leds));

  }
}
