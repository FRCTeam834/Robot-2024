// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonIntakeVisionShot extends SequentialCommandGroup {

  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Vision vision;
  private final LEDs leds;

  /** Creates a new AutonIntakeVisionShot. */
  public AutonIntakeVisionShot(Shooter shooter, Indexer indexer, Intake intake, Vision vision, LEDs leds) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.vision = vision;
    this.leds = leds;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new AutonIntakeAndWiggle(shooter, indexer, intake, leds),
        new WaitCommand(0.5)
      ),
      new AutonVisionShot(shooter, indexer, vision, leds)
    );
  }
}
