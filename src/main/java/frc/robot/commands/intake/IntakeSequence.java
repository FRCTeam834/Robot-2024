// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends SequentialCommandGroup {
  /** Creates a new IntakeSequence. */
  public IntakeSequence(Intake intake, Indexer indexer, Shooter shooter, LEDs leds, BooleanSupplier runSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterToIntakeAngle(shooter),
      new IntakeAndIndex(intake, indexer, shooter, leds).onlyWhile(runSupplier::getAsBoolean),
      new WiggleIndexer(intake, indexer, leds).onlyIf(indexer::noteDetectedIntakeSide)
    );
  }
}
