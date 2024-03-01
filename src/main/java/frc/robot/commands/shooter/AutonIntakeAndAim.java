// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeAndIndex;
import frc.robot.commands.intake.WiggleIndexer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonIntakeAndAim extends SequentialCommandGroup {
  /** Creates a new AutonReadyShooter. */
  public AutonIntakeAndAim(Intake intake, Indexer indexer, Shooter shooter, PoseEstimator poseEstimator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeAndIndex(intake, indexer, shooter),
      new ParallelCommandGroup(
        new WiggleIndexer(intake, indexer),
        new SequentialCommandGroup(
          new WaitCommand(0.1),
          new LockOnSpeaker(shooter, poseEstimator)
        )
      )
    );
  }
}
