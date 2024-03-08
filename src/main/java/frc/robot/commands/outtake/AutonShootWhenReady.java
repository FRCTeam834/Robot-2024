// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.DriveLockToSpeaker;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShootWhenReady extends ParallelRaceGroup {
  /** Creates a new AutonShootWhenReady. */
  public AutonShootWhenReady(Swerve swerve, Shooter shooter, Indexer indexer, Intake intake, PoseEstimator poseEstimator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootWhenReady(indexer, shooter, poseEstimator),
      new DriveLockToSpeaker(swerve, poseEstimator, () -> { return 0; }, () -> { return 0; }, () -> { return 0; }, 0)
    );
  }
}
