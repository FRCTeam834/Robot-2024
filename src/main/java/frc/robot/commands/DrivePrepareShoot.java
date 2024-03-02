// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.outtake.LockOnSpeaker;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivePrepareShoot extends ParallelCommandGroup {
  /** Creates a new DrivePrepareShoot. */
  public DrivePrepareShoot(Swerve driveTrain, Shooter shooter, Indexer indexer, PoseEstimator poseEstimator, DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LockOnSpeaker(shooter, indexer, poseEstimator),
      new DriveLockToSpeaker(driveTrain, poseEstimator, vxSupplier, vySupplier, omegaSupplier, 1)
    );
  }
}
