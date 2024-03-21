// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.outtake.LockOnAprilTag;
import frc.robot.commands.outtake.LockOnSpeaker;
import frc.robot.commands.outtake.ShootWhenReady;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.LEDs;
import frc.robot.utility.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveShootWhenReady extends ParallelCommandGroup {
  /** Creates a new DrivePrepareShoot. */
  public DriveShootWhenReady(Swerve driveTrain, Shooter shooter, Indexer indexer, Vision vision, DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new LockOnSpeaker(shooter, indexer, vision),
      // new LockOnAprilTag(shooter, indexer, vision),
      new ShootWhenReady(indexer, shooter, vision, leds),
      new DriveLockToSpeaker(driveTrain, vision, vxSupplier, vySupplier, omegaSupplier, 0.1)
    );
  }
}
