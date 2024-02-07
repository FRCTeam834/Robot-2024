// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TempShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;

public class RobotContainer {
  private final LoggedDashboardChooser<Command> autonChooser = new LoggedDashboardChooser<>("Pick Auton");
  private final Shooter shooter;
  
  public RobotContainer() {
    shooter = new Shooter(new ShooterIOSparkMAX());
    autonChooser.addDefaultOption("Do Nothing", new InstantCommand());
    configureBindings();
    shooter.setDefaultCommand(new TempShooter(shooter));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
