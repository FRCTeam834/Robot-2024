// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithSpeeds;
import frc.robot.commands.test.DumbShooter;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModuleIOMAXSwerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSparkMAX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.PoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** Initialize subsystems */
  Vision vision = new Vision(Constants.aprilTagCameras, Constants.noteDetectionCamera);
  Swerve swerve = new Swerve(
    new SwerveModuleIOMAXSwerve(0),
    new SwerveModuleIOMAXSwerve(1),
    new SwerveModuleIOMAXSwerve(2),
    new SwerveModuleIOMAXSwerve(3),
    new GyroIOPigeon2()
  );
  PoseEstimator poseEstimator = new PoseEstimator(swerve, vision);
  Shooter shooter = new Shooter(new ShooterIOSparkMAX());
  Indexer indexer = new Indexer(new IndexerIOSparkMAX());
  Intake intake = new Intake(new IntakeIOSparkMAX());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerve.setDefaultCommand(new DriveWithSpeeds(
      swerve,
      OI::getRightJoystickX,
      OI::getRightJoystickY,
      OI::getLeftJoystickX
    ));
    
    /*
    shooter.setDefaultCommand(new DumbShooter(
      shooter,
      OI::getXboxRightJoystickY,
      OI::getXboxLeftJoystickY
    ));
    //*/
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
