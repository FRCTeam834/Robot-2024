// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithNoteAlign;
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
  //Shooter shooter = new Shooter(new ShooterIOSparkMAX());
  //Indexer indexer = new Indexer(new IndexerIOSparkMAX());
  //Intake intake = new Intake(new IntakeIOSparkMAX());

  private final SendableChooser<Command> autoChooser;
  private final Field2d pathPlannerField;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerve.setDefaultCommand(new DriveWithSpeeds(
      swerve,
      OI::getRightJoystickY,
      OI::getRightJoystickX,
      OI::getLeftJoystickX
    ));

    /* 
    swerve.setDefaultCommand(new AlignToNote(swerve, 
    vision, 
    OI::getRightJoystickX, 
    OI::getRightJoystickY, 
    OI::getLeftJoystickX, 
    OI::isRightJoystickTriggerPressed
    ));
    */

    /*
    shooter.setDefaultCommand(new DumbShooter(
      shooter,
      OI::getXboxRightJoystickY,
      OI::getXboxLeftJoystickY
    ));
    //*/

    /**
     * Pathplanner stuff
     */
    swerve.configureAutoBuilder(poseEstimator);
    // autoChooser = new SendableChooser<>();
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    //autoChooser.addOption("S-Curve", new PathPlannerAuto("S-Curve"));
    
    SmartDashboard.putData(autoChooser);
    pathPlannerField = new Field2d();
    SmartDashboard.putData("PathTarget", pathPlannerField); 

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        pathPlannerField.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        pathPlannerField.getObject("target pose").setPose(pose);
    });
    
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
    /** Amp lineup */
    new JoystickButton(OI.rightJoystick, 1).whileTrue(AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("Amp Lineup"),
      Constants.AMP_LINEUP_CONSTRAINTS,
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    ));

    new JoystickButton(OI.rightJoystick, 3).onTrue(new InstantCommand(() -> {
      swerve.resetYaw(0);
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }
}
