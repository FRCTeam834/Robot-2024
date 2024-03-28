// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotMode;
import frc.robot.commands.DriveLockToSpeaker;
import frc.robot.commands.DriveShootWhenReady;
import frc.robot.commands.DriveWithNoteAlign;
import frc.robot.commands.DriveWithSpeeds;
import frc.robot.commands.Autons.AutonIntakeAndWiggle;
import frc.robot.commands.Autons.AutonIntakeVisionShot;
import frc.robot.commands.Autons.AutonVisionShot;
import frc.robot.commands.climber.ClimbWithJoysticks;
import frc.robot.commands.deflector.DeflectorToNeutralPosition;
import frc.robot.commands.deflector.DeflectorToScoringPosition;
import frc.robot.commands.intake.EjectStuckNote;
import frc.robot.commands.intake.IntakeAndIndex;
import frc.robot.commands.intake.IntakeSequence;
import frc.robot.commands.intake.WiggleIndexer;
import frc.robot.commands.outtake.Auton4NoteShot;
import frc.robot.commands.outtake.AutonIndexerFeed;
import frc.robot.commands.outtake.AutonShootWhenReady;
import frc.robot.commands.outtake.AutonShot5;
import frc.robot.commands.outtake.AutonShot5Point5;
import frc.robot.commands.outtake.AutonThievery;
import frc.robot.commands.outtake.DPadControlShooter;
import frc.robot.commands.outtake.DumbShooter;
import frc.robot.commands.outtake.FeedShot;
import frc.robot.commands.outtake.IndexerFeed;
import frc.robot.commands.outtake.LockOnAprilTag;
import frc.robot.commands.outtake.ManualFarPost;
import frc.robot.commands.outtake.ShootWhenReadyNoYaw;
import frc.robot.commands.outtake.SubwooferShot;
import frc.robot.commands.outtake.alignShooterToTag;
import frc.robot.commands.outtake.Amp.AmpShot;
import frc.robot.commands.outtake.Amp.GetReadyAmpShot;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.deflector.Deflector;
import frc.robot.subsystems.deflector.DeflectorIOSparkMax;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModuleIOMAXSwerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSparkMAX;
import frc.robot.subsystems.indexer.Indexer.Setpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;
import frc.robot.subsystems.vision.AprilTagIOPhotonVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.LEDs;
import frc.robot.utility.PoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** Initialize subsystems */
  public static Shooter shooter = new Shooter(new ShooterIOSparkMAX());
  public static AprilTagIOPhotonVision[] aprilTagCameras = {
        new AprilTagIOPhotonVision(
            "abc",
            new Transform3d(Units.inchesToMeters(-7.25), Units.inchesToMeters(-7.96875), Units.inchesToMeters(16.125),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-10), Units.degreesToRadians(180))
            ), shooter
        )
        /*new AprilTagIOPhotonVision(
            "CameraRight",
            new Transform3d(Units.inchesToMeters(5), Units.inchesToMeters(9.5), Units.inchesToMeters(15),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(10), Units.degreesToRadians(0)))
        ),
        new AprilTagIOPhotonVision(
            "CameraLeft",
            new Transform3d(0.0, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, 0.0))
        )*/
    };
  public static Vision vision = new Vision(aprilTagCameras, Constants.noteDetectionCamera);
  public static Swerve swerve = new Swerve(
    new SwerveModuleIOMAXSwerve(0),
    new SwerveModuleIOMAXSwerve(1),
    new SwerveModuleIOMAXSwerve(2),
    new SwerveModuleIOMAXSwerve(3),
    new GyroIOPigeon2()
  );
  public static PoseEstimator poseEstimator = new PoseEstimator(swerve, vision);
  public static Indexer indexer = new Indexer(new IndexerIOSparkMAX());
  public static Intake intake = new Intake(new IntakeIOSparkMAX());
  public static Deflector deflector = new Deflector(new DeflectorIOSparkMax());
  public static Climber climber = new Climber(new ClimberIOSparkMax());
  public LEDs leds = new LEDs();

  

  private final SendableChooser<Command> autoChooser;
  private final Field2d pathPlannerField;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerve.setDefaultCommand(new DriveWithSpeeds(
      swerve,
      OI::getRightJoystickY,
      OI::getRightJoystickX,
      OI::getLeftJoystickX
    ));

    climber.setDefaultCommand(new ClimbWithJoysticks(
       climber,
       OI::getXboxRightJoystickY,
       OI::getXboxLeftJoystickY
     ));

    shooter.setDefaultCommand(new alignShooterToTag(shooter, indexer, vision));

    /**
     * Pathplanner stuff
    */

    NamedCommands.registerCommand("SubwooferShot", new SubwooferShot(shooter, indexer));
    NamedCommands.registerCommand("LongSubwooferShot", new Auton4NoteShot(shooter, indexer));
    NamedCommands.registerCommand("AutonIntake", new AutonIntakeAndWiggle(shooter, indexer, intake, leds));
    NamedCommands.registerCommand("IndexerFeed", new AutonIndexerFeed(indexer));
    NamedCommands.registerCommand("DeflectorOut", new DeflectorToScoringPosition(deflector));
    NamedCommands.registerCommand("AmpShot", new AmpShot(shooter, indexer));
    NamedCommands.registerCommand("AutonShootWhenReady", new AutonShootWhenReady(swerve, shooter, indexer, intake, vision, leds));
    NamedCommands.registerCommand("AutonIntakeVisionShot", new AutonIntakeVisionShot(shooter, indexer, intake, vision, leds));
    NamedCommands.registerCommand("WiggleIndexer", new IntakeAndIndex(intake, indexer, shooter, leds));
    NamedCommands.registerCommand("AutonShot5", new AutonShot5(shooter, indexer));
    NamedCommands.registerCommand("AutonShot5Point5", new AutonShot5Point5(shooter, indexer));
    NamedCommands.registerCommand("AutonThievery", new AutonThievery(shooter, indexer, intake));
    NamedCommands.registerCommand("AutonVisionShot", new AutonVisionShot(shooter, indexer, vision, leds));

    swerve.configureAutoBuilder(poseEstimator);
    autoChooser = new SendableChooser<>();
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("houdini", new PathPlannerAuto("houdini"));
    autoChooser.addOption("MID ABC", new PathPlannerAuto("MID ABC"));
    autoChooser.addOption("MID AB", new PathPlannerAuto("MID AB"));
    autoChooser.addOption("MID BC", new PathPlannerAuto("MID BC"));
    autoChooser.addOption("Vision MID ABC", new PathPlannerAuto("Vision MID ABC"));

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

  JoystickButton leftJoystick1 = new JoystickButton(OI.leftJoystick, 1);
  JoystickButton leftJoystick3 = new JoystickButton(OI.leftJoystick, 3);

  JoystickButton rightJoystick1 = new JoystickButton(OI.rightJoystick, 1);
  JoystickButton rightJoystick3 = new JoystickButton(OI.rightJoystick, 3);
  JoystickButton rightJoystick10 = new JoystickButton(OI.rightJoystick, 10);

  JoystickButton xboxA = new JoystickButton(OI.xbox, 1);
  JoystickButton xboxB = new JoystickButton(OI.xbox, 2);
  JoystickButton xboxX = new JoystickButton(OI.xbox, 3);
  JoystickButton xboxY = new JoystickButton(OI.xbox, 4);
  JoystickButton xboxRB = new JoystickButton(OI.xbox, 6);
  JoystickButton xboxLB = new JoystickButton(OI.xbox, 5);

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

    // For tuning ONLY
    // if (Constants.robotMode == RobotMode.DEVELOPMENT) {
    //  shooter.setDefaultCommand(new DumbShooter(
    //    shooter,
    //    OI::getXboxRightJoystickY,
    //    OI::getXboxLeftJoystickY
    //  ));
    // }

    rightJoystick10.onTrue(new InstantCommand(() -> { 
      poseEstimator.resetPose(new Pose2d(
        poseEstimator.getEstimatedPose().getTranslation(),
        new Rotation2d()
      ));
    }));

    rightJoystick1.whileTrue(new DriveShootWhenReady(
      swerve, shooter, indexer, vision,
      OI::getRightJoystickY,
      OI::getRightJoystickX,
      OI::getLeftJoystickX,
      leds
    ));

    //leftJoystick3.whileTrue(new IntakeSequence(intake, indexer, shooter, leds, leftJoystick3));

    xboxA.whileTrue(new SubwooferShot(shooter, indexer));
    xboxB.whileTrue(new EjectStuckNote(intake, indexer, shooter));
    xboxY.whileTrue(new InstantCommand(() -> {
      indexer.setVoltage(-3);
    }));
    xboxY.onFalse(new InstantCommand(() -> {
      indexer.stop();
    }));
    //xboxB.whileTrue(new ManualFarPost(shooter, indexer));
    //xboxY.whileTrue(new ParallelCommandGroup(
    //  new InstantCommand(() -> {
    //    shooter.setDesiredPivotAngle(1.1);
    //  }),
    //  new DeflectorToScoringPosition(deflector)
    //));
    xboxRB.whileTrue(new GetReadyAmpShot(shooter));
    //xboxLB.whileTrue(new DeflectorToNeutralPosition(deflector));

    xboxLB.onTrue(new FeedShot(shooter, indexer));
    xboxX.onTrue(new IntakeSequence(intake, indexer, shooter, leds, xboxX));
    //xboxY.whileTrue(new DeflectorToNeutralPosition(deflector));

    /** Amp lineup */
    
    /*leftJoystick1.whileTrue(AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("Amp Lineup"),
      Constants.AMP_LINEUP_CONSTRAINTS,
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    ));*/
    leftJoystick1.whileTrue(new SequentialCommandGroup(
      //new DeflectorToScoringPosition(deflector),
      new AmpShot(shooter, indexer)
    ));
    leftJoystick1.onFalse(new DeflectorToNeutralPosition(deflector));

    /*
    new JoystickButton(OI.leftJoystick, 7).onTrue(new DeflectorToNeutralPosition(deflector));
    new JoystickButton(OI.leftJoystick, 6).onTrue(new DeflectorToScoringPosition(deflector));
    */

    leftJoystick3.onTrue(new IntakeSequence(intake, indexer, shooter, leds, leftJoystick3));
    
    //new JoystickButton(OI.rightJoystick, 2).whileTrue(new IndexerFeed(indexer));
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
