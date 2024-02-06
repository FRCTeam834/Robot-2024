// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.LoggedTunableNumber;

public class Swerve extends SubsystemBase {
  private static final LoggedTunableNumber maxModuleSpeed = new LoggedTunableNumber("Drivetrain/MaxModuleSpeed");
  private static final LoggedTunableNumber maxTranslationSpeed = new LoggedTunableNumber("Drivetrain/MaxTranslationSpeed");
  private static final LoggedTunableNumber maxSteerSpeed = new LoggedTunableNumber("Drivetrain/MaxSteerSpeed");
  /**
   * FL ----- FR
   * |        |
   * |        |
   * |        |
   * BL ----- BR
   */
  private final SwerveModule[] modules = new SwerveModule[4]; // ordered: { FL, FR, BL, BR }
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(0, 0),
    new Translation2d(0, 0),
    new Translation2d(0, 0),
    new Translation2d(0, 0)
  );
  
  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

  static {
    maxTranslationSpeed.initDefault(Units.feetToMeters(1));
  }

  /** Creates a new DriveTrain. */
  public Swerve(
    SwerveModuleIO flSwerveModuleIO,
    SwerveModuleIO frSwerveModuleIO,
    SwerveModuleIO blSwerveModuleIO,
    SwerveModuleIO brSwerveModuleIO,
    GyroIO gyro
  ) {
    modules[0] = new SwerveModule(flSwerveModuleIO, 0);
    modules[1] = new SwerveModule(frSwerveModuleIO, 1);
    modules[2] = new SwerveModule(blSwerveModuleIO, 2);
    modules[3] = new SwerveModule(brSwerveModuleIO, 3);
  }

  public void setDesiredSpeeds (ChassisSpeeds speeds) {
    setpoint = speeds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      return;
    }

    setpoint = ChassisSpeeds.discretize(setpoint, 0.02);
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(setpoint);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      lastChassisSpeeds,
      maxModuleSpeed.get(),
      maxTranslationSpeed.get(),
      maxSteerSpeed.get()
    );

    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);

    lastChassisSpeeds = setpoint;
  }
}
