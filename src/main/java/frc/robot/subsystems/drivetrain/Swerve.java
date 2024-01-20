// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  /**
   * FL ----- FR
   * |        |
   * |        |
   * |        |
   * BL ----- BR
   */
  private final SwerveModule[] modules = new SwerveModule[4]; // ordered: { FL, FR, BL, BR }
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(null);
  private ChassisSpeeds setpoint = new ChassisSpeeds();
  /** Creates a new DriveTrain. */
  public Swerve(
    SwerveModuleIO flSwerveModuleIO,
    SwerveModuleIO frSwerveModuleIO,
    SwerveModuleIO blSwerveModuleIO,
    SwerveModuleIO brSwerveModuleIO
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

    
  }
}
