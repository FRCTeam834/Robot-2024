// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.utility.TunableNumber;

public class Swerve extends SubsystemBase {
  public static final TunableNumber maxModuleSpeed = new TunableNumber("drive/MaxModuleSpeed");
  public static final TunableNumber maxTranslationSpeed = new TunableNumber("drive/MaxTranslationSpeed");
  public static final TunableNumber maxSteerSpeed = new TunableNumber("drive/MaxSteerSpeed");
  /**
   * FL ----- FR
   * |        |
   * |        |
   * |        |
   * BL ----- BR
   */
  private final GyroIO gyro;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final SwerveModule[] modules = new SwerveModule[4]; // ordered: { FL, FR, BL, BR }
  private final double width = Units.inchesToMeters(26.2);
  private final double length = Units.inchesToMeters(26.25);
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(-width / 2, length / 2),
    new Translation2d(width / 2, length / 2),
    new Translation2d(-width / 2, -length / 2),
    new Translation2d(width / 2, -length / 2)
  );
  private boolean stopped = true;
  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

  static {
    maxModuleSpeed.initDefault(Units.feetToMeters(5));
    maxTranslationSpeed.initDefault(Units.feetToMeters(5));
    maxSteerSpeed.initDefault(Units.degreesToRadians(360));
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
    this.gyro = gyro;
    SmartDashboard.putData(this);
  }

  public Rotation2d getYaw () {
    return new Rotation2d(gyroInputs.yaw);
  }

  public double getYawRadians () {
    return getYaw().getRadians();
  }

  public void drive (
    double vx,
    double vy,
    double omega
  ) {
    vx = -vx;
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getYaw());
    setDesiredSpeeds(speeds);
  }

  public void setDesiredSpeeds (ChassisSpeeds speeds) {
    stopped = false;
    setpoint = speeds;
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroInputs);

    if (DriverStation.isDisabled() || stopped) {
      stop();
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

    for (var module : modules) {
      module.periodic();
    }
  }

  public void stop () {
    stopped = true;
    for (var module : modules) {
      module.stop();
    }
  }

  public SwerveModulePosition[] getModulePositions () {
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

  /** Expose kinematics for pose estimator etc. */
  public SwerveDriveKinematics getKinematics () {
    return kinematics;
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("Swerve");

    builder.addDoubleProperty("GyroYaw", this::getYawRadians, null);
  }
}
