// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.utility.TunableNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();
  private boolean pivotStopped = true;
  private boolean shooterStopped = true;
  
  private static final TunableNumber pivotkP = new TunableNumber("shooter/pivotkP");
  private static final TunableNumber pivotkD = new TunableNumber("shooter/pivotkD");

  private static final TunableNumber pivotkS = new TunableNumber("shooter/pivotkS");
  private static final TunableNumber pivotkG = new TunableNumber("shooter/pivotkG");
  private static final TunableNumber pivotkV = new TunableNumber("shooter/pivotkV");

  /** Both rollers should have the same control parameters */
  private static final TunableNumber rollerkP = new TunableNumber("shooter/rollerkP");
  private static final TunableNumber rollerkD = new TunableNumber("shooter/rollerkD");

  private static final TunableNumber rollerkS = new TunableNumber("shooter/rollerkS");
  private static final TunableNumber rollerkV = new TunableNumber("shooter/rollerkV");

  /** NOTE: tuning values are NOT set here, set them in defaults */
  private ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0, 0);
  private final ProfiledPIDController pivotPID = new ProfiledPIDController(0.0, 0.0, 0.0, 
    new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360)));


  private double desiredAngle = 0.0; // radians
  private double desiredRollerSpeeds = 0.0; // rpm

  /** Defaults (final values) are initialized here */
  static {
    /** */
    pivotkP.initDefault(8);
    pivotkD.initDefault(0.005);
    pivotkS.initDefault(0.05);
    pivotkG.initDefault(0.09);
    pivotkV.initDefault(1.6);
    /** */
    rollerkP.initDefault(1);
    rollerkD.initDefault(0);
    rollerkS.initDefault(0.1);
    rollerkV.initDefault(0);
  }

  private static InterpolatingDoubleTreeMap shotAngleTable = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shotAngleToleranceTable = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shotSpeedTable = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shotSpeedToleranceTable = new InterpolatingDoubleTreeMap();
  private static double intakePivotAngle = Units.degreesToRadians(0);
  private static double idleSpeed = 0;

  /** Initialize values for shot table */
  static {
    /** key: <horizontal distance m>, value: <pivot angle rad> */
    shotAngleTable.put(0.0, 0.0);
    shotAngleTable.put(1.0, 1.0);
    /** key: <horizontal distance m>, value: <rpm> */
    shotSpeedTable.put(0.0, 0.0);
    shotSpeedTable.put(1.0, 1.0);

    /**
     * TOLERANCES
     */

    /** key: <horizontal distance m>, value: <pivot angle tolerance rad> */
    shotAngleToleranceTable.put(-1.0, Units.degreesToRadians(2)); // -1 is values for amp I guess
    shotAngleToleranceTable.put(0.0, Units.degreesToRadians(2));
    shotAngleToleranceTable.put(5.0, Units.degreesToRadians(0.2));
     /** key: <horizontal distance m>, value: <tolerance rpm> */
    shotSpeedToleranceTable.put(-1.0, 20.0); // -1 is values for amp I guess
    shotSpeedToleranceTable.put(0.5, 20.0);
    shotSpeedToleranceTable.put(5.0, 5.0);
  }

  public Shooter(ShooterIO io) {
    this.io = io;

    // * Q:Is this still used for the pivot? A: Creo que sí
    pivotPID.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (pivotkP.hasChanged(hashCode()) || pivotkD.hasChanged(hashCode())) {
      pivotPID.setPID(pivotkP.get(), 0.0, pivotkD.get());
    }
    if (pivotkS.hasChanged(hashCode()) || pivotkG.hasChanged(hashCode()) || pivotkV.hasChanged(hashCode())) {
      pivotFeedforward = new ArmFeedforward(pivotkS.get(), pivotkG.get(), pivotkV.get());
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");
      System.out.println("Runs");

    }
    if (rollerkP.hasChanged(hashCode()) || rollerkD.hasChanged(hashCode())) {
      io.setRollerPID(rollerkP.get(), 0.0, rollerkD.get());
    }
    if (rollerkS.hasChanged(hashCode()) || rollerkV.hasChanged(hashCode())) {
      io.setRollerFeedforward(rollerkS.get(), rollerkV.get());
    }
    
    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    if (!pivotStopped) {
      io.setPivotVoltage(
        pivotFeedforward.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity) +
        pivotPID.calculate(inputs.pivotAngle));
    }
    if (!shooterStopped) {
      io.setRollerSpeeds(desiredRollerSpeeds);
    }
  }

  public void stop () {
    pivotStopped = true;
    shooterStopped = true;
    io.setPivotVoltage(0.0);
    io.setTopRollerVoltage(0.0);
    io.setBottomRollerVoltage(0.0);
  }

  /**
   * 
   * @param angle radians
   */
  public void setDesiredPivotAngle (double angle) {
    pivotStopped = false;
    desiredAngle = angle;
    pivotPID.setGoal(new TrapezoidProfile.State(angle, 0.0));
  }

  public void setPivotVoltage (double voltage) {
    io.setPivotVoltage(voltage);
  }

  /**
   * 
   * @param dist flat distance to speaker meters
   * @return radians
   */
  public double getPivotAngleForDistance (double dist) {
    return shotAngleTable.get(dist);
  }

  public double getShooterSpeedForDistance (double dist) {
    return shotSpeedTable.get(dist);
  }

  public double getIdleShooterSpeed () {
    return Shooter.idleSpeed;
  }

  public double getIntakePivotAngle () {
    return intakePivotAngle;
  }

  /**
   * 
   * @param speeds rpm
   */
  public void setDesiredRollerSpeeds (double speeds) {
    shooterStopped = false;
    desiredRollerSpeeds = speeds;
  }

  public double getCurrentPivotAngle () {
    return inputs.pivotAngle;
  }

  public double getCurrentTopRollerSpeed () {
    return inputs.topRollerVelocity;
  }

  public double getCurrentBottomRollerSpeed () {
    return inputs.bottomRollerVelocity;
  }

  public boolean atSetpoint (double dist) {
    double toleranceRPM = shotSpeedToleranceTable.get(dist);
    double toleranceAngle = shotAngleToleranceTable.get(dist);
    return 
      Math.abs(desiredRollerSpeeds - getCurrentTopRollerSpeed()) <= toleranceRPM &&
      Math.abs(desiredRollerSpeeds - getCurrentBottomRollerSpeed()) <= toleranceRPM &&
      Math.abs(desiredAngle - getCurrentPivotAngle()) <= toleranceAngle;
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("PivotAngle", this::getCurrentPivotAngle, null);
    builder.addDoubleProperty("TopRollerSpeed", this::getCurrentTopRollerSpeed, null);
    builder.addDoubleProperty("BottomRollerSpeed", this::getCurrentBottomRollerSpeed, null);
    builder.addDoubleProperty("AppliedVoltage", () -> {
      return inputs.pivotAppliedVoltage;
    }, null);
    builder.addDoubleProperty("DesiredAngle", () -> {
      return desiredAngle;
    }, null);
  }
}
