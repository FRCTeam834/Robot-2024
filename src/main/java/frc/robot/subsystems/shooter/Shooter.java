// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
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
  private static final TunableNumber pivotkI = new TunableNumber("shooter/pivotkI");

  private static final TunableNumber pivotkS = new TunableNumber("shooter/pivotkS");
  private static final TunableNumber pivotkG = new TunableNumber("shooter/pivotkG");
  private static final TunableNumber pivotkV = new TunableNumber("shooter/pivotkV");

  /** Both rollers should have the same control parameters */
  private static final TunableNumber rollerkP = new TunableNumber("shooter/rollerkP");
  private static final TunableNumber rollerkD = new TunableNumber("shooter/rollerkD");

  private static final TunableNumber topRollerkS = new TunableNumber("shooter/topRollerkS");
  private static final TunableNumber topRollerkV = new TunableNumber("shooter/topRollerkV");

  private static final TunableNumber bottomRollerkS = new TunableNumber("shooter/bottomRollerkS");
  private static final TunableNumber bottomRollerkV = new TunableNumber("shooter/bottomRollerkV");

  /** NOTE: tuning values are NOT set here, set them in defaults */
  private ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0, 0);
  private final ProfiledPIDController pivotPID = new ProfiledPIDController(0.0, 0.0, 0.0, 
    new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(300)));


  private double desiredAngle = 0.0; // radians
  //private double desiredRollerSpeeds = 0.0;
  private double desiredTopRollerSpeed = 0.0;
  private double desiredBottomRollerSpeed = 0.0;


  private double currentDist = 0.0;

  /** Defaults (final values) are initialized here */
  static {
    /** */
    pivotkP.initDefault(7);
    pivotkD.initDefault(0);
    pivotkI.initDefault(0);
    pivotkS.initDefault(0.02);
    pivotkG.initDefault(0.12);
    pivotkV.initDefault(1.5);
    /** */
    rollerkP.initDefault(0.0009);
    rollerkD.initDefault(0);
    topRollerkS.initDefault(0.145);
    topRollerkV.initDefault(0.00175);
    bottomRollerkS.initDefault(0.145);
    bottomRollerkV.initDefault(0.00176);
  }

  private static InterpolatingDoubleTreeMap shotAngleTable = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shotAngleToleranceTable = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shotSpeedTable = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shotSpeedToleranceTable = new InterpolatingDoubleTreeMap();
  private static double intakePivotAngle = Units.degreesToRadians(0.95);
  private static double idleSpeed = 0;

  /** Initialize values for shot table */
  static {
    /** key: <horizontal distance m>, value: <pivot angle rad> */
    shotAngleTable.put(0.0, 0.75);
    shotAngleTable.put(1.998, 0.743);
    shotAngleTable.put(2.235, 0.7248);
    shotAngleTable.put(2.576, 0.6954);
    shotAngleTable.put(2.872, 0.6313);
    shotAngleTable.put(3.27, 0.6);
    shotAngleTable.put(3.508, 0.565);
    shotAngleTable.put(3.976, 0.523);
    shotAngleTable.put(100.0, 0.523);
    
    /** key: <horizontal distance m>, value: <rpm> */
    
    shotSpeedTable.put(0.0, 4000.0);
    shotSpeedTable.put(1.998, 4000.0);
    shotSpeedTable.put(2.235, 4000.0);
    shotSpeedTable.put(3.27, 4200.0);
    shotSpeedTable.put(3.976, 4600.0);
    shotSpeedTable.put(100.0, 4600.0);
    
    

    /**
     * TOLERANCES
     */

    /** key: <horizontal distance m>, value: <pivot angle tolerance rad> */
    shotAngleToleranceTable.put(0.0, Units.degreesToRadians(2));
    shotAngleToleranceTable.put(3.976, Units.degreesToRadians(1));
    shotAngleToleranceTable.put(5.0, Units.degreesToRadians(1));
    shotAngleToleranceTable.put(100.0, Units.degreesToRadians(0.75));
     /** key: <horizontal distance m>, value: <tolerance rpm> */
    shotSpeedToleranceTable.put(0.0, 100.0);
    shotSpeedToleranceTable.put(3.976, 100.0);
    shotSpeedToleranceTable.put(5.0, 100.0);
    shotSpeedToleranceTable.put(100.0, 100.0);
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
      pivotPID.setPID(pivotkP.get(), pivotkI.get(), pivotkD.get());
    }
    if (pivotkS.hasChanged(hashCode()) || pivotkG.hasChanged(hashCode()) || pivotkV.hasChanged(hashCode())) {
      pivotFeedforward = new ArmFeedforward(pivotkS.get(), pivotkG.get(), pivotkV.get());
    }
    if (rollerkP.hasChanged(hashCode()) || rollerkD.hasChanged(hashCode())) {
      io.setRollerPID(rollerkP.get(), 0.0, rollerkD.get());
    }
    if (topRollerkS.hasChanged(hashCode()) || topRollerkV.hasChanged(hashCode())) {
      io.setTopRollerFeedforward(topRollerkS.get(), topRollerkV.get());
    }
    if (bottomRollerkS.hasChanged(hashCode()) || bottomRollerkV.hasChanged(hashCode())) {
      io.setBottomRollerFeedforward(bottomRollerkS.get(), bottomRollerkV.get());
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
      //io.setRollerSpeeds(desiredRollerSpeeds);
      io.setDesiredTopRollerSpeed(desiredTopRollerSpeed);
      io.setDesiredBottomRollerSpeed(desiredBottomRollerSpeed);
    }
  }

  public void stop () {
    //pivotPID.reset(inputs.pivotAngle, 0.0);
    pivotStopped = true;
    shooterStopped = true;
    //desiredRollerSpeeds = 0;
    desiredBottomRollerSpeed = 0;
    desiredTopRollerSpeed = 0;
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
    angle = MathUtil.clamp(angle, 0.17, 1.1);
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
    //desiredRollerSpeeds = speeds;
    setDesiredTopRollerSpeed(speeds);
    setDesiredBottomRollerSpeed(speeds);
  }

  public void setDesiredTopRollerSpeed (double speed) {
    shooterStopped = false;
    desiredTopRollerSpeed = speed;
  }

  public void setDesiredBottomRollerSpeed (double speed) {
    shooterStopped = false;
    desiredBottomRollerSpeed = speed;
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
    double setpointRPM = shotSpeedTable.get(dist);
    double setpointAngle = shotAngleTable.get(dist);
    return 
      Math.abs(setpointRPM - getCurrentTopRollerSpeed()) <= toleranceRPM &&
      Math.abs(setpointRPM - getCurrentBottomRollerSpeed()) <= toleranceRPM &&
      Math.abs(setpointAngle - getCurrentPivotAngle()) <= toleranceAngle;
  }

  public boolean atDesiredSetpoint (double toleranceAngle, double toleranceRPM) {
    return 
      Math.abs(desiredTopRollerSpeed - getCurrentTopRollerSpeed()) <= toleranceRPM &&
      Math.abs(desiredBottomRollerSpeed - getCurrentBottomRollerSpeed()) <= toleranceRPM &&
      Math.abs(desiredAngle - getCurrentPivotAngle()) <= toleranceAngle;
  }

  public boolean atDesiredRollerSetpoint (double toleranceRPM) {
    return Math.abs(desiredTopRollerSpeed - getCurrentTopRollerSpeed()) <= toleranceRPM &&
      Math.abs(desiredBottomRollerSpeed - getCurrentBottomRollerSpeed()) <= toleranceRPM &&
      Math.abs(getCurrentTopRollerSpeed() - getCurrentBottomRollerSpeed()) <= 20;
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    /// if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("PivotAngle", this::getCurrentPivotAngle, null);
    builder.addDoubleProperty("TopRollerSpeed", this::getCurrentTopRollerSpeed, null);
    builder.addDoubleProperty("DesiredSpeed", () -> { 
      return desiredTopRollerSpeed; }, null);
    builder.addDoubleProperty("BottomRollerSpeed", this::getCurrentBottomRollerSpeed, null);
    builder.addDoubleProperty("AppliedVoltage", () -> {
      return inputs.pivotAppliedVoltage;
    }, null);
    builder.addDoubleProperty("DesiredAngle", () -> {
      return desiredAngle;
    }, null);
    builder.addDoubleProperty("SetpointError", () -> {
      return pivotPID.getPositionError();
    }, null);
  }
}
