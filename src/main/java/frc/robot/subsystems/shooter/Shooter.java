// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final PIDController pivotPID = new PIDController(0, 0, 0);

  private final PIDController topRollerPID = new PIDController(0, 0, 0);
  private final PIDController bottomRollerPID = new PIDController(0, 0, 0);

  private SimpleMotorFeedforward rollerFeedforward = new SimpleMotorFeedforward(0, 0);

  private double desiredAngle = 0.0; // radians
  private double desiredRollerSpeeds = 0.0; // rpm

  /** Defaults (final values) are initialized here */
  static {
    /** */
    pivotkP.initDefault(0);
    pivotkD.initDefault(0);
    pivotkS.initDefault(0);
    pivotkG.initDefault(0);
    pivotkV.initDefault(0);
    /** */
    rollerkP.initDefault(0);
    rollerkD.initDefault(0);
    rollerkS.initDefault(0);
    rollerkV.initDefault(0);
  }

  private static InterpolatingDoubleTreeMap shotTable = new InterpolatingDoubleTreeMap();

  /** Initialize values for shot table */
  static {
    /** key: <horizontal distance m>, value: <pivot angle rad> */
    shotTable.put(0.0, 0.0);
    shotTable.put(1.0, 1.0);
  }

  public Shooter(ShooterIO io) {
    this.io = io;

    // * Q:Is this still used for the pivot? A: Creo que sí
    pivotPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (pivotkP.hasChanged() || pivotkD.hasChanged()) {
      pivotPID.setPID(pivotkP.get(), 0.0, pivotkD.get());
    }
    if (pivotkS.hasChanged() || pivotkG.hasChanged() || pivotkV.hasChanged()) {
      pivotFeedforward = new ArmFeedforward(pivotkS.get(), pivotkG.get(), pivotkV.get());
    }
    if (rollerkP.hasChanged() || rollerkD.hasChanged()) {
      topRollerPID.setPID(rollerkP.get(), 0.0, rollerkD.get());
      bottomRollerPID.setPID(rollerkP.get(), 0.0, rollerkD.get());
    }
    if (rollerkS.hasChanged() || rollerkV.hasChanged()) {
      rollerFeedforward = new SimpleMotorFeedforward(rollerkS.get(), rollerkV.get());
    }
    
    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    if (!pivotStopped) {
      io.setPivotVoltage(
        pivotFeedforward.calculate(desiredAngle, 0.0) +
        pivotPID.calculate(inputs.pivotAngle, desiredAngle));
    }
    if (!shooterStopped) {
      io.setTopRollerVoltage(
        rollerFeedforward.calculate(desiredRollerSpeeds) +
        topRollerPID.calculate(inputs.topRollerVelocity, desiredRollerSpeeds));
      io.setBottomRollerVoltage(
        rollerFeedforward.calculate(desiredRollerSpeeds) +
        bottomRollerPID.calculate(inputs.bottomRollerVelocity, desiredRollerSpeeds));
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
  }

  /**
   * 
   * @param dist flat distance to speaker meters
   * @return radians
   */
  public double getPivotAngleForDistance (double dist) {
    return shotTable.get(dist);
  }

  /**
   * 
   * @param speeds rpm
   */
  public void setDesiredRollerSpeeds (double speeds) {
    shooterStopped = false;
    desiredRollerSpeeds = speeds;
  }
}
