// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  /** Stores setpoint "speeds" (voltages) for indexer */
  public static enum Setpoint {
    FAST(12.0),
    SLOW(8.0),
    FEED(4.0),
    STOP(0.0);

    public final double voltage;

    private Setpoint (double voltage) {
      this.voltage = voltage;
    }
  };

  public Indexer(IndexerIO io) {
    this.io = io;
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }
  }

  public void setSetpoint (Setpoint setpoint) {
    setVoltage(setpoint.voltage);
  }

  public void setVoltage (double voltage) {
    io.setVoltage(voltage);
  }

  public boolean noteDetectedIntakeSide () {
    return inputs.noteIsDetectedFront;
  }

  public boolean noteDetectedShooterSide () {
    return inputs.noteIsDetectedBack;
  }

  public boolean hasNote () {
    return noteDetectedIntakeSide() || noteDetectedShooterSide();
  }

  public void stop () {
    setVoltage(0);
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("Indexer");

    builder.addBooleanProperty("noteDetectedBack", () -> { return inputs.noteIsDetectedBack; }, null);
    builder.addBooleanProperty("noteDetectedFront", () -> { return inputs.noteIsDetectedFront; }, null);
  }
}
