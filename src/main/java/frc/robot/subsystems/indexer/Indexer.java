// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  /** Stores setpoint "speeds" (voltages) for indexer */
  public static enum Setpoint {
    FAST(2.0),
    SLOW(1.0),
    FEED(1.0),
    STOP(0.0);

    public final double voltage;

    private Setpoint (double voltage) {
      this.voltage = voltage;
    }
  };

  public Indexer(IndexerIO io) {
    this.io = io;
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

  public boolean noteDetectedFront () {
    return inputs.noteIsDetectedFront;
  }

  public boolean noteDetectedBack () {
    return inputs.noteIsDetectedBack;
  }

  public boolean hasNote () {
    return noteDetectedBack();
  }

  public void stop () {
    setVoltage(0);
  }
}
