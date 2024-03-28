// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO io;

  /** Stores setpoint "speeds" (voltages) for intake */
  public static enum Setpoint {
    FAST(8.0),
    SLOW(6.0), // 8
    SLOWREVERSE(-3.0),
    STOP(0.0);

    public final double voltage;

    private Setpoint (double voltage) {
      this.voltage = voltage;
    }
  };

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
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

  public void stop () {
    setVoltage(0);
  }
}
