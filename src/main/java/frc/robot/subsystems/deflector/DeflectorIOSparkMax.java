// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deflector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DeflectorIOSparkMax implements DeflectorIO {
  /** Creates a new DeflectorIOSparkMax. */
  private final CANSparkMax deflectorMotor;
  private final RelativeEncoder deflectorEncoder;
  
  public DeflectorIOSparkMax() {
    deflectorMotor = new CANSparkMax(9, MotorType.kBrushless); //! Add correct ID for the deflector motor
    deflectorEncoder = deflectorMotor.getEncoder();

    deflectorMotor.restoreFactoryDefaults();
    deflectorMotor.setIdleMode(IdleMode.kBrake);
    deflectorMotor.setSmartCurrentLimit(20); //! Need correct current limit

    //! Burn flash needed?

  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    inputs.deflectorAngle = deflectorEncoder.getPosition();
    inputs.deflectorVelocity = deflectorEncoder.getVelocity();
  }

  @Override
  public void setDeflectorVoltage(double voltage) {
    deflectorMotor.setVoltage(voltage);
  }
  

}
