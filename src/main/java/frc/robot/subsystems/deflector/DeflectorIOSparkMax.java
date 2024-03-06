// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deflector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;


public class DeflectorIOSparkMax implements DeflectorIO {
  /** Creates a new DeflectorIOSparkMax. */
  private final CANSparkMax deflectorMotor;
  private final RelativeEncoder deflectorEncoder;
  
  public DeflectorIOSparkMax() {
    deflectorMotor = new CANSparkMax(10, MotorType.kBrushless); //! Add correct ID for the deflector motor
    deflectorEncoder = deflectorMotor.getEncoder();

    deflectorMotor.restoreFactoryDefaults();
    deflectorMotor.setIdleMode(IdleMode.kCoast);
    deflectorMotor.setSmartCurrentLimit(20); //! Need correct current limit

    deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

    //! Burn flash needed?
    if (Constants.robotMode == RobotMode.COMPETITION) {
      Timer.delay(0.2);
      deflectorMotor.burnFlash();
    }
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

  @Override
  public void stopDeflector(){
    deflectorMotor.setVoltage(0);
  }
  

}