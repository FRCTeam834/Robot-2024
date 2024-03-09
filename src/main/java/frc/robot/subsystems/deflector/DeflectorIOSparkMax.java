// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deflector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import java.util.function.Supplier;

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

    configureSpark("", () -> { return deflectorMotor.restoreFactoryDefaults(); });
    configureSpark("", () -> { return deflectorMotor.setIdleMode(IdleMode.kCoast); });
    configureSpark("", () -> { return deflectorMotor.setSmartCurrentLimit(20); }); //! Need correct current limit

    configureSpark("", () -> { return deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); });
    configureSpark("", () -> { return deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); });
    configureSpark("", () -> { return deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); });
    configureSpark("", () -> { return deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); });
    configureSpark("", () -> { return deflectorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); });

    //! Burn flash needed?
    if (Constants.robotMode == RobotMode.COMPETITION) {
      Timer.delay(0.25);
      configureSpark("", () -> { return deflectorMotor.burnFlash(); });
      Timer.delay(0.25);
    }
  }

  public static boolean configureSpark(String message, Supplier<REVLibError> config) {
        REVLibError err = REVLibError.kOk;
        for (int i = 0; i < 10; i++) {
            err = config.get();
            if (err == REVLibError.kOk) {
                return true;
            }
        }

        DriverStation.reportError(String.format(
            "[MergeError] - CANSparkMax failed to configure setting. MergeMessage: %s. Spark error code: %s \nSee stack trace below.", 
            message,
            err.toString()), 
            true);
            
        return false;
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