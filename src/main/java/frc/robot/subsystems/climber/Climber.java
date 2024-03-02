// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  private double rightSpeed = 0.0;
  private double leftSpeed = 0.0;
  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setArmSpeeds (double rightArmSpeed, double leftArmSpeed){
    // !Need constantd and homing sequence etc.

    //if(inputs.rightArmHeight >= Constants.ClimberConstants.minArmHeight 
    //&& inputs.rightArmHeight <= Constants.ClimberConstants.maxArmHeight){
      rightSpeed = rightArmSpeed;
    //} else {
     // inputs.rightSwerveVelocity = 0;
    //}

    //if(inputs.leftArmHeight >= Constants.ClimberConstants.minArmHeight 
    //&& inputs.leftArmHeight <= Constants.ClimberConstants.maxArmHeight){
      leftSpeed = leftArmSpeed;
    //} else {
    //  inputs.leftSwerveVelocity = 0;
    //}
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setMotorVoltage(leftSpeed * 12, //multiplies by 12 for a max of 12 volts
                         rightSpeed * 12);
  }
}