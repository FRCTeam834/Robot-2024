// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deflector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.deflector.DeflectorIO;

public class Deflector extends SubsystemBase {
  /** Creates a new Deflector. */
  private final DeflectorIO io;
  private final DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  private boolean isLifted = true;
  //In degrees
  private double desiredAngle = 0.0;
  private final double VOLTS_TO_HOLD_THE_DEFLECTOR = 0.1; //! Add the correct voltage here

  //Tunable feedforward constants
  private static final LoggedTunableNumber deflectorkS = new LoggedTunableNumber("deflector/deflectorkS");
  private static final LoggedTunableNumber deflectorkG = new LoggedTunableNumber("deflector/deflectorkG");
  private static final LoggedTunableNumber deflectorkV = new LoggedTunableNumber("deflector/deflectorkV");

  private ArmFeedforward deflectorFeedforward = new ArmFeedforward(0.0,0.0,0.0);
  
  // Limit switch
  private static final DigitalInput limitSwitch = new DigitalInput(0); //! Put the correct channel here
  
  //! Need the right defaults here
  static {
    deflectorkS.initDefault(0.0);
    deflectorkG.initDefault(0.0);
    deflectorkV.initDefault(0.0);
  }



  public Deflector(DeflectorIO io) {
    this.io = io;
    SmartDashboard.putData(this);
  }

  public void goToScoringPosition() {
    desiredAngle = -90; //! Add correct angle here
  }

  public void goToNeutralPosition() {
    desiredAngle = 0.0; //! Add correct angle here
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("deflector", inputs);
    if (limitSwitch.get()) {
      // Set the voltage to a small number to keep the limit switch up
      isLifted = true;
    } else {
      isLifted = false;
    }
    if (isLifted) {
      io.setDeflectorVoltage(VOLTS_TO_HOLD_THE_DEFLECTOR);
    } else {
      io.setDeflectorVoltage(deflectorFeedforward.calculate(desiredAngle*3.14159/180, 0.0));
    }
    
    if (deflectorkS.hasChanged(hashCode()) || deflectorkG.hasChanged(hashCode()) || deflectorkV.hasChanged(hashCode())) {
      deflectorFeedforward = new ArmFeedforward(deflectorkS.get(), deflectorkG.get(), deflectorkV.get());
    }

    if (DriverStation.isDisabled()) {
      //stop();
      return;
    }

  }
}
