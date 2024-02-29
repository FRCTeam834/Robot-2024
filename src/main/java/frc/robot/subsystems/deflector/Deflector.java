// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deflector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.deflector.DeflectorIO;
import frc.robot.subsystems.deflector.DeflectorIO.DeflectorIOInputs;
import frc.robot.utility.TunableNumber;

public class Deflector extends SubsystemBase {
  /** Creates a new Deflector. */
  private final DeflectorIO io;
  private final DeflectorIOInputs inputs = new DeflectorIOInputs();

  boolean isLifted = true;
  //In degrees
  private double desiredAngle = 0.0;


  //Tunable feedforward constants
  private static final TunableNumber deflectorkS = new TunableNumber("deflector/deflectorkS");
  private static final TunableNumber deflectorkG = new TunableNumber("deflector/deflectorkG");
  private static final TunableNumber deflectorkV = new TunableNumber("deflector/deflectorkV");

  private ArmFeedforward deflectorFeedforward = new ArmFeedforward(0.0,0.0,0.0);
  
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

  public void stop() {
    io.stopDeflector();
  }

  public void setVoltage (double voltage) {
    io.setDeflectorVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    //!Might need to flip this
    /*if (isLifted) {
      io.stopDeflector();
    } else {
      io.setDeflectorVoltage(deflectorFeedforward.calculate(desiredAngle*3.14159265/180, 0.0));
    }
    
    if (deflectorkS.hasChanged(hashCode()) || deflectorkG.hasChanged(hashCode()) || deflectorkV.hasChanged(hashCode())) {
      deflectorFeedforward = new ArmFeedforward(deflectorkS.get(), deflectorkG.get(), deflectorkV.get());
    }
*/
    if (DriverStation.isDisabled()) {
      io.stopDeflector();
      return;
    }

  }
}