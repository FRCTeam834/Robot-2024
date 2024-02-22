// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.utility.LEDColors;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private Spark blinkin;
  public final int LED_PWM_PORT = 0;
  
  public LEDs() {
    blinkin = new Spark(LED_PWM_PORT);
    //!Set default color here
    blinkin.set(LEDColors.WHITE);
  }

  public void setLEDColor(double colorValue){
    //see LED Color utility for the right LED colors
    blinkin.set(colorValue);
  }

  public void defaultColor() {
    /*
    if (alliance is red) {
      blinkin.set(LEDColors.RED_PULSE);
    } else if (alliance is blue) {
      blinkin.set(LEDColors.BLUE_PULSE);
    } else {
      blinkin.set(LEDColors.WHITE);
    }
     */
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
