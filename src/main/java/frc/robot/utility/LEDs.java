// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private Spark blinkin;
  public final int LED_PWM_PORT = 0;

  private Colors defaultColor = Colors.BLUE;
  private Colors timedColor = null;
  private final Timer timer = new Timer();
  private double timedSeconds = 0.0;

  public static enum Colors {
    BLUE(-0.41),
    STROBEWHITE(-0.05);

    public final double signal;

    private Colors (double signal) {
      this.signal = signal;
    }
  };

  public LEDs() {
    blinkin = new Spark(LED_PWM_PORT);
    setLEDColor(Colors.BLUE);
  }

  public void setColorForTime (Colors color, double seconds) {
    timer.reset();
    timer.start();
    timedSeconds = seconds;
    timedColor = color;
    setLEDColor(color);
  }

  public void cancelColorForTime () {
    timedSeconds = 0.0;
  }

  public void setLEDColor(Colors color){
    //see LED Color utility for the right LED colors
    blinkin.set(color.signal);
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
    if (timedColor != null && timer.hasElapsed(timedSeconds)) {
      timedColor = null;
      timer.reset();
      timer.stop();
      setLEDColor(defaultColor);
    }
    // This method will be called once per scheduler run
  }
}