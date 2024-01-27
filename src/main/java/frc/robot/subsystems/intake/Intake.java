package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public double getRPM() {
    return inputs.rpm;
  }

  public void stopIntake () {
    io.setVoltage(0.0);
  }

  public void startIntake() {
    io.setVoltage(5);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Intake", inputs);

  }

}