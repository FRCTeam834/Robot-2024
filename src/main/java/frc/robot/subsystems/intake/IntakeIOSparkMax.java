package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public IntakeIOSparkMax() {
        motor = new CANSparkMax(6, MotorType.kBrushless);
        //throw new Error("Did not set motor ids for intake.");
        encoder = motor.getEncoder();

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setInverted(false);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(20);
        encoder.setAverageDepth(2);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            motor.burnFlash();
        }
        

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.rpm = encoder.getVelocity();
        inputs.voltage = motor.getBusVoltage();
    }

    // sets the voltage.
    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

}
