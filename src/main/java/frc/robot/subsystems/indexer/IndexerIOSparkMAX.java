package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class IndexerIOSparkMAX implements IndexerIO {
    private final CANSparkMax motor;
    private final DigitalInput frontBeamBreak; // detects when note first comes through
    private final DigitalInput backBeamBreak; // detects when to stop note to hold

    public IndexerIOSparkMAX() {
        motor = new CANSparkMax(17, MotorType.kBrushless);

        // channel # on DIO
        frontBeamBreak = new DigitalInput(0);
        backBeamBreak = new DigitalInput(1);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(20);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            motor.burnFlash();
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // our sensors are normally open
        inputs.noteIsDetectedFront = frontBeamBreak.get();
        inputs.noteIsDetectedBack = backBeamBreak.get();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
