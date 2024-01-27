package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;



public class IndexerIOSparkMax implements IndexerIO {
    private final CANSparkMax roller2Motor;
    private final CANSparkMax roller3Motor;
    private final RelativeEncoder encoder2;
    private final RelativeEncoder encoder3;
    private final DigitalInput beamBreaker;

    public IndexerIOSparkMax() {
        roller2Motor = new CANSparkMax(2389, MotorType.kBrushless);
        roller3Motor = new CANSparkMax(2389, MotorType.kBrushless);
        encoder2 = roller2Motor.getEncoder();
        encoder3 = roller3Motor.getEncoder();

        // channel # on DIO
        beamBreaker = new DigitalInput(0);


        roller2Motor.restoreFactoryDefaults();
        roller3Motor.restoreFactoryDefaults();
        roller2Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        roller3Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        roller2Motor.setInverted(false);
        roller3Motor.setInverted(true);
        roller2Motor.enableVoltageCompensation(12.0);
        roller3Motor.enableVoltageCompensation(12.0);
        roller2Motor.setSmartCurrentLimit(20);
        roller3Motor.setSmartCurrentLimit(20);
        encoder2.setAverageDepth(2);
        encoder3.setAverageDepth(2);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            roller2Motor.burnFlash();
            roller3Motor.burnFlash();
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.noteIsDetected = !beamBreaker.get();
        inputs.roller2Velocity = encoder2.getVelocity();
        inputs.roller3Velocity = encoder3.getVelocity();
    }

    @Override
    public void setRollerVoltage(double volts) {
        roller2Motor.setVoltage(volts);
        roller3Motor.setVoltage(volts);
    }

    @Override
    public void notifyIndexer(IndexerIOInputs objectDetected) {}
}