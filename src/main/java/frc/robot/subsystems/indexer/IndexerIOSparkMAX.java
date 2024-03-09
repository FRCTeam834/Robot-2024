package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class IndexerIOSparkMAX implements IndexerIO {
    private final CANSparkMax motor;
    private final DigitalInput frontBeamBreak; // detects when note first comes through
    private final DigitalInput backBeamBreak; // detects when to stop note to hold

    public IndexerIOSparkMAX() {
        motor = new CANSparkMax(17, MotorType.kBrushless);

        // channel # on DIO
        frontBeamBreak = new DigitalInput(9);
        backBeamBreak = new DigitalInput(8);

        configureSpark("", () -> { return motor.restoreFactoryDefaults(); });
        configureSpark("", () -> { return motor.setIdleMode(IdleMode.kBrake); });
        configureSpark("", () -> { return motor.enableVoltageCompensation(12.0); });
        motor.setInverted(false);
        configureSpark("", () -> { return motor.setSmartCurrentLimit(40); });
        configureSpark("", () -> { return motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); });
        configureSpark("", () -> { return motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); });
        configureSpark("", () -> { return motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); });
        configureSpark("", () -> { return motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); });
        configureSpark("", () -> { return motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); });

        if(Constants.robotMode == RobotMode.COMPETITION) {
            Timer.delay(0.2);
            configureSpark("", () -> { return motor.burnFlash(); });
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
    public void updateInputs(IndexerIOInputs inputs) {
        // our sensors are normally open
        inputs.noteIsDetectedFront = !frontBeamBreak.get();
        inputs.noteIsDetectedBack = !backBeamBreak.get();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
