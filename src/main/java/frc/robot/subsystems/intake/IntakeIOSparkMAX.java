package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class IntakeIOSparkMAX implements IntakeIO {
    private final CANSparkMax rollerMotor;

    public IntakeIOSparkMAX() {
        rollerMotor = new CANSparkMax(16, MotorType.kBrushless);

        configureSpark("", () -> { return rollerMotor.restoreFactoryDefaults(); });
        configureSpark("", () -> { return rollerMotor.setIdleMode(IdleMode.kCoast); });
        configureSpark("", () -> { return rollerMotor.enableVoltageCompensation(12.0); });
        configureSpark("", () -> { return rollerMotor.setSmartCurrentLimit(40); });
        configureSpark("", () -> { return rollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); });
        configureSpark("", () -> { return rollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); });
        configureSpark("", () -> { return rollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); });
        configureSpark("", () -> { return rollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); });
        configureSpark("", () -> { return rollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); });

        rollerMotor.setInverted(true);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            Timer.delay(0.25);
            configureSpark("", () -> { return rollerMotor.burnFlash(); });
            Timer.delay(0.25);
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
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void setVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }
}
