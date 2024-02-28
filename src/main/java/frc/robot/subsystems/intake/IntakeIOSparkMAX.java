package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class IntakeIOSparkMAX implements IntakeIO {
    private final CANSparkMax rollerMotor;

    public IntakeIOSparkMAX() {
        rollerMotor = new CANSparkMax(16, MotorType.kBrushless);

        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setIdleMode(IdleMode.kCoast);
        rollerMotor.enableVoltageCompensation(12.0);
        rollerMotor.setSmartCurrentLimit(30);

        rollerMotor.setInverted(true);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            rollerMotor.burnFlash();
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void setVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }
}
