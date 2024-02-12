package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class IntakeIOSparkMAX implements IntakeIO {
    private final CANSparkMax topRollerMotor;
    private final CANSparkMax bottomRollerMotor;

    public IntakeIOSparkMAX() {
        topRollerMotor = new CANSparkMax(123, MotorType.kBrushless);
        bottomRollerMotor = new CANSparkMax(124, MotorType.kBrushless);

        CANSparkMax[] motors = { topRollerMotor, bottomRollerMotor };

        for (CANSparkMax motor : motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(IdleMode.kBrake);
            motor.enableVoltageCompensation(12.0);
            motor.setSmartCurrentLimit(20);
        }

        topRollerMotor.setInverted(false);
        bottomRollerMotor.setInverted(true);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            topRollerMotor.burnFlash();
            bottomRollerMotor.burnFlash();
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void setVoltage(double voltage) {
        topRollerMotor.setVoltage(voltage);
        bottomRollerMotor.setVoltage(voltage);
    }
}
