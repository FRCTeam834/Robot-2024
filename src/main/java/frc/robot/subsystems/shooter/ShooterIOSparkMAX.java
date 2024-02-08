package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.Constants.RobotMode;
import frc.robot.Constants;

public class ShooterIOSparkMAX implements ShooterIO {

    private final CANSparkMax rollerMotor0;
    private final CANSparkMax rollerMotor1;
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder rollerEncoder0;
    private final RelativeEncoder rollerEncoder1;
    private final RelativeEncoder pivotEncoder;
    public static final double PIVOT_GEAR_REDUCTION = 1;

    public ShooterIOSparkMAX() {
        rollerMotor0 = new CANSparkMax(4, MotorType.kBrushless);
        rollerMotor1 = new CANSparkMax(5, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(8, MotorType.kBrushless);

        rollerEncoder0 = rollerMotor0.getEncoder();
        rollerEncoder1 = rollerMotor1.getEncoder();
        pivotEncoder = pivotMotor.getEncoder();

        CANSparkMax[] motors = {rollerMotor0, rollerMotor1, pivotMotor};

        for (CANSparkMax motor : motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(IdleMode.kBrake);
            motor.enableVoltageCompensation(12.0);
        }

        rollerMotor0.setInverted(false);
        rollerMotor1.setInverted(true);
        rollerMotor0.setSmartCurrentLimit(50);
        rollerMotor1.setSmartCurrentLimit(50);
        pivotMotor.setSmartCurrentLimit(20);

        pivotEncoder.setPositionConversionFactor(2 * Math.PI / PIVOT_GEAR_REDUCTION);
        pivotEncoder.setVelocityConversionFactor(2 * Math.PI / (PIVOT_GEAR_REDUCTION * 60));

        //TODO: what do?
        pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            rollerMotor0.burnFlash();
            rollerMotor1.burnFlash();
            pivotMotor.burnFlash();
        }

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.pivotAngle = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
        inputs.roller0Velocity = rollerEncoder0.getVelocity();
        inputs.roller1Velocity = rollerEncoder1.getVelocity();
    }

    @Override
    public void setTopRollerVoltage(double volts) {
        rollerMotor0.setVoltage(volts);
    }

    @Override
    public void setBottomRollerVoltage(double volts) {
        rollerMotor1.setVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
}