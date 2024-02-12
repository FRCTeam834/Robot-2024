package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class ShooterIOSparkMAX implements ShooterIO {

    private final CANSparkMax topRollerMotor;
    private final CANSparkMax bottomRollerMotor;
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder topRollerEncoder;
    private final RelativeEncoder bottomRollerEncoder;
    private final AbsoluteEncoder pivotEncoder;

    public ShooterIOSparkMAX() {
        topRollerMotor = new CANSparkMax(999, MotorType.kBrushless);
        bottomRollerMotor = new CANSparkMax(998, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(997, MotorType.kBrushless);

        topRollerEncoder = topRollerMotor.getEncoder();
        bottomRollerEncoder = bottomRollerMotor.getEncoder();
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setZeroOffset(0.0);

        CANSparkMax[] motors = { topRollerMotor, bottomRollerMotor, pivotMotor };

        for (CANSparkMax motor : motors) {
            motor.restoreFactoryDefaults();
            motor.enableVoltageCompensation(12.0);
        }

        topRollerMotor.setInverted(false);
        bottomRollerMotor.setInverted(true);

        topRollerMotor.setIdleMode(IdleMode.kCoast);
        bottomRollerMotor.setIdleMode(IdleMode.kCoast);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        topRollerMotor.setSmartCurrentLimit(40);
        bottomRollerMotor.setSmartCurrentLimit(40);
        pivotMotor.setSmartCurrentLimit(40);
        
        // absolute encoder is 1:1
        pivotEncoder.setPositionConversionFactor(2 * Math.PI);
        pivotEncoder.setVelocityConversionFactor(2 * Math.PI / 60);

        //TODO: what do?
        pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            topRollerMotor.burnFlash();
            bottomRollerMotor.burnFlash();
            pivotMotor.burnFlash();
        }

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.pivotAngle = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
        inputs.topRollerVelocity = topRollerEncoder.getVelocity();
        inputs.bottomRollerVelocity = bottomRollerEncoder.getVelocity();
    }

    @Override
    public void setTopRollerVoltage(double volts) {
        topRollerMotor.setVoltage(volts);
    }

    @Override
    public void setBottomRollerVoltage(double volts) {
        bottomRollerMotor.setVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
}