package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class ShooterIOSparkMAX implements ShooterIO {

    private final CANSparkMax topRollerMotor;
    private final CANSparkMax bottomRollerMotor;
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder topRollerEncoder;
    private final RelativeEncoder bottomRollerEncoder;
    private final AbsoluteEncoder pivotEncoder;

    private SparkPIDController topRollerController;
    private SparkPIDController bottomRollerController;
    private SimpleMotorFeedforward rollerFeedforward = new SimpleMotorFeedforward(0, 0);

    public ShooterIOSparkMAX() {
        topRollerMotor = new CANSparkMax(12, MotorType.kBrushless);
        bottomRollerMotor = new CANSparkMax(13, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(11, MotorType.kBrushless);

        CANSparkMax[] motors = { topRollerMotor, bottomRollerMotor, pivotMotor };

        for (CANSparkMax motor : motors) {
            motor.restoreFactoryDefaults();
            motor.enableVoltageCompensation(12.0);
        }

        topRollerController = topRollerMotor.getPIDController();
        bottomRollerController = bottomRollerMotor.getPIDController();

        topRollerEncoder = topRollerMotor.getEncoder();
        bottomRollerEncoder = bottomRollerMotor.getEncoder();

        topRollerController.setFeedbackDevice(topRollerEncoder);
        bottomRollerController.setFeedbackDevice(bottomRollerEncoder);

        setRollerPID(0.0, 0.0, 0.0);

        topRollerEncoder.setAverageDepth(2);
        topRollerEncoder.setMeasurementPeriod(10);
        bottomRollerEncoder.setAverageDepth(2);
        bottomRollerEncoder.setMeasurementPeriod(10);
        
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setInverted(true);
        pivotEncoder.setPositionConversionFactor(Math.PI * 2);
        pivotEncoder.setVelocityConversionFactor(Math.PI * 2 / 60);
        pivotEncoder.setZeroOffset(4.0828111 - 0.1570796327);
        pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);

        topRollerMotor.setInverted(false);
        bottomRollerMotor.setInverted(true);

        topRollerMotor.setIdleMode(IdleMode.kCoast);
        bottomRollerMotor.setIdleMode(IdleMode.kCoast);
        pivotMotor.setIdleMode(IdleMode.kCoast);

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
        inputs.pivotAppliedVoltage = pivotMotor.getAppliedOutput();
    }

    @Override
    public void setRollerSpeeds (double speeds) {
        topRollerController.setReference(speeds, ControlType.kVelocity, 0, rollerFeedforward.calculate(speeds));
        bottomRollerController.setReference(speeds, ControlType.kVelocity, 0, rollerFeedforward.calculate(speeds));
    }

    public void setRollerPID (double kP, double kI, double kD) {
        topRollerController.setP(kP);
        topRollerController.setI(kI);
        topRollerController.setD(kD);
        bottomRollerController.setP(kP);
        bottomRollerController.setI(kI);
        bottomRollerController.setD(kD);
    }

    public void setRollerFeedforward (double kS, double kV) {
        rollerFeedforward = new SimpleMotorFeedforward(kS, kV);
    }

    public void setTopRollerVoltage(double volts) {
        topRollerMotor.setVoltage(volts);
    }

    public void setBottomRollerVoltage(double volts) {
        bottomRollerMotor.setVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
}