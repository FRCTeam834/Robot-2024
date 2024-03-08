package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class ShooterIOSparkMAX implements ShooterIO {

    private final CANSparkFlex topRollerMotor;
    private final CANSparkFlex bottomRollerMotor;
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder topRollerEncoder;
    private final RelativeEncoder bottomRollerEncoder;
    private final AbsoluteEncoder pivotEncoder;

    private SparkPIDController topRollerController;
    private SparkPIDController bottomRollerController;
    private SimpleMotorFeedforward topRollerFeedforward = new SimpleMotorFeedforward(0, 0);
    private SimpleMotorFeedforward bottomRollerFeedforward = new SimpleMotorFeedforward(0, 0);

    public ShooterIOSparkMAX() {
        topRollerMotor = new CANSparkFlex(12, MotorType.kBrushless);
        bottomRollerMotor = new CANSparkFlex(13, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(11, MotorType.kBrushless);

        CANSparkFlex[] motors = { topRollerMotor, bottomRollerMotor };

        for (CANSparkFlex motor : motors) {
            configureSpark("roller restore factory", () -> { return motor.restoreFactoryDefaults(); });
            configureSpark("roller enable voltage comp", () -> { return motor.enableVoltageCompensation(12.0); });
        }

        configureSpark("", () -> { return pivotMotor.restoreFactoryDefaults(); });
        configureSpark("", () -> { return pivotMotor.enableVoltageCompensation(12.0); });

        topRollerController = topRollerMotor.getPIDController();
        bottomRollerController = bottomRollerMotor.getPIDController();

        topRollerEncoder = topRollerMotor.getEncoder();
        bottomRollerEncoder = bottomRollerMotor.getEncoder();

        configureSpark("", () -> { return topRollerController.setFeedbackDevice(topRollerEncoder); });
        configureSpark("", () -> { return bottomRollerController.setFeedbackDevice(bottomRollerEncoder); });

        setRollerPID(0.0, 0.0, 0.0);

        configureSpark("", () -> { return topRollerEncoder.setAverageDepth(4); });
        configureSpark("", () -> { return topRollerEncoder.setMeasurementPeriod(10); });
        configureSpark("", () -> { return bottomRollerEncoder.setAverageDepth(4); });
        configureSpark("", () -> { return bottomRollerEncoder.setMeasurementPeriod(10); });
        
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        configureSpark("", () -> { return pivotEncoder.setInverted(true); });
        configureSpark("", () -> { return pivotEncoder.setPositionConversionFactor(Math.PI * 2); });
        configureSpark("", () -> { return pivotEncoder.setVelocityConversionFactor(Math.PI * 2 / 60); });
        configureSpark("", () -> { return pivotEncoder.setZeroOffset(4.073166 - 0.1570796327); });
        configureSpark("", () -> { return pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); });

        topRollerMotor.setInverted(false);
        bottomRollerMotor.setInverted(true);

        configureSpark("", () -> { return topRollerMotor.setIdleMode(IdleMode.kCoast); });
        configureSpark("", () -> { return bottomRollerMotor.setIdleMode(IdleMode.kCoast); });
        configureSpark("", () -> { return pivotMotor.setIdleMode(IdleMode.kCoast); });

        configureSpark("", () -> { return topRollerMotor.setSmartCurrentLimit(40); });
        configureSpark("", () -> { return bottomRollerMotor.setSmartCurrentLimit(40); });
        configureSpark("", () -> { return pivotMotor.setSmartCurrentLimit(40); });

        // absolute encoder is 1:1
        configureSpark("", () -> { return pivotEncoder.setPositionConversionFactor(2 * Math.PI); });
        configureSpark("", () -> { return pivotEncoder.setVelocityConversionFactor(2 * Math.PI / 60); });

        //TODO: what do?
        pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);

        configureSpark("", () -> { return topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); });
        configureSpark("", () -> { return bottomRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); });
        configureSpark("", () -> { return topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); });
        configureSpark("", () -> { return bottomRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); });
        configureSpark("", () -> { return topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); });
        configureSpark("", () -> { return bottomRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); });
        configureSpark("", () -> { return topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); });
        configureSpark("", () -> { return bottomRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); });
        configureSpark("", () -> { return topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); });
        configureSpark("", () -> { return bottomRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); });

        configureSpark("", () -> { return topRollerMotor.setClosedLoopRampRate(0.5); });
        configureSpark("", () -> { return bottomRollerMotor.setClosedLoopRampRate(0.5); });

        if(Constants.robotMode == RobotMode.COMPETITION) {
            Timer.delay(0.2);
            configureSpark("", () -> { return topRollerMotor.burnFlash(); });
            configureSpark("", () -> { return bottomRollerMotor.burnFlash(); });
            configureSpark("", () -> { return pivotMotor.burnFlash(); });
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
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.pivotAngle = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
        inputs.topRollerVelocity = topRollerEncoder.getVelocity();
        inputs.bottomRollerVelocity = bottomRollerEncoder.getVelocity();
        inputs.pivotAppliedVoltage = pivotMotor.getAppliedOutput();
    }

    @Override
    public void setRollerSpeeds (double speeds) {
        topRollerController.setReference(speeds, ControlType.kVelocity, 0, topRollerFeedforward.calculate(speeds));
        bottomRollerController.setReference(speeds, ControlType.kVelocity, 0, bottomRollerFeedforward.calculate(speeds));
    }

    public void setRollerPID (double kP, double kI, double kD) {
        configureSpark("", () -> { return topRollerController.setP(kP); });
        configureSpark("", () -> { return topRollerController.setI(kI); });
        configureSpark("", () -> { return topRollerController.setD(kD); });
        configureSpark("", () -> { return bottomRollerController.setP(kP); });
        configureSpark("", () -> { return bottomRollerController.setI(kI); });
        configureSpark("", () -> { return bottomRollerController.setD(kD); });
    }

    public void setTopRollerFeedforward (double kS, double kV) {
        topRollerFeedforward = new SimpleMotorFeedforward(kS, kV);
    }

    public void setBottomRollerFeedforward (double kS, double kV) {
        bottomRollerFeedforward = new SimpleMotorFeedforward(kS, kV);
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