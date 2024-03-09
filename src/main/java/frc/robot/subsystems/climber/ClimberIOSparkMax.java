package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax rightArmMotor; //Motor is a Swerve
    private final CANSparkMax leftArmMotor; //Motor is a Swerve
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    public ClimberIOSparkMax() {
        rightArmMotor = new CANSparkMax(15, MotorType.kBrushless); //unknown ID for SparkMax
        leftArmMotor = new CANSparkMax(14, MotorType.kBrushless); //unknown ID for SparkMax
        encoderRight = rightArmMotor.getEncoder();
        encoderLeft = leftArmMotor.getEncoder();

        configureSpark("", () -> { return rightArmMotor.restoreFactoryDefaults(); });
        configureSpark("", () -> { return leftArmMotor.restoreFactoryDefaults(); });
        configureSpark("", () -> { return rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); });
        configureSpark("", () -> { return leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); });
        rightArmMotor.setInverted(false); //check before running
        leftArmMotor.setInverted(false); //check before running
        configureSpark("", () -> { return rightArmMotor.enableVoltageCompensation(12); });
        configureSpark("", () -> { return leftArmMotor.enableVoltageCompensation(12); });
        configureSpark("", () -> { return rightArmMotor.setSmartCurrentLimit(40); });
        configureSpark("", () -> { return leftArmMotor.setSmartCurrentLimit(40); });

        configureSpark("", () -> { return rightArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); });
        configureSpark("", () -> { return leftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); });
        configureSpark("", () -> { return rightArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); });
        configureSpark("", () -> { return leftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); });
        configureSpark("", () -> { return rightArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); });
        configureSpark("", () -> { return leftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); });
        configureSpark("", () -> { return rightArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); });
        configureSpark("", () -> { return leftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); });
        configureSpark("", () -> { return rightArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); });
        configureSpark("", () -> { return leftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); });

        if(Constants.robotMode == RobotMode.COMPETITION) {
            Timer.delay(0.2);
            configureSpark("", () -> { return rightArmMotor.burnFlash(); });
            configureSpark("", () -> { return leftArmMotor.burnFlash(); });
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

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.rightSwerveVelocity = encoderRight.getVelocity();
        inputs.leftSwerveVelocity = encoderLeft.getVelocity();
        inputs.rightArmHeight = encoderRight.getPosition(); //position is calculated in rotations
        inputs.leftArmHeight = encoderLeft.getPosition(); //ditto
    }

    public void setMotorVoltage(double rightVolts, double leftVolts) {
        rightArmMotor.setVoltage(rightVolts);
        leftArmMotor.setVoltage(leftVolts);
    }

    public void setCurrentLimit (int currentLimit) {
        rightArmMotor.setSmartCurrentLimit(currentLimit);
        leftArmMotor.setSmartCurrentLimit(currentLimit);
    }

    public void zeroRightClimber () {
        encoderRight.setPosition(0.0);
    }

    public void zeroLeftClimber () {
        encoderLeft.setPosition(0.0);
    }
}