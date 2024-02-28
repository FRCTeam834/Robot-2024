package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax rightArmMotor; //Motor is a Swerve
    private final CANSparkMax leftArmMotor; //Motor is a Swerve
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    public ClimberIOSparkMax() {
        rightArmMotor = new CANSparkMax(0, MotorType.kBrushless); //unknown ID for SparkMax
        leftArmMotor = new CANSparkMax(0, MotorType.kBrushless); //unknown ID for SparkMax
        encoderRight = rightArmMotor.getEncoder();
        encoderLeft = leftArmMotor.getEncoder();

        rightArmMotor.restoreFactoryDefaults();
        leftArmMotor.restoreFactoryDefaults();
        rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightArmMotor.setInverted(false); //check before running
        leftArmMotor.setInverted(false); //check before running
        rightArmMotor.enableVoltageCompensation(12);
        leftArmMotor.enableVoltageCompensation(12);
        rightArmMotor.setSmartCurrentLimit(40);
        leftArmMotor.setSmartCurrentLimit(40);

        if(Constants.robotMode == RobotMode.COMPETITION) {
            rightArmMotor.burnFlash();
            leftArmMotor.burnFlash();
        }
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
}