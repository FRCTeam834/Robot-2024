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
        rightArmMotor.enableVoltageCompensation(0); //replace with realistic value
        leftArmMotor.enableVoltageCompensation(0); //replace with realistic value
        rightArmMotor.setSmartCurrentLimit(0); //replace with realistic value
        leftArmMotor.setSmartCurrentLimit(0); //replace with realistic value
        encoderRight.setAverageDepth(2); //I have no idea what 2 means
        encoderLeft.setAverageDepth(2); //I have no idea what 2 means

        if(Constants.robotMode == RobotMode.COMPETITION) {
            rightArmMotor.burnFlash();
            leftArmMotor.burnFlash();
        }
    }


    public void updateInputs(ClimberIOInputs inputs) {
        inputs.rightSwerveVelocity = encoderRight.getVelocity();
        inputs.leftSwerveVelocity = encoderLeft.getVelocity();
        inputs.rightArmHeight = 0; //write a calculation to find the current height - can be done with voltages
        inputs.leftArmHeight = 0; //write a calculation to find the current height - can be done with voltages
    }

    public void setMotorVoltage(double volts) {
        rightArmMotor.setVoltage(volts);
        leftArmMotor.setVoltage(volts);
    }
}
