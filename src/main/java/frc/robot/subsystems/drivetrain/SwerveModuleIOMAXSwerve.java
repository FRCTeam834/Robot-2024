package frc.robot.subsystems.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class SwerveModuleIOMAXSwerve implements SwerveModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax steerSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder steerEncoder;

    private final double wheelDiameter = Units.inchesToMeters(3.0);
    private final double driveEncoderGearing = 4.71429;
    private final double steerEncoderGearing = 1;

    public SwerveModuleIOMAXSwerve (int index) {
        /** CAN IDs and encoder offsets are different for each module */
        switch (index) {
            /** Front Left */
            case 0: {
                driveSparkMax = new CANSparkMax(0, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setZeroOffset(0);
                break;
            }
            /** Front Right */
            case 1: {
                driveSparkMax = new CANSparkMax(0, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setZeroOffset(0);
                break;
            }
            /** Back Left */
            case 2: {
                driveSparkMax = new CANSparkMax(0, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setZeroOffset(0);
                break;
            }
            /** Back Right */
            case 3: {
                driveSparkMax = new CANSparkMax(0, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setZeroOffset(0);
                break;
            }
            default: throw new RuntimeException("Invalid SwerveModuleIOMAXSwerve index!");
        }

        /** Everything else is common between every swerve module */

        driveEncoder = driveSparkMax.getEncoder();

        driveSparkMax.restoreFactoryDefaults();
        steerSparkMax.restoreFactoryDefaults();

        driveSparkMax.setIdleMode(IdleMode.kBrake);
        steerSparkMax.setIdleMode(IdleMode.kBrake);
        driveSparkMax.enableVoltageCompensation(12.0);
        steerSparkMax.enableVoltageCompensation(12.0);
        driveSparkMax.setSmartCurrentLimit(40);
        steerSparkMax.setSmartCurrentLimit(20);

        // rpm -> m/s
        driveEncoder.setVelocityConversionFactor(Math.PI * wheelDiameter / (60 * driveEncoderGearing));
        // revs -> meters
        driveEncoder.setPositionConversionFactor(Math.PI * wheelDiameter / driveEncoderGearing);
        // revs -> radians
        steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing);
        steerEncoder.setInverted(true); // MAXSwerve has steer gearing reversed

        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10); // motor position frame
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);

        if (Constants.robotMode == RobotMode.COMPETITION) {
            driveSparkMax.burnFlash();
            steerSparkMax.burnFlash();
        }
    }

    public void updateInputs (SwerveModuleIOInputs inputs) {
        inputs.driveVelocity = driveEncoder.getVelocity();
        inputs.steerAngle = steerEncoder.getPosition();
    }

    public void setDriveVoltage (double voltage) {
        driveSparkMax.setVoltage(voltage);
    }

    public void setSteerVoltage (double voltage) {
        steerSparkMax.setVoltage(voltage);
    }
}
