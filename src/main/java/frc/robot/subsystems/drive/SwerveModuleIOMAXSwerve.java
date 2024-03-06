package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class SwerveModuleIOMAXSwerve implements SwerveModuleIO {
    private final CANSparkFlex driveSparkMax;
    private final CANSparkMax steerSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder steerEncoder;

    private final double wheelDiameter = Units.inchesToMeters(3.0);
    private final double driveEncoderGearing = 3.56;
    private final double steerEncoderGearing = 1;

    public SwerveModuleIOMAXSwerve (int index) {
        /** CAN IDs and encoder offsets are different for each module */
        switch (index) {
            /** Front Left */
            case 0: {
                driveSparkMax = new CANSparkFlex(3, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(2, MotorType.kBrushless);
                driveSparkMax.restoreFactoryDefaults();
                steerSparkMax.restoreFactoryDefaults();
                steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setInverted(true); // MAXSwerve has steer gearing reversed
                // revs -> radians
                steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing);
                steerEncoder.setZeroOffset(4.63282 - Units.degreesToRadians(90));
                break;
            }
            /** Front Right */
            case 1: {
                driveSparkMax = new CANSparkFlex(5, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(4, MotorType.kBrushless);
                driveSparkMax.restoreFactoryDefaults();
                steerSparkMax.restoreFactoryDefaults();
                steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setInverted(true); // MAXSwerve has steer gearing reversed
                // revs -> radians
                steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing);
                steerEncoder.setZeroOffset(1.1958 - Units.degreesToRadians(0));
                break;
            }
            /** Back Left */
            case 2: {
                driveSparkMax = new CANSparkFlex(7, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(6, MotorType.kBrushless);
                driveSparkMax.restoreFactoryDefaults();
                steerSparkMax.restoreFactoryDefaults();
                steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setInverted(true); // MAXSwerve has steer gearing reversed
                // revs -> radians
                steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing);
                steerEncoder.setZeroOffset(2.715878 + Units.degreesToRadians(180));
                break;
            }
            /** Back Right */
            case 3: {
                driveSparkMax = new CANSparkFlex(9, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(8, MotorType.kBrushless);
                driveSparkMax.restoreFactoryDefaults();
                steerSparkMax.restoreFactoryDefaults();
                steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                steerEncoder.setInverted(true); // MAXSwerve has steer gearing reversed
                // revs -> radians
                steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing);
                steerEncoder.setZeroOffset(4.708008 + Units.degreesToRadians(90));
                break;
            }
            default: throw new RuntimeException("Invalid SwerveModuleIOMAXSwerve index!");
        }

        /** Everything else is common between every swerve module */

        driveEncoder = driveSparkMax.getEncoder();
        //driveSparkMax.setInverted(false);
        //steerSparkMax.setInverted(false);

        driveSparkMax.setIdleMode(IdleMode.kBrake);
        steerSparkMax.setIdleMode(IdleMode.kBrake);
        driveSparkMax.enableVoltageCompensation(12.0);
        steerSparkMax.enableVoltageCompensation(12.0);
        driveSparkMax.setSmartCurrentLimit(40);
        steerSparkMax.setSmartCurrentLimit(20);
        driveSparkMax.setInverted(false);

        // rpm -> m/s
        driveEncoder.setVelocityConversionFactor(Math.PI * wheelDiameter / (60 * driveEncoderGearing));
        // revs -> meters
        driveEncoder.setPositionConversionFactor(Math.PI * wheelDiameter / driveEncoderGearing);

        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // motor position frame
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        //steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        //steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);

        if (Constants.robotMode == RobotMode.COMPETITION) {
            Timer.delay(0.2);
            driveSparkMax.burnFlash();
            steerSparkMax.burnFlash();
        }
    }

    public void updateInputs (SwerveModuleIOInputs inputs) {
        inputs.didLastError = didLastError();
        if (!didLastError()) {
            inputs.drivePosition = driveEncoder.getPosition();
            inputs.driveVelocity = driveEncoder.getVelocity();
            // Normalize [-pi, pi]
            inputs.steerAngle = MathUtil.angleModulus(steerEncoder.getPosition());
        } else {
            throw new Error("caught one?: " + driveEncoder.getPosition() + "\n" + MathUtil.angleModulus(steerEncoder.getPosition()));
        }
    }

    public boolean didLastError () {
        return driveSparkMax.getLastError() == REVLibError.kOk
            && steerSparkMax.getLastError() == REVLibError.kOk;
    }

    public void setDriveVoltage (double voltage) {
        driveSparkMax.setVoltage(voltage);
    }

    public void setSteerVoltage (double voltage) {
        steerSparkMax.setVoltage(voltage);
    }
}
