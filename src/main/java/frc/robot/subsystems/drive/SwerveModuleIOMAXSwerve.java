package frc.robot.subsystems.drive;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

public class SwerveModuleIOMAXSwerve implements SwerveModuleIO {
    private final CANSparkFlex driveSparkMax;
    private final CANSparkMax steerSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder steerEncoder;

    private final double wheelDiameter = Units.inchesToMeters(3.0);
    private final double driveEncoderGearing = 4.71;
    private final double steerEncoderGearing = 1;

    public SwerveModuleIOMAXSwerve (int index) {
        /** CAN IDs and encoder offsets are different for each module */
        
        switch (index) {
            /** Front Left */
            case 0: {
                driveSparkMax = new CANSparkFlex(3, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(2, MotorType.kBrushless);
                configureSpark("", () -> { return driveSparkMax.restoreFactoryDefaults(); });
                configureSpark("", () -> { return steerSparkMax.restoreFactoryDefaults(); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); });
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                configureSpark("", () -> { return steerEncoder.setInverted(true); }); // MAXSwerve has steer gearing reversed
                // revs -> radians
                configureSpark("", () -> { return steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerEncoder.setZeroOffset(4.6345182 - Units.degreesToRadians(90)); });
                break;
            }
            /** Front Right */
            case 1: {
                driveSparkMax = new CANSparkFlex(5, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(4, MotorType.kBrushless);
                configureSpark("", () -> { return driveSparkMax.restoreFactoryDefaults(); });
                configureSpark("", () -> { return steerSparkMax.restoreFactoryDefaults(); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); });
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                configureSpark("", () -> { return steerEncoder.setInverted(true); }); // MAXSwerve has steer gearing reversed
                // revs -> radians
                configureSpark("", () -> { return steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerEncoder.setZeroOffset(1.1953152 - Units.degreesToRadians(0)); });
                break;
            }
            /** Back Left */
            case 2: {
                driveSparkMax = new CANSparkFlex(7, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(6, MotorType.kBrushless);
                configureSpark("", () -> { return driveSparkMax.restoreFactoryDefaults(); });
                configureSpark("", () -> { return steerSparkMax.restoreFactoryDefaults(); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); });
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                configureSpark("", () -> { return steerEncoder.setInverted(true); }); // MAXSwerve has steer gearing reversed
                // revs -> radians
                configureSpark("", () -> { return steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerEncoder.setZeroOffset(2.7099678 + Units.degreesToRadians(180)); });
                break;
            }
            /** Back Right */
            case 3: {
                driveSparkMax = new CANSparkFlex(9, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(8, MotorType.kBrushless);
                configureSpark("", () -> { return driveSparkMax.restoreFactoryDefaults(); });
                configureSpark("", () -> { return steerSparkMax.restoreFactoryDefaults(); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); });
                steerEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                configureSpark("", () -> { return steerEncoder.setInverted(true); }); // MAXSwerve has steer gearing reversed
                // revs -> radians
                configureSpark("", () -> { return steerEncoder.setPositionConversionFactor(2 * Math.PI / steerEncoderGearing); });
                Timer.delay(0.2);
                configureSpark("", () -> { return steerEncoder.setZeroOffset(4.7144107 + Units.degreesToRadians(90)); });
                break;
            }
            default: throw new RuntimeException("Invalid SwerveModuleIOMAXSwerve index!");
        }

        /** Everything else is common between every swerve module */

        driveEncoder = driveSparkMax.getEncoder();
        //driveSparkMax.setInverted(false);
        //steerSparkMax.setInverted(false);

        configureSpark("", () -> { return driveSparkMax.setIdleMode(IdleMode.kBrake); });
        configureSpark("", () -> { return steerSparkMax.setIdleMode(IdleMode.kBrake); });
        configureSpark("", () -> { return driveSparkMax.enableVoltageCompensation(12.0); });
        configureSpark("", () -> { return steerSparkMax.enableVoltageCompensation(12.0); });
        configureSpark("", () -> { return driveSparkMax.setSmartCurrentLimit(40); });
        configureSpark("", () -> { return steerSparkMax.setSmartCurrentLimit(20); });
        driveSparkMax.setInverted(false);

        // rpm -> m/s
        configureSpark("", () -> { return driveEncoder.setVelocityConversionFactor(Math.PI * wheelDiameter / (60 * driveEncoderGearing)); });
        // revs -> meters
        configureSpark("", () -> { return driveEncoder.setPositionConversionFactor(Math.PI * wheelDiameter / driveEncoderGearing); });

        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        configureSpark("", () -> { return driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); }); // motor position frame
        configureSpark("", () -> { return driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); });
        configureSpark("", () -> { return driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); });
        configureSpark("", () -> { return driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); });
        configureSpark("", () -> { return driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); });

        configureSpark("", () -> { return steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); });
        configureSpark("", () -> { return steerSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); });

        if (Constants.robotMode == RobotMode.COMPETITION) {
            Timer.delay(0.25);
            configureSpark("", () -> { return driveSparkMax.burnFlash(); });
            Timer.delay(0.25);
            configureSpark("", () -> { return steerSparkMax.burnFlash(); });
            Timer.delay(0.25);
        }
    }

    public static boolean configureSpark(String message, Supplier<REVLibError> config) {
        REVLibError err = REVLibError.kOk;
        for (int i = 0; i < 15; i++) {
            err = config.get();
            if (err == REVLibError.kOk) {
                return true;
            }
            Timer.delay(0.04);
        }

        DriverStation.reportError(String.format(
            "[MergeError] - CANSparkMax failed to configure setting. MergeMessage: %s. Spark error code: %s \nSee stack trace below.", 
            message,
            err.toString()), 
            true);
            
        return false;
    }

    public static boolean sparkGet(String message, Supplier<REVLibError> config) {
        REVLibError err = REVLibError.kOk;
        for (int i = 0; i < 5; i++) {
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

    public void updateInputs (SwerveModuleIOInputs inputs) {
        double lastPosition = inputs.drivePosition;
        inputs.drivePosition = driveEncoder.getPosition();
        if (driveSparkMax.getLastError() != REVLibError.kOk) {
            inputs.drivePosition = lastPosition;
        }


        double lastVelocity = inputs.driveVelocity;
        inputs.driveVelocity = driveEncoder.getVelocity();
        if (driveSparkMax.getLastError() != REVLibError.kOk) {
            inputs.driveVelocity = lastVelocity;
        }

        double lastSteerAngle = inputs.steerAngle;
        inputs.steerAngle = MathUtil.angleModulus(steerEncoder.getPosition());
        if (steerSparkMax.getLastError() != REVLibError.kOk) {
            // Normalize [-pi, pi]
            inputs.steerAngle = lastSteerAngle;
        }
    }

    public void setDriveVoltage (double voltage) {
        driveSparkMax.setVoltage(voltage);
    }

    public void setSteerVoltage (double voltage) {
        steerSparkMax.setVoltage(voltage);
    }
}
