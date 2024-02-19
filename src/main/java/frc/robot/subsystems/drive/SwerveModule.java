package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.utility.TunableNumber;

public class SwerveModule extends SubsystemBase {
    private final int index;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    private static final TunableNumber drivekP = new TunableNumber("drive/SwerveModule/drivekP");
    private static final TunableNumber steerkP = new TunableNumber("drive/SwerveModule/steerkP");

    private static final TunableNumber drivekS = new TunableNumber("drive/SwerveModule/drivekS");
    private static final TunableNumber drivekV = new TunableNumber("drive/SwerveModule/drivekV");

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private final PIDController driveController = new PIDController(0, 0, 0);
    private final PIDController steerController = new PIDController(0, 0, 0);

    private SwerveModuleState setpoint = new SwerveModuleState();

    static {
        drivekS.initDefault(0.31437);
        drivekV.initDefault(2);
        drivekP.initDefault(1);
        steerkP.initDefault(1);
    }

    public SwerveModule (SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        steerController.enableContinuousInput(0, Math.PI * 2);
        SmartDashboard.putData("SwerveModule " + index, this);
    }

    public void periodic () {
        io.updateInputs(inputs);

        // Update if tunable numbers have changed
        if (drivekP.hasChanged(hashCode())) {
            driveController.setPID(drivekP.get(), 0.0, 0.0);
        }
        if (steerkP.hasChanged(hashCode())) {
            steerController.setPID(steerkP.get(), 0.0, 0.0);
        }
        if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
            driveFeedforward = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
        }
    }

    public void setDesiredState (SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());
        // Don't drive if speed is too low to prevent jittering
        if (Math.abs(optimizedState.speedMetersPerSecond) < 1e-3) {
            optimizedState.speedMetersPerSecond = 0;
        }

        // Scale by cosine of angle error, to reduce movement in perpendicular
        // direction of desired while steering catches up
        optimizedState.speedMetersPerSecond *= Math.cos(steerController.getPositionError());

        io.setDriveVoltage(
            driveFeedforward.calculate(optimizedState.speedMetersPerSecond) +
            driveController.calculate(inputs.driveVelocity, optimizedState.speedMetersPerSecond));
        io.setSteerVoltage(
            steerController.calculate(inputs.steerAngle, optimizedState.angle.getRadians()));

        setpoint = optimizedState;
    }

    public Rotation2d getAngle () {
        return new Rotation2d(getAngleRadians());
    }

    public double getAngleRadians () {
        return inputs.steerAngle;
    }

    public double getSpeed () {
        return inputs.driveVelocity;
    }

    public SwerveModulePosition getPosition () {
        return new SwerveModulePosition(inputs.drivePosition, getAngle());
    }

    public SwerveModuleState getState () {
        return new SwerveModuleState(inputs.driveVelocity, getAngle());
    }

    public void stop () {
        io.setDriveVoltage(0.0);
        io.setSteerVoltage(0.0);
    }

    public double getSetpointSpeed () {
        return setpoint.speedMetersPerSecond;
    }

    public double getSetpointAngle () {
        return setpoint.angle.getRadians();
    }

    @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("SwerveModule" + index);

    builder.addDoubleProperty("Setpoint Speed", this::getSetpointSpeed, null);
    builder.addDoubleProperty("Setpoint Angle", this::getSetpointAngle, null);
    builder.addDoubleProperty("Speed", this::getSpeed, null);
    builder.addDoubleProperty("Angle", this::getAngleRadians, null);
    builder.addDoubleProperty("PID Voltage", () -> driveController.calculate(inputs.driveVelocity, setpoint.speedMetersPerSecond), null);
  }
}
