package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utility.LoggedTunableNumber;

public class SwerveModule {
    private final int index;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drivetrain/SwerveModule/DrivekP");
    private static final LoggedTunableNumber steerkP = new LoggedTunableNumber("Drivetrain/SwerveModule/SteerkP");

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.31437, 0.7);
    private final PIDController driveController = new PIDController(0.0, 0.0, 0.0,0.02);
    private final PIDController steerController = new PIDController(0.0, 0.0, 0.0,0.02);

    static {
        drivekP.initDefault(0.4);
        steerkP.initDefault(0.6);
    }

    public SwerveModule (SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic () {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/SwerveModule" + Integer.toString(index), inputs);

        // Update if tunable numbers have changed
        if (drivekP.hasChanged(hashCode())) {
            driveController.setPID(drivekP.get(), 0.0, 0.0);
        }
        if (steerkP.hasChanged(hashCode())) {
            steerController.setPID(steerkP.get(), 0.0, 0.0);
        }
    }

    public void setDesiredState (SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());
        // optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.getPositionError());
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
            driveController.calculate(inputs.steerAngle, optimizedState.angle.getRadians()));
    }

    public Rotation2d getAngle () {
        return new Rotation2d(MathUtil.angleModulus(inputs.steerAngle));
    }

    public void stop () {
        io.setDriveVoltage(0.0);
        io.setSteerVoltage(0.0);
    }
}
