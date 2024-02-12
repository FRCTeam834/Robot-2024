package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.utility.TunableNumber;

public class SwerveModule {
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

    static {
        drivekS.initDefault(0.31437);
        drivekV.initDefault(0.7);
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

        // Update if tunable numbers have changed
        if (drivekP.hasChanged()) {
            driveController.setPID(drivekP.get(), 0.0, 0.0);
        }
        if (steerkP.hasChanged()) {
            steerController.setPID(steerkP.get(), 0.0, 0.0);
        }
        if (drivekS.hasChanged() || drivekV.hasChanged()) {
            driveFeedforward = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
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

    public SwerveModulePosition getPosition () {
        return new SwerveModulePosition(inputs.drivePosition, getAngle());
    }

    public void stop () {
        io.setDriveVoltage(0.0);
        io.setSteerVoltage(0.0);
    }
}
