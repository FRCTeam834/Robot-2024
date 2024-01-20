package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drivetrain.ShooterIOInputsAutoLogged;
import frc.robot.utility.LoggedTunableNumber;

public class Shooter {
    private final int index;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // ! ¿¿¿Qué está pasando aquí???
    private static final LoggedTunableNumber pivotkP = new LoggedTunableNumber("shooter/Shooter/pivotkP");

    private final ArmFeedforward feedForward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
    private final PIDController pidController = new PIDController(0.0, 0.0, 0.0, 0.0);

    private double desiredAngle = 0.0;

    static {
        pivotkP.initDefault(0.0);
    }

    public Shooter (ShooterIO io, int index) {
        this.io = io;
        this.index = index;

        // * Q:Is this still used for the pivot? A: Creo que sí
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic () {
        io.updateInputs(inputs);
        Logger.processInputs("shooter/Shooter" + Integer.toString(index), inputs);

        if (pivotkP.hasChanged(hashCode())) {
            pidController.setPID(pivotkP.get(), 0.0, 0.0);
        }

        io.setPivotVoltage(feedForward.calculate(inputs.pivotAngle, inputs.pivotVelocity) + pidController.calculate(inputs.pivotAngle, desiredAngle));

    }

    public void shootNote () {
        io.setShooterVoltage(ShooterConstants.MAX_SHOOTER_SPEED);
    }

    public void stopShooters () {
        io.setShooterVoltage(0.0);
    }

    public void pivotToAngle (Rotation2d angle) {
        // * Angle converted to voltage in periodic
        desiredAngle = angle.getRotations();
    }
    
}