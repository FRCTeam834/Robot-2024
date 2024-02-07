package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;
import frc.robot.utility.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Obligatory DIO comment
    private static final LoggedTunableNumber pivotkP = new LoggedTunableNumber("shooter/Shooter/pivotkP");
    private static final LoggedTunableNumber topRollerkP = new LoggedTunableNumber("shooter/Shooter/topRollerkP");
    private static final LoggedTunableNumber bottomRollerkP = new LoggedTunableNumber("shooter/Shooter/bottomRollerkP");
    
    private static final LoggedTunableNumber topRollerkS = new LoggedTunableNumber("shooter/Shooter/topRollerkS");
    private static final LoggedTunableNumber bottomRollerkS = new LoggedTunableNumber("shooter/Shooter/bottomRollerkS");
    
    private static final LoggedTunableNumber topRollerkV = new LoggedTunableNumber("shooter/Shooter/topRollerkV");
    private static final LoggedTunableNumber bottomRollerkV = new LoggedTunableNumber("shooter/Shooter/bottomRollerkV");

    private final ArmFeedforward pivotFeedForward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
    private final PIDController pivotPID = new PIDController(0.0, 0.0, 0.0, 0.02);

    private final PIDController topRollerPID = new PIDController(0.0, 0.0, 0.0, 0.02);
    private final PIDController bottomRollerPID = new PIDController(0.0, 0.0, 0.0, 0.02);

    private SimpleMotorFeedforward topRollerFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
    private SimpleMotorFeedforward bottomRollerFeedForward = new SimpleMotorFeedforward(0.0, 0.0);

    private double desiredAngle = 0.0;
    private double desiredRollerSpeeds = 0.0;

    static {
        pivotkP.initDefault(0.0);
        topRollerkP.initDefault(0.0);
        bottomRollerkP.initDefault(0.0);
        topRollerkS.initDefault(0.05);
        bottomRollerkS.initDefault(0.05);
        topRollerkV.initDefault(0.003);
        bottomRollerkV.initDefault(0.003);
    }

    public Shooter (ShooterIO io) {
        this.io = io;

        // * Q:Is this still used for the pivot? A: Creo que sí
        pivotPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic () {
        io.updateInputs(inputs);
        Logger.processInputs("shooter/Shooter", inputs);

        if (pivotkP.hasChanged(hashCode())) {
            pivotPID.setPID(pivotkP.get(), 0.0, 0.0);
        }
        if (topRollerkP.hasChanged(hashCode())) {
            topRollerPID.setPID(topRollerkP.get(), 0.0, 0.0);
        }
        if (bottomRollerkP.hasChanged(hashCode())) {
            bottomRollerPID.setPID(bottomRollerkP.get(), 0.0, 0.0);
        }
        if (topRollerkS.hasChanged(hashCode()) || topRollerkV.hasChanged(hashCode())) {
            topRollerFeedForward = new SimpleMotorFeedforward(topRollerkS.get(), topRollerkV.get());
        }
        if (bottomRollerkS.hasChanged(hashCode()) || bottomRollerkV.hasChanged(hashCode())) {
            bottomRollerFeedForward = new SimpleMotorFeedforward(bottomRollerkS.get(), bottomRollerkV.get());
        }

        io.setPivotVoltage(pivotFeedForward.calculate(desiredAngle, 0.0) + pivotPID.calculate(inputs.pivotAngle, desiredAngle));
        io.setTopRollerVoltage(topRollerFeedForward.calculate(desiredRollerSpeeds) + topRollerPID.calculate(inputs.roller0Velocity, desiredRollerSpeeds));
        io.setBottomRollerVoltage(bottomRollerFeedForward.calculate(desiredRollerSpeeds) + bottomRollerPID.calculate(inputs.roller1Velocity, desiredRollerSpeeds));

    }
    
    public void stopShooters () {
        io.setTopRollerVoltage(0.0);
        io.setBottomRollerVoltage(0.0);
    }

    public void pivotToAngle (Rotation2d angle) {
        // * Angle converted to voltage in periodic
        desiredAngle = angle.getRadians();
    }

    public void setDesiredRollerSpeeds (double speeds) {
        desiredRollerSpeeds = speeds;
    }
}