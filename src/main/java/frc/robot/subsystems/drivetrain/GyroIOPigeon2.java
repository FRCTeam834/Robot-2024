package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    
    public GyroIOPigeon2 () {
        pigeon = new Pigeon2(0);
        pigeon.setYaw(0.0);
    }

    public void updateInputs (GyroIOInputs inputs) {
        inputs.yaw = pigeon.getYaw().getValue();
    }
}
