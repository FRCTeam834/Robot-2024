package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2 () {
        pigeon = new Pigeon2(12);
        pigeon.setYaw(0.0);
    }

    public void updateInputs (GyroIOInputs inputs) {
        inputs.yaw = pigeon.getYaw().getValue();
    }

    public void resetYaw(double angle){
        pigeon.setYaw(angle);
    }
}