package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yaw = 0.0;
    }
}