package frc.robot.subsystems.drive;

public interface GyroIO {
    public static class GyroIOInputs {
        public double yaw = 0.0;
    }

    public default void updateInputs (GyroIOInputs inputs) {}
}