package frc.robot.subsystems.drive;

public interface SwerveModuleIO {
    public static class SwerveModuleIOInputs {
        public double drivePosition = 0.0;
        public double driveVelocity = 0.0;
        public double steerAngle = 0.0;
    }

    /** Update inputs with new sensor readings */
    public default void updateInputs (SwerveModuleIOInputs inputs) {}
    /** Set desired voltage for drive controller */
    public default void setDriveVoltage (double voltage) {}
    /** Set desired voltage for steer controller */
    public default void setSteerVoltage (double voltage) {}
}
