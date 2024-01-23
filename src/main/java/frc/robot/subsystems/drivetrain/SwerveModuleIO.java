package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    /**
     * "Inputs" are any data being sent from external source to robot code
     * (e.g. encoder information)
     */
    @AutoLog
    public static class SwerveModuleIOInputs {
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
