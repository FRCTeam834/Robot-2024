package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public static class ShooterIOInputs {
        public double topRollerVelocity;
        public double bottomRollerVelocity;
        public double pivotAngle;
        public double pivotVelocity;
        public double pivotAppliedVoltage;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    //public default void setTopRollerVoltage(double voltage) {}
    //public default void setBottomRollerVoltage(double voltage) {}
    public default void setRollerSpeeds (double speed) {}
    public default void setPivotVoltage(double voltage) {}
    public default void setRollerPID (double kP, double kI, double kD) {}
    public default void setRollerFeedforward (double kS, double kV) {}
    public default void setTopRollerVoltage(double volts) {}
    public default void setBottomRollerVoltage(double volts) {}
}
