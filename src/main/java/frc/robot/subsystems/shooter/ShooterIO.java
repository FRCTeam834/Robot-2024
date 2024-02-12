package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public static class ShooterIOInputs {
        public double topRollerVelocity;
        public double bottomRollerVelocity;
        public double pivotAngle;
        public double pivotVelocity;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setTopRollerVoltage(double voltage) {}
    public default void setBottomRollerVoltage(double voltage) {}
    public default void setPivotVoltage(double voltage) {}
}
