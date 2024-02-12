package frc.robot.subsystems.intake;

public interface IntakeIO {
    public static class IntakeIOInputs {}

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
}