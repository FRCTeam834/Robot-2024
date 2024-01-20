// Copyright 2024 Nick Bull
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    /**
     * "Inputs" are any data being sent from external source to robot code
     * (e.g. encoder information)
     */
    @AutoLog
    public static class IntakeIOInputs {
        public double rpm;
        public double voltage;
    }

    public default void updateInputs(IntakeIOInputs inputs){}

    // sets the voltate.
    public default void setVoltage(double volts){}

}