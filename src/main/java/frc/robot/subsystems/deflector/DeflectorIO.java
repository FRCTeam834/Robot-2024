package frc.robot.subsystems.deflector;

import org.littletonrobotics.junction.AutoLog;


public interface DeflectorIO {
    @AutoLog
    public static class DeflectorIOInputs {
        public double deflectorAngle;
        public double deflectorVelocity;
    }

    public default void updateInputs(DeflectorIOInputs inputs){}

    public default void setDeflectorVoltage(double voltage){}

}
