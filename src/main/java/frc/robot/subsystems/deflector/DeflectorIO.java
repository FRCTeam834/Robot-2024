package frc.robot.subsystems.deflector;


public interface DeflectorIO {

    public static class DeflectorIOInputs {
        public double deflectorAngle;
        public double deflectorVelocity;
    }

    public default void updateInputs(DeflectorIOInputs inputs){}

    public default void setDeflectorVoltage(double voltage){}

    public default void stopDeflector(){}

}