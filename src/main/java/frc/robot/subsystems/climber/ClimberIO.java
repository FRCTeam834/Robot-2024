package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
   
    @AutoLog
    public static class ClimberIOInputs {
        public double rightSwerveVelocity;
        public double leftSwerveVelocity;
        public double rightArmHeight; //current height of the right arm (so it doesn't break)
        public double leftArmHeight; //basically the same thing but the other arm
    }

    public default void updateInputs(ClimberIOInputs inputs) {}
    public default void setSwerveVelocity(double rightVoltage, double leftVoltage) {}
}
