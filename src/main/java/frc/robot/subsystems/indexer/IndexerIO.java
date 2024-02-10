package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

//import frc.robot.subsystems.indexer....

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double  roller2Velocity;
        public double  roller3Velocity;
        public boolean noteIsDetectedFront; //Checking if the first sensor sees a note
        public boolean noteIsDetectedBack; //Checking if the second sensor sees a note
    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setRollerVoltage(double voltage) {}
}