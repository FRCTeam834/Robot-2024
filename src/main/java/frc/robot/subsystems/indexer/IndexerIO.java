package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

//import frc.robot.subsystems.indexer....

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double  roller2Velocity;
        public double  roller3Velocity;
        public boolean noteIsDetected;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setRollerVoltage(double voltage) {}
    public default void notifyIndexer(IndexerIOInputs objectDetected) {} //modify "holdingNote" in Indexer.java

}