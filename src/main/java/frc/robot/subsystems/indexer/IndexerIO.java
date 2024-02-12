package frc.robot.subsystems.indexer;

public interface IndexerIO {
    public static class IndexerIOInputs {
        public boolean noteIsDetectedFront; //Checking if the first sensor sees a note
        public boolean noteIsDetectedBack; //Checking if the second sensor sees a note
    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
}