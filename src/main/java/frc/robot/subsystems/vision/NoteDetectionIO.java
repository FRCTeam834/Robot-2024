package frc.robot.subsystems.vision;

/** Add your docs here. */
public interface NoteDetectionIO {

    public class NoteDetectionIOInputs {
        Double rotationToNote = 0.0;
    }
    default void updateInputs(NoteDetectionIOInputs inputs) {}

    default boolean isConnected () { return false; }

    default String getName(){
        return "";
    }

    default Double getRotationToNote(){
        return null;
    }
    

}