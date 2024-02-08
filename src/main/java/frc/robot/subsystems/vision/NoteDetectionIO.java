// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface NoteDetectionIO {

    @AutoLog
    public class NoteDetectionIOInputs {
        double rotationToNote = 0.0;
    }
    default void updateInputs() {}

    default String getName(){
        return "";
    }

    default double getRotationToNote(){
        return 0.0;
    }
    

}
