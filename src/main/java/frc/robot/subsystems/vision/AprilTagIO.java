// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;




import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface AprilTagIO {
    @AutoLog
    class AprilTagIOInputs {
        Pose3d poseEstimate3d = new Pose3d();
        double timestamp = 0;

    }
    
    default void updateInputs(AprilTagIOInputs inputs) {}

    default String getName() {
        return "";
    }
} 
