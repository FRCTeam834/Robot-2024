// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;


public interface VisionIO {
    /**
     * "Inputs" are any data being sent from external source to robot code
     * (e.g. encoder information)
     */
    @AutoLog
    public class VisionIOInputs {
        // public Pose2d previousPose = new Pose2d(0, 0, new Rotation2d());
    }

    public Pose2d getEstimatedGlobalPose2d();

    public Optional<EstimatedRobotPose> updateEstimatedGlobalPose(Pose2d prevEstimatedRobotPose);

    /** Update inputs with new sensor readings */
    // public default void updateInputs (VisionIOInputs inputs) {}

    // public default Optional<EstimatedRobotPose> getEstimatedGlobalPose() {}

}
