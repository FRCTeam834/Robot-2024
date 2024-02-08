// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class NoteDetectionIOPhotonVision implements NoteDetectionIO{
    String name;
    PhotonCamera camera;
    Transform3d robotToCamera;

    public NoteDetectionIOPhotonVision(String name){
        this.name = name;
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(){
        var inputs = new NoteDetectionIOInputs();
        inputs.rotationToNote = getRotationToNote();
    }

    @Override
    public double getRotationToNote(){
        var result = camera.getLatestResult();
        if (!result.hasTargets()){
            return 0.0;
        }
        
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d cameraToNote = target.getBestCameraToTarget();
        Transform3d robotToNote = cameraToNote.plus(robotToCamera);
        return robotToNote.getRotation().getZ();
    }

    @Override
    public String getName(){
        return this.name;
    }

}
