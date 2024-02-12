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
    public void updateInputs(NoteDetectionIOInputs inputs){
        inputs.rotationToNote = getRotationToNote();
    }

    @Override
    public Double getRotationToNote(){
        var result = camera.getLatestResult();
        if (!result.hasTargets()){
            return null;
        }
        
        /** !Remember to set target sort mode to closest */
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getYaw();
    }

    @Override
    public String getName(){
        return this.name;
    }

}