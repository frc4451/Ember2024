package frc.robot.subsystems.vision.object_detection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.VisionConstants.VisionSource;

public class ObjectDetectionPhoton implements ObjectDetectionIO {
    private final PhotonCamera camera;

    public ObjectDetectionPhoton(VisionSource source) {
        camera = new PhotonCamera(source.name());
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        PhotonPipelineResult frame = camera.getLatestResult();
        inputs.frame = frame;

    }
}
