package frc.robot.subsystems.vision.object_detection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.VisionConstants.VisionSource;

public class ObjectDetectionPhoton implements ObjectDetectionIO {
    private final PhotonCamera camera;

    private final Thread periodicThread = new Thread(() -> {
        while (!Thread.currentThread().isInterrupted()) {
            periodic();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    });

    public ObjectDetectionPhoton(VisionSource source) {
        camera = new PhotonCamera(source.name());

        periodicThread.setPriority(Thread.MAX_PRIORITY);
        periodicThread.start();
    }

    private PhotonPipelineResult frame = new PhotonPipelineResult();

    private void periodic() {
        frame = camera.getLatestResult();
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.frame = frame;
    }
}
