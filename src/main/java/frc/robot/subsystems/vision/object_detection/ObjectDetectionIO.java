package frc.robot.subsystems.vision.object_detection;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface ObjectDetectionIO {
    @AutoLog
    public static class ObjectDetectionIOInputs {
        public PhotonPipelineResult frame = new PhotonPipelineResult();
    }

    public default void updateInputs(ObjectDetectionIOInputs inputs) {
    }
}
