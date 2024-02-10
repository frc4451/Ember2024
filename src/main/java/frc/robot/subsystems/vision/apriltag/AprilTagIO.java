package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AprilTagIO {
    @AutoLog
    public static class AprilTagIOInputs {
        public PhotonPipelineResult frame = new PhotonPipelineResult();
        public boolean isDuplicateFrame = false;

        public EstimatedPose estimatedPose = new EstimatedPose();
    }

    public default void updateInputs(AprilTagIOInputs inputs) {
    }
}
