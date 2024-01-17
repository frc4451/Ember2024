package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AprilTagIO {
    @AutoLog
    public static class AprilTagIOInputs {
        public PhotonPipelineResult frame = new PhotonPipelineResult();
    }

    public default void updateInputs(AprilTagIOInputs inputs) {
    }

    public default Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult frame) {
        return Optional.empty();
    }
}
