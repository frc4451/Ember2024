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

    public default void updateInputs() {
    }

    public default void periodic() {
    };

    public default Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return Optional.empty();
    }
}
